#!/usr/bin/env python3
# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
"""Sweep core configurations over the benchmarks: the runs of a design space
exploration.

    ./util/sweep.py <sweep.toml> [--build-jobs N] [--jobs N] [--rebuild] [--reuse-runs] [--dry-run]
                    [--runner CMD [--points N] [--verilator-jobs N]]

A sweep file names a base configuration and the points to run:

    name = "example"
    config = "cv64a6_splus"            # base configuration, as CVA6_CONFIG
    perf = true                        # build with cycle accounting (PERF=1)
    benchmarks = ["coremark", "em-"]   # optional: names containing one of these
    max_cycles = 200000000             # optional

    [[point]]                          # points, in order: the first is the baseline
    label = "baseline"

    [[point]]
    label = "fast-loads"
    params = { NrLoadPipeRegs = 0 }    # fields of the configuration's cva6_cfg
    sim_args = "+MemDelay=0"           # plusargs for all the point's simulations

    [grid]                             # and every combination of these, after the points
    NrLoadPipeRegs = [0, 1]
    BHTEntries = [64, 128]
    sim_args = ["", "+MemDelay=0"]

A parameter's value is a SystemVerilog expression, cast like the value of the
field it replaces. Every distinct set of parameters is one model: the base
configuration package with those fields replaced, generated under the sweep's
directory and built once. Points that differ only in simulation arguments share
it, and so do later invocations, as long as the package, the build options and
the tracked sources are unchanged. Models are built with $VERILATOR, else with
Verilator under $SIM_ENV, the command prefix of the environment Verilator
simulations are built and run in (empty by default; util/iis-env.sh sets it),
and each point is run with run-bench.py under the same prefix; then all of them
are compared with compare-runs.py.

Everything goes to verilator/bench/sweeps/<name>/: models/<model>/ (package and
build), runs/<point>/ (run-bench.py's results), compare/ (compare-runs.py's),
and sweep.json, which says what each point is. Runs in the uv environment
(see ../README.md): uv run ./util/sweep.py <sweep.toml>.

The builds and the runs can go to other hosts, through a runner: a command
line, --runner or $SWEEP_RUNNER, that runs one job elsewhere as

    <runner> --cores N -- <command> [<argument>...]

It must run the command where this checkout is reachable (the arguments hold
its absolute paths as this host sees them; mapping them is the runner's
business), wait for it, print its output and exit with its status. Then every
model is built at once, on --verilator-jobs cores each (default 8), and every
point runs at once, -j simulations each (default 16), as the runner finds room
for them; --build-jobs and --points limit how many are in flight.
"""

import argparse
import concurrent.futures
import datetime
import hashlib
import itertools
import json
import os
import re
import shlex
import subprocess
import sys
import tomllib
from pathlib import Path

TEST_DIR = Path(__file__).resolve().parent.parent
ROOT_DIR = TEST_DIR.parent
SIM_ENV = os.environ.get("SIM_ENV", "").split()
# The Verilator that builds the models: $VERILATOR, else the one under $SIM_ENV
VERILATOR = os.environ.get("VERILATOR") or shlex.join([*SIM_ENV, "verilator"])
# Simulations per point and Verilator jobs per model, with a runner
RUNNER_JOBS = 16
RUNNER_VERILATOR_JOBS = 8


def label_dir(label):
    return re.sub(r"[^A-Za-z0-9_.+=,-]", "_", label)


def expand_points(sweep):
    """The sweep's points: [(label, params, sim_args)], explicit ones first."""
    points = []
    for p in sweep.get("point", []):
        if "label" not in p:
            sys.exit("error: every [[point]] needs a label")
        unknown = set(p) - {"label", "params", "sim_args"}
        if unknown:
            sys.exit(f"error: point {p['label']}: unknown keys {sorted(unknown)}")
        points.append((p["label"], {k: str(v) for k, v in p.get("params", {}).items()},
                       p.get("sim_args", "")))
    grid = dict(sweep.get("grid", {}))
    if grid:
        sim_args = grid.pop("sim_args", [""])
        keys = list(grid)
        for values in itertools.product(*(grid[k] for k in keys), sim_args):
            params = {k: str(v) for k, v in zip(keys, values[:-1])}
            parts = [f"{k}={v}" for k, v in params.items()] + ([values[-1]] if values[-1] else [])
            points.append((",".join(parts) or "grid", params, values[-1]))
    if not points:
        sys.exit("error: the sweep has no [[point]] and no [grid]")
    labels = [label for label, _, _ in points]
    dupes = sorted({l for l in labels if labels.count(l) > 1})
    if dupes:
        sys.exit(f"error: points with the same label: {', '.join(dupes)}")
    return points


def generate_package(config, params):
    """The configuration package of config with the cva6_cfg fields in params
    replaced. Each must be a field of cva6_cfg, on a line of its own."""
    base = ROOT_DIR / "config" / "target" / config / "rtl_cfg_pkg.sv"
    if not base.is_file():
        sys.exit(f"error: no configuration package {base}")
    lines = base.read_text().splitlines(keepends=True)
    start = next((i for i, l in enumerate(lines) if re.search(r"\bcva6_cfg\s*=\s*'\{", l)), None)
    if start is None:
        sys.exit(f"error: {base}: no cva6_cfg assignment")
    end = next(i for i in range(start, len(lines)) if re.match(r"\s*\};", lines[i]))
    for field, value in params.items():
        pattern = re.compile(rf"^(\s*{re.escape(field)}\s*:\s*)(.+?)([ \t]*,?)([ \t]*//.*)?(\n?)$")
        hits = [i for i in range(start, end) if pattern.match(lines[i])]
        if len(hits) != 1:
            sys.exit(f"error: {field} is {'not a' if not hits else 'more than one'} field of "
                     f"{config}'s cva6_cfg")
        m = pattern.match(lines[hits[0]])
        cast = re.match(r"(\w+)'\(.*\)$", m.group(2))
        new = f"{cast.group(1)}'({value})" if cast else value
        lines[hits[0]] = f"{m.group(1)}{new}{m.group(3)}  // sweep: was {m.group(2)}{m.group(5)}"
    header = (f"// Generated by util/sweep.py from {base.relative_to(ROOT_DIR)}, with "
              + (", ".join(f"{k} = {v}" for k, v in params.items()) or "no change") + "\n")
    return header + "".join(lines)


# The tracked files a model can depend on: not the benchmarks, the scripts or
# the documentation, so that changing those rebuilds no model
MODEL_SOURCES = [".", ":(exclude)test/util", ":(exclude)test/sw", ":(exclude)docs",
                 ":(exclude)*.md"]


def git(*args):
    return subprocess.run(["git", "-C", str(ROOT_DIR), *args], capture_output=True, text=True,
                          check=True).stdout


def tree_state():
    """What the sources of a model are: the tracked files' contents, committed
    or not."""
    return git("ls-files", "-s", "--", *MODEL_SOURCES) + git("diff", "HEAD", "--", *MODEL_SOURCES)


def build_model(model_dir, config, package, perf, verilator_jobs, rebuild, tree, runner):
    """Build the model of one parameter set, unless an identical one is there,
    here or through the runner. Returns (model_dir, verdict)."""
    key = hashlib.sha256(json.dumps([config, package, perf, tree]).encode()).hexdigest()
    pkg = model_dir / "cva6_config_pkg.sv"
    vtest = model_dir / "build" / "Vtest"
    log = model_dir / "logs" / "build.log"
    key_file = model_dir / "build.key"
    if (not rebuild and key_file.is_file() and key_file.read_text() == key and vtest.is_file()
            and log.is_file() and "%Error" not in log.read_text()):
        return model_dir, "reused"
    model_dir.mkdir(parents=True, exist_ok=True)
    pkg.write_text(package)
    key_file.unlink(missing_ok=True)
    subprocess.run(["make", "-C", str(TEST_DIR), "verilator-clean", f"VERILATOR_DIR={model_dir}"],
                   capture_output=True)
    cmd = ["make", "-C", str(TEST_DIR), "verilator-build", f"CVA6_CONFIG={config}",
           f"CVA6_CONFIG_PKG={pkg}", f"VERILATOR={VERILATOR}", f"VERILATOR_DIR={model_dir}",
           "THREADS=1", f"PERF={1 if perf else 0}", f"VERILATOR_JOBS={verilator_jobs}"]
    if runner:
        cmd = [*runner, "--cores", str(verilator_jobs), "--", *cmd]
    with open(model_dir / "make.log", "w") as f:
        subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT)
    flist = model_dir / "cva6tb.flist"
    if not flist.is_file():
        # make stopped before generating it: say why
        text = (model_dir / "make.log").read_text(errors="replace")
        why = next((l.strip() for l in text.splitlines() if "*** " in l or "%Error" in l), "")
        return model_dir, f"FAIL: {why or 'no file list'} (see {model_dir / 'make.log'})"
    # The package as the host that built it names it: another path through a runner
    used = f"/{model_dir.name}/{pkg.name}"
    if not any(l == str(pkg) or l.endswith(used) for l in flist.read_text().splitlines()):
        return model_dir, f"FAIL: the file list does not use {pkg} (see make.log)"
    if not vtest.is_file() or not log.is_file() or "%Error" in log.read_text():
        return model_dir, f"FAIL: see {log if log.is_file() else model_dir / 'make.log'}"
    key_file.write_text(key)
    return model_dir, "built"


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("sweep", help="the sweep file (TOML)")
    parser.add_argument("--build-jobs", type=int,
                        help="models built at once (default 2, with a runner all)")
    parser.add_argument("-j", "--jobs", type=int,
                        help="simulations at once within a run (default: all cores, with a runner "
                        f"{RUNNER_JOBS})")
    parser.add_argument("--runner", default=os.environ.get("SWEEP_RUNNER", ""),
                        help="the command line that runs a build or a run elsewhere, see above "
                        "(default: $SWEEP_RUNNER; none: here)")
    parser.add_argument("--points", type=int,
                        help="points run at once (default 1, with a runner all)")
    parser.add_argument("--verilator-jobs", type=int,
                        help="Verilator jobs per model (default: the cores over --build-jobs, with "
                        f"a runner {RUNNER_VERILATOR_JOBS})")
    parser.add_argument("--rebuild", action="store_true", help="build every model again")
    parser.add_argument("--reuse-runs", action="store_true",
                        help="keep the results of points already run, rather than run them again")
    parser.add_argument("--dry-run", action="store_true", help="only show the plan")
    args = parser.parse_args()

    sweep = tomllib.loads(Path(args.sweep).read_text())
    for key in ("name", "config"):
        if key not in sweep:
            sys.exit(f"error: the sweep file needs '{key}'")
    unknown = set(sweep) - {"name", "config", "perf", "benchmarks", "max_cycles", "point", "grid"}
    if unknown:
        sys.exit(f"error: unknown keys {sorted(unknown)}")
    config, perf = sweep["config"], bool(sweep.get("perf", True))
    points = expand_points(sweep)
    out = TEST_DIR / "verilator" / "bench" / "sweeps" / label_dir(sweep["name"])

    # One model per distinct parameter set, named after it
    models = {}
    for _, params, _ in points:
        key = json.dumps(params, sort_keys=True)
        if key not in models:
            name = ",".join(f"{k}={v}" for k, v in sorted(params.items())) or "base"
            models[key] = {"params": params, "dir": out / "models" / label_dir(name)}
    packages = {k: generate_package(config, m["params"]) for k, m in models.items()}

    print(f"sweep {sweep['name']}: {len(points)} points, {len(models)} models of {config}"
          f"{' with cycle accounting' if perf else ''}, in {out}")
    for label, params, sim_args in points:
        print(f"  {label}: {', '.join(f'{k}={v}' for k, v in params.items()) or 'base'}"
              f"{'  ' + sim_args if sim_args else ''}")
    if args.dry_run:
        for k, m in models.items():
            print(f"--- {m['dir'] / 'cva6_config_pkg.sv'}, changed lines:")
            for line in packages[k].splitlines():
                if "// sweep: was" in line:
                    print("  " + line.strip())
        return 0

    manifest = {"name": sweep["name"], "config": config, "perf": perf,
                "date": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
                "sweep_file": str(Path(args.sweep).resolve()), "points": []}
    for label, params, sim_args in points:
        model = models[json.dumps(params, sort_keys=True)]
        manifest["points"].append({"label": label, "params": params, "sim_args": sim_args,
                                   "model": str(model["dir"]),
                                   "run": str(out / "runs" / label_dir(label))})

    # Build
    tree = tree_state()
    runner = shlex.split(args.runner)
    build_jobs = args.build_jobs or (len(models) if runner else 2)
    verilator_jobs = args.verilator_jobs or (
        RUNNER_VERILATOR_JOBS if runner else max(1, os.cpu_count() // build_jobs))
    if runner:
        print(f"runner: {shlex.join(runner)}", flush=True)
        manifest["runner"] = shlex.join(runner)
        # Builds on other hosts share this checkout's dependencies: check them out
        # here, or concurrent builds clone the same one at once and fail; and the
        # generated sources need the uv environment, which those hosts may lack
        for cmd in (["bender", "-d", str(ROOT_DIR), "checkout"],
                    ["make", "-C", str(TEST_DIR), "hw-deps"]):
            proc = subprocess.run(cmd, capture_output=True, text=True)
            if proc.returncode != 0:
                sys.exit(f"error: {shlex.join(cmd)} failed:\n{proc.stdout[-2000:]}{proc.stderr[-2000:]}")
    verdicts = {}
    with concurrent.futures.ThreadPoolExecutor(max_workers=build_jobs) as pool:
        futures = {pool.submit(build_model, m["dir"], config, packages[k], perf, verilator_jobs,
                               args.rebuild, tree, runner): k for k, m in models.items()}
        for fut in concurrent.futures.as_completed(futures):
            model_dir, verdict = fut.result()
            verdicts[futures[fut]] = verdict
            print(f"model {model_dir.name}: {verdict}", flush=True)
    failed = [k for k, v in verdicts.items() if v.startswith("FAIL")]
    if failed:
        sys.exit("error: not all models built; no point is run")

    # Run
    jobs = args.jobs or (RUNNER_JOBS if runner else os.cpu_count())
    if runner:
        # Cores for the runner to find: no more than the point has simulations
        hexes = [h.stem for h in (TEST_DIR / "sw" / "out" / "bench").glob("*.hex")]
        if sweep.get("benchmarks"):
            hexes = [h for h in hexes if any(f in h for f in sweep["benchmarks"])]
        jobs = max(1, min(jobs, len(hexes) or jobs))
    # The commit the models were built from: run-bench.py may not see it through a runner
    manifest["rtl"] = git("rev-parse", "HEAD").strip() + (
        " (with uncommitted changes)" if git("diff", "HEAD", "--", *MODEL_SOURCES) else "")

    def run_point(label, params, sim_args):
        model = models[json.dumps(params, sort_keys=True)]
        run_dir = out / "runs" / label_dir(label)
        description = " ".join([config] + [f"{k}={v}" for k, v in params.items()])
        results = run_dir / "results.json"
        if args.reuse_runs and results.is_file():
            return f"point {label}: reusing {run_dir}"
        # A result left from an earlier run must not pass for this one's
        results.unlink(missing_ok=True)
        cmd = [*SIM_ENV, str(TEST_DIR / "util" / "run-bench.py"), "run",
               "--sim", str(model["dir"] / "build" / "Vtest"), "--out", str(run_dir),
               "--tag", label, "--config", description, "--sim-args", sim_args,
               "-j", str(jobs)]
        if sweep.get("benchmarks"):
            cmd += ["--filter", *sweep["benchmarks"]]
        if sweep.get("max_cycles"):
            cmd += ["--max-cycles", str(sweep["max_cycles"])]
        print(f"point {label}: {shlex.join(cmd[len(SIM_ENV):])}", flush=True)
        if runner:
            cmd = [*runner, "--cores", str(jobs), "--", *cmd]
        proc = subprocess.run(cmd, capture_output=True, text=True)
        if not results.is_file():
            # run-bench.py stopped before running anything: say why
            last = (proc.stdout + proc.stderr).strip().splitlines()
            return f"point {label}: FAIL, no results: {last[-1] if last else 'no output'}"
        summary = json.loads(results.read_text())["summary"]
        return (f"point {label}: {summary['passed']} passed, {summary['failed']} failed, "
                f"geomean IPC {summary['geomean_ipc'] or 0:.4f}")

    runs = [(label, out / "runs" / label_dir(label)) for label, _, _ in points]
    with concurrent.futures.ThreadPoolExecutor(
            max_workers=args.points or (len(points) if runner else 1)) as pool:
        for fut in concurrent.futures.as_completed([pool.submit(run_point, *p) for p in points]):
            print(fut.result(), flush=True)
    (out / "sweep.json").write_text(json.dumps(manifest, indent=2) + "\n")

    # Compare
    ran = [(label, d) for label, d in runs if (d / "results.json").is_file()]
    if len(ran) >= 2:
        subprocess.run([sys.executable, str(TEST_DIR / "util" / "compare-runs.py"),
                        *(str(d) for _, d in ran), "--labels", *(label for label, _ in ran),
                        "--out", str(out / "compare")])
    print(f"sweep {sweep['name']}: results in {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
