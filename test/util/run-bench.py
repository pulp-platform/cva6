#!/usr/bin/env python3
# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
"""Run the benchmarks on the testbench and collect their performance.

    run      simulate every benchmark under sw/out/bench (make -C sw bench) on a
             built model, in parallel, and write results.json, results.csv and
             one log per benchmark to the output directory
    check    run the benchmarks of the model's expected results
             (sw/bench/expected/<config>.json), report each one's cycles
             against them, and end with a verdict line: **BENCH CHECK PASSED**
             if all match, **BENCH CHECK FAILED** otherwise; --update writes
             the expected results from the run instead

plot-run.py shows a run's results, compare-runs.py compares runs.

A benchmark passes when the testbench reports **SIMULATION PASSED** (main
returned 0), it printed its region-of-interest counters, and, for those that
validate their output (all but Embench, whose main returns non-zero on a
mismatch), the validation line is there. Cycles and retired instructions are
the region's mcycle and minstret deltas, as measured by the benchmark itself
(sw/bench/lib/bench.c).

Like the other simulation flows it runs in the environment that built the
model:

    ./util/run-bench.py run

Only the standard library is used, so it runs on that environment's Python.
"""

import argparse
import concurrent.futures
import csv
import datetime
import json
import math
import os
import platform
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

from benchresults import RESULTS_SCHEMA, by_suite

TEST_DIR = Path(__file__).resolve().parent.parent
EXPECTED_DIR = TEST_DIR / "sw/bench/expected"
EXPECTED_SCHEMA = 1

# `check --set quick`: one or more programs of each suite, about 5 minutes
# of simulation with 4 jobs
QUICK_SET = ("coremark", "dhrystone", "em-minver", "em-ud", "em-statemate", "em-wikisort",
             "em-aha-mont64", "rs-central", "chase-16k", "chase-64k", "chase-256k", "lua-sieve")

# Console lines a benchmark must print, besides its ROI counters, to count as
# correct, by name or, for keys ending in '-', by name prefix. CoreMark returns 0
# even when its checksums mismatch; the others return non-zero on a mismatch,
# and their line confirms the check ran.
REQUIRED_OUTPUT = {
    "coremark": "Correct operation validated",
    "dhrystone": "[DHRY] results verified",
    "rs-": "[RS] results verified",
    "chase-": "[CHASE] chain verified",
    "lua-": "[LUA] results verified",
}

# Dhrystones per second of the VAX 11/780, the 1 DMIPS reference
DHRYSTONES_PER_DMIPS = 1757

RE_ROI = re.compile(r"\[BENCH\] roi cycles=(\d+) instret=(\d+)")
RE_TRAP = re.compile(r"\[BENCH\] trap (.*)")
RE_TIME = re.compile(r"^@\s*(\d+)ns")
RE_COREMARK_ITER = re.compile(r"Iterations\s*:\s*(\d+)")
RE_DHRYSTONE_RUNS = re.compile(r"\[DHRY\] runs=(\d+)")
RE_TARGET = re.compile(r"TARGET_(CV[0-9A-Z_]+)")


def git_info(path):
    def git(*args):
        try:
            return subprocess.run(["git", "-C", str(path), *args], capture_output=True,
                                  text=True, check=True).stdout.strip()
        except (subprocess.CalledProcessError, FileNotFoundError):
            return None

    return {
        "commit": git("rev-parse", "HEAD"),
        "branch": git("rev-parse", "--abbrev-ref", "HEAD"),
        "dirty": bool(git("status", "--porcelain", "--untracked-files=no")),
    }


def model_config(sim):
    """The CVA6 configuration the model was built for, from its Bender file list."""
    for flist in (sim.parent / "cva6tb.flist", sim.parent.parent / "cva6tb.flist"):
        if flist.is_file():
            m = RE_TARGET.search(flist.read_text())
            if m:
                return m.group(1).lower()
    return None


def run_one(sim, hex_path, log_path, max_cycles, sim_args):
    name = hex_path.stem
    # A simulation writes files to its working directory (timer log, waves, and
    # CVA6's legacy trace if built in): give each its own, emptied first.
    workdir = log_path.parent / "work" / name
    shutil.rmtree(workdir, ignore_errors=True)
    workdir.mkdir(parents=True)
    cmd = [str(sim), f"+binary={hex_path}", f"+MaxCycles={max_cycles}", *sim_args]
    start = time.monotonic()
    # a program gone wrong may print anything: keep its log readable
    proc = subprocess.run(cmd, cwd=workdir, capture_output=True, text=True, errors="replace")
    wall = time.monotonic() - start
    log = proc.stdout + proc.stderr
    log_path.write_text(log)

    result = {"name": name, "status": "FAIL", "cycles": None, "instret": None, "ipc": None,
              "sim_cycles": None, "wall_s": round(wall, 1), "detail": ""}

    times = [int(m.group(1)) for m in map(RE_TIME.match, log.splitlines()) if m]
    if times:
        result["sim_cycles"] = times[-1]  # the testbench clock is 1 GHz: 1 ns per cycle

    roi = RE_ROI.search(log)
    if roi:
        result["cycles"], result["instret"] = int(roi.group(1)), int(roi.group(2))
        if result["cycles"]:
            result["ipc"] = round(result["instret"] / result["cycles"], 4)
    if name == "coremark":
        it = RE_COREMARK_ITER.search(log)
        if it and result["cycles"]:
            result["coremark_per_mhz"] = round(int(it.group(1)) * 1e6 / result["cycles"], 4)

    if name == "dhrystone":
        runs = RE_DHRYSTONE_RUNS.search(log)
        if runs and result["cycles"]:
            result["dmips_per_mhz"] = round(
                int(runs.group(1)) * 1e6 / DHRYSTONES_PER_DMIPS / result["cycles"], 4)

    trap = RE_TRAP.search(log)
    required = next((line for key, line in REQUIRED_OUTPUT.items()
                     if name == key or (key.endswith("-") and name.startswith(key))), None)
    if trap:
        result["status"], result["detail"] = "TRAP", trap.group(1)
    elif "SIMULATION PASSED" not in log:
        result["status"] = "TIMEOUT" if "timed out" in log else "FAIL"
    elif not roi:
        result["status"], result["detail"] = "NOROI", "no [BENCH] roi line"
    elif required and required not in log:
        result["status"], result["detail"] = "INVALID", f"missing '{required}'"
    else:
        result["status"] = "PASS"
    return result


def geomean(values):
    values = [v for v in values if v]
    return math.exp(sum(map(math.log, values)) / len(values)) if values else None


def print_table(rows, columns):
    widths = [max(len(str(h)), *(len(str(r[i])) for r in rows)) for i, h in enumerate(columns)]
    line = "  ".join(f"{{:<{w}}}" if i == 0 else f"{{:>{w}}}" for i, w in enumerate(widths))
    print(line.format(*columns))
    print(line.format(*("-" * w for w in widths)))
    for r in rows:
        print(line.format(*r))


def simulate(sim, hex_dir, hexes, out, config, tag, sim_args, jobs, max_cycles):
    """Run the images in parallel, one simulation each, and write results.json
    and results.csv to `out`. Returns the run as results.json holds it."""
    out.mkdir(parents=True, exist_ok=True)
    build_info = hex_dir / "build-info.txt"
    meta = {
        "date": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
        "host": platform.node(),
        "config": config,
        "sim": str(sim),
        "sim_mtime": datetime.datetime.fromtimestamp(sim.stat().st_mtime).isoformat(
            timespec="seconds"),
        "sim_args": sim_args,
        "max_cycles": max_cycles,
        "rtl": git_info(TEST_DIR),
        "software": build_info.read_text().strip().splitlines() if build_info.is_file() else None,
        "tag": tag,
    }
    print(f"bench: {len(hexes)} benchmarks on {meta['config']} ({sim}), {jobs} at a time")

    results = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=jobs) as pool:
        futures = [pool.submit(run_one, sim, h, out / f"{h.stem}.log", max_cycles, sim_args.split())
                   for h in hexes]
        for i, fut in enumerate(concurrent.futures.as_completed(futures), 1):
            r = fut.result()
            results.append(r)
            ipc = f"ipc={r['ipc']:.3f}" if r["ipc"] else ""
            print(f"[{i}/{len(hexes)}] {r['status']:<7} {r['name']:<20} {ipc} "
                  f"({r['wall_s']:.0f} s) {r['detail']}", flush=True)
    results.sort(key=lambda r: r["name"])

    passed = [r for r in results if r["status"] == "PASS"]
    summary = {
        "passed": len(passed),
        "failed": len(results) - len(passed),
        "geomean_ipc": geomean([r["ipc"] for r in passed]),
        "geomean_ipc_by_suite": {t: geomean(v.values()) for t, v in
                                 by_suite({r["name"]: r["ipc"] for r in passed}).items()},
        "coremark_per_mhz": next((r.get("coremark_per_mhz") for r in passed
                                  if r["name"] == "coremark"), None),
        "dmips_per_mhz": next((r.get("dmips_per_mhz") for r in passed
                               if r["name"] == "dhrystone"), None),
        "sim_cycles_per_s": round(sum(r["sim_cycles"] or 0 for r in results) /
                                  max(sum(r["wall_s"] for r in results), 1e-9)),
    }
    (out / "results.json").write_text(
        json.dumps({"schema": RESULTS_SCHEMA, "meta": meta, "summary": summary,
                    "results": results}, indent=2) + "\n")
    fields = ["name", "status", "cycles", "instret", "ipc", "sim_cycles", "wall_s", "detail"]
    with open(out / "results.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(results)
    return {"meta": meta, "summary": summary, "results": results}


def model_and_images(args, target="bench"):
    """The model, the image directory and its images, or an error message.
    target is the make target that builds the images."""
    sim = Path(args.sim).resolve()
    hex_dir = Path(args.hex_dir).resolve()
    if not os.access(sim, os.X_OK):
        return None, None, None, (f"{sim} not built; run "
                                  "'make verilator-build CVA6_CONFIG=<cfg> THREADS=1'")
    hexes = sorted(hex_dir.glob("*.hex"))
    if not hexes:
        return None, None, None, f"no benchmarks under {hex_dir}; run 'make -C sw {target}'"
    return sim, hex_dir, hexes, None


def cmd_run(args):
    sim, hex_dir, hexes, error = model_and_images(args)
    if error:
        sys.exit(f"error: {error}")
    if args.filter:
        hexes = [h for h in hexes if any(f in h.stem for f in args.filter)]
        if not hexes:
            sys.exit(f"error: no benchmark under {hex_dir} matches {' '.join(args.filter)}")
    config = args.config or model_config(sim)
    out = Path(args.out or TEST_DIR / "verilator/bench" / (args.tag or config or "run")).resolve()
    run = simulate(sim, hex_dir, hexes, out, config, args.tag, args.sim_args, args.jobs,
                   args.max_cycles)
    results, summary = run["results"], run["summary"]
    passed = [r for r in results if r["status"] == "PASS"]

    print()
    print_table([(r["name"], r["status"], r["cycles"] or "-", r["instret"] or "-",
                  f"{r['ipc']:.3f}" if r["ipc"] else "-") for r in results],
                ["benchmark", "status", "cycles", "instret", "ipc"])
    print("---")
    if summary["geomean_ipc"]:
        print(f"geomean IPC over {len(passed)} passing: {summary['geomean_ipc']:.4f}")
        if len(summary["geomean_ipc_by_suite"]) > 1:
            print("  by suite: " + ", ".join(f"{t} {g:.4f}" for t, g in
                                             summary["geomean_ipc_by_suite"].items()))
    if summary["coremark_per_mhz"]:
        print(f"CoreMark/MHz: {summary['coremark_per_mhz']:.4f}")
    if summary["dmips_per_mhz"]:
        print(f"DMIPS/MHz: {summary['dmips_per_mhz']:.4f}")
    print(f"simulation speed: {summary['sim_cycles_per_s']} cycles/s per job")
    print(f"bench: {summary['passed']} passed, {summary['failed']} failed; results in {out}")
    print(f"show the results: uv run ./util/plot-run.py {out}")
    return 0 if summary["failed"] == 0 else 1


def verdict(word, why=""):
    """The check's verdict, as the line a log is judged by."""
    print(f"**BENCH CHECK {word}**{': ' + why if why else ''}")
    return 0 if word in ("PASSED", "REPORTED", "UPDATED") else 1


def change(cycles, expected):
    pct = (cycles / expected - 1) * 100
    return f"{pct:+.2f}%" if pct else "="


def cmd_check(args):
    sim, hex_dir, hexes, error = model_and_images(args, "bench-ci")
    if error:
        return verdict("FAILED", error)
    config = args.config or model_config(sim)
    if not config:
        return verdict("FAILED", f"cannot tell the configuration of {sim}; give --config")
    path = Path(args.expected) if args.expected else EXPECTED_DIR / f"{config}.json"
    expected = json.loads(path.read_text()) if path.is_file() else None
    if expected and expected.get("schema", 1) > EXPECTED_SCHEMA:
        return verdict("FAILED", f"{path}: schema {expected['schema']}, newer than this script's")
    images = {h.stem: h for h in hexes}

    # What to run: everything, to write the expected results; otherwise what
    # they list, measured as they were
    if args.update:
        names = sorted(images)
    else:
        if not expected:
            return verdict("FAILED", f"no expected results for {config} ({path}); "
                           "write them with --update")
        if expected["config"] != config:
            return verdict("FAILED", f"{path} is for {expected['config']}, the model is {config}")
        if expected.get("sim_args", "") != args.sim_args:
            return verdict("FAILED", f"{path} was measured with sim args "
                           f"'{expected.get('sim_args', '')}', not '{args.sim_args}'")
        names = sorted(expected["benchmarks"])
        if args.set == "quick":
            names = [n for n in names if n in QUICK_SET]
        missing = [n for n in names if n not in images]
        if missing:
            return verdict("FAILED", f"no images under {hex_dir} for {', '.join(missing)}; "
                           "run 'make -C sw bench-ci'")
    out = Path(args.out or TEST_DIR / "verilator/bench" / f"check-{config}").resolve()
    run = simulate(sim, hex_dir, [images[n] for n in names], out, config, f"check-{config}",
                   args.sim_args, args.jobs, args.max_cycles)
    results = {r["name"]: r for r in run["results"]}
    failed = [n for n in names if results[n]["status"] != "PASS"]
    old = (expected or {}).get("benchmarks", {})

    # The report: each benchmark's cycles against the expected ones, then the
    # geometric mean of the ratios per suite and overall
    rows, ratios = [], {}
    for n in names:
        r, e = results[n], old.get(n)
        note = ""
        if r["status"] != "PASS":
            note = f"{r['status']} {r['detail']}".strip()
        elif e and r["instret"] != e["instret"]:
            note = f"other binary: {r['instret']} instructions, expected {e['instret']}"
        elif e:
            ratios[n] = r["cycles"] / e["cycles"]
        rows.append((n, e["cycles"] if e else "-", r["cycles"] or "-",
                     change(r["cycles"], e["cycles"]) if e and n in ratios else "-", note))
    print()
    print_table(rows, ["benchmark", "expected", "cycles", "change", ""])
    print("---")
    if ratios:
        print("cycles against the expected results, geometric mean (+ slower, - faster):")
        for suite, values in by_suite(ratios).items():
            g = geomean(values.values())
            print(f"  {suite:<14} {(g - 1) * 100:+.2f}%  ({len(values)})")
        g = geomean(ratios.values())
        print(f"  {'all':<14} {(g - 1) * 100:+.2f}%  ({len(ratios)})")
    print(f"results in {out}")

    if args.update:
        if failed:
            return verdict("FAILED", f"not updated: {', '.join(failed)} did not pass")
        doc = {"schema": EXPECTED_SCHEMA, "config": config, "sim_args": args.sim_args,
               "measured": {k: run["meta"][k] for k in ("date", "rtl", "software")},
               "benchmarks": {n: {"cycles": results[n]["cycles"],
                                  "instret": results[n]["instret"]} for n in names}}
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(doc, indent=2) + "\n")
        return verdict("UPDATED", f"{path}, {len(names)} benchmarks")

    other = [n for n in names if results[n]["status"] == "PASS" and n not in ratios]
    changed = [n for n, x in ratios.items() if x != 1]
    if failed or other:
        return verdict("FAILED", "; ".join(
            ([f"did not pass: {', '.join(failed)}"] if failed else []) +
            ([f"not the binaries the expected results measured (other instruction counts): "
              f"{', '.join(other)}"] if other else [])))
    if changed:
        why = (f"{len(changed)} of {len(names)} benchmarks changed; if intended, "
               f"update {path.relative_to(TEST_DIR) if path.is_relative_to(TEST_DIR) else path} "
               "with --update (make bench-check-update) and commit it")
        return verdict("REPORTED" if args.report_only else "FAILED", why)
    return verdict("PASSED", f"{len(names)} benchmarks as expected")


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)

    run = sub.add_parser("run", help="simulate the benchmarks")
    run.add_argument("--sim", default=TEST_DIR / "verilator/build/Vtest",
                     help="model to simulate (default: verilator/build/Vtest)")
    run.add_argument("--hex-dir", default=TEST_DIR / "sw/out/bench",
                     help="benchmark images (default: sw/out/bench)")
    run.add_argument("--out", help="logs and results (default: verilator/bench/<tag or config>, "
                     "which 'make verilator-clean' leaves alone)")
    run.add_argument("-j", "--jobs", type=int, default=int(os.environ.get("NJOBS", os.cpu_count())))
    run.add_argument("--max-cycles", type=int, default=200_000_000)
    run.add_argument("--sim-args", default="", help="extra plusargs for every simulation")
    run.add_argument("--filter", nargs="*", help="only benchmarks whose name contains one of these")
    run.add_argument("--config", help="configuration name to record, if not from the file list")
    run.add_argument("--tag", help="label for this run, used to name its results and in plots")
    run.set_defaults(func=cmd_run)

    check = sub.add_parser("check", help="compare with the expected results")
    check.add_argument("--sim", default=TEST_DIR / "verilator/build/Vtest",
                       help="model to simulate (default: verilator/build/Vtest)")
    check.add_argument("--hex-dir", default=TEST_DIR / "sw/out/bench-ci",
                       help="benchmark images (default: sw/out/bench-ci, make -C sw bench-ci)")
    check.add_argument("--expected", help="expected results "
                       "(default: sw/bench/expected/<configuration>.json)")
    check.add_argument("--out", help="logs and results (default: verilator/bench/check-<config>)")
    check.add_argument("-j", "--jobs", type=int,
                       default=int(os.environ.get("NJOBS", os.cpu_count())))
    check.add_argument("--max-cycles", type=int, default=200_000_000)
    check.add_argument("--sim-args", default="", help="extra plusargs for every simulation")
    check.add_argument("--config", help="the model's configuration, if not from the file list")
    check.add_argument("--set", choices=("full", "quick"), default="full",
                       help="every benchmark of the expected results, or the quick set")
    check.add_argument("--report-only", action="store_true",
                       help="a change of cycles is reported (**BENCH CHECK REPORTED**), "
                       "not a failure; failing benchmarks still fail")
    check.add_argument("--update", action="store_true",
                       help="run every benchmark and write the expected results from it")
    check.set_defaults(func=cmd_check)

    args = parser.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
