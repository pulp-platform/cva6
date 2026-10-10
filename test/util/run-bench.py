#!/usr/bin/env python3
# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
"""Run the benchmarks on the testbench and collect their performance.

    run      simulate every benchmark under sw/out/bench (make -C sw bench) on a
             built model, in parallel, and write results.json, results.csv and
             one log per benchmark to the output directory

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

# Console lines a benchmark must print, besides its ROI counters, to count as
# correct, by name or, for keys ending in '-', by name prefix. CoreMark returns 0
# even when its checksums mismatch; the others return non-zero on a mismatch,
# and their line confirms the check ran.
REQUIRED_OUTPUT = {
    "coremark": "Correct operation validated",
    "rs-": "[RS] results verified",
}


RE_ROI = re.compile(r"\[BENCH\] roi cycles=(\d+) instret=(\d+)")
RE_TRAP = re.compile(r"\[BENCH\] trap (.*)")
RE_TIME = re.compile(r"^@\s*(\d+)ns")
RE_COREMARK_ITER = re.compile(r"Iterations\s*:\s*(\d+)")
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


def cmd_run(args):
    sim = Path(args.sim).resolve()
    hex_dir = Path(args.hex_dir).resolve()
    config = args.config or model_config(sim)
    out = Path(args.out or TEST_DIR / "verilator/bench" / (args.tag or config or "run")).resolve()
    if not os.access(sim, os.X_OK):
        sys.exit(f"error: {sim} not built; run 'make verilator-build CVA6_CONFIG=<cfg> THREADS=1'")
    hexes = sorted(hex_dir.glob("*.hex"))
    if args.filter:
        hexes = [h for h in hexes if any(f in h.stem for f in args.filter)]
    if not hexes:
        sys.exit(f"error: no benchmarks under {hex_dir}; run 'make -C sw bench'")
    out.mkdir(parents=True, exist_ok=True)

    build_info = hex_dir / "build-info.txt"
    meta = {
        "date": datetime.datetime.now(datetime.timezone.utc).isoformat(timespec="seconds"),
        "host": platform.node(),
        "config": config,
        "sim": str(sim),
        "sim_mtime": datetime.datetime.fromtimestamp(sim.stat().st_mtime).isoformat(
            timespec="seconds"),
        "sim_args": args.sim_args,
        "max_cycles": args.max_cycles,
        "rtl": git_info(TEST_DIR),
        "software": build_info.read_text().strip().splitlines() if build_info.is_file() else None,
        "tag": args.tag,
    }
    sim_args = args.sim_args.split()
    print(f"bench: {len(hexes)} benchmarks on {meta['config']} ({sim}), {args.jobs} at a time")

    results = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        futures = [pool.submit(run_one, sim, h, out / f"{h.stem}.log", args.max_cycles, sim_args)
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
    print(f"simulation speed: {summary['sim_cycles_per_s']} cycles/s per job")
    print(f"bench: {summary['passed']} passed, {summary['failed']} failed; results in {out}")
    print(f"show the results: uv run ./util/plot-run.py {out}")
    return 0 if summary["failed"] == 0 else 1


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

    args = parser.parse_args()
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
