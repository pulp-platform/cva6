#!/usr/bin/env python3
# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
"""Plot the results of one benchmark run: one core configuration, any number of
benchmarks.

    ./util/plot-run.py verilator/bench/cva6s+ [--out <dir>] [--format png|svg|pdf]

Reads the run's results.json (written by run-bench.py) and writes, by default to
<run>/plots/:

    ipc            IPC per benchmark, and the geometric mean of each suite
    ipc.csv        IPC, cycles and instructions per benchmark

Runs in the uv environment (see ../README.md): uv run ./util/plot-run.py <run>.
"""

import argparse
import csv
import sys
from pathlib import Path

from benchplot import (FONT, INK_MUTED, INK_SECONDARY, SERIES, bar_height, row_figure, rows_layout,
                       save, style, titles)
from benchresults import by_suite, geomean, load_run, run_label


def plot_ipc(results, label, out):
    suites = by_suite({r["name"]: r["ipc"] for r in results})
    if len(suites) > 1:
        heads = [(f"[{t}]", geomean(v.values())) for t, v in suites.items()]
        subtitle = f"{len(results)} benchmarks; the geometric mean of each suite on top."
    else:
        heads = [("[geomean]", geomean(r["ipc"] for r in results))]
        subtitle = f"{len(results)} benchmarks; their geometric mean on top."
    names = [n for n, _ in heads] + [r["name"] for r in results]
    values = [v for _, v in heads] + [r["ipc"] for r in results]
    fig, ax = row_figure(len(names), header_in=0.8)
    ys = rows_layout(len(names), len(heads))
    xmax = max(values) * 1.15
    ax.set_ylim(ys[-1] + 0.8, -0.8)
    ax.set_xlim(0, xmax)
    style(ax)
    ax.barh(ys, values, height=bar_height(ax), color=SERIES[0])
    for y, v in zip(ys, values):
        ax.text(v + xmax * 0.01, y, f"{v:.3f}", va="center", fontsize=FONT - 1, color=INK_SECONDARY)
    ax.set_yticks(ys, names)
    for tick in ax.get_yticklabels()[:len(heads)]:
        tick.set_fontweight("bold")
    ax.set_xlabel("instructions per cycle, in each benchmark's measured region", fontsize=FONT,
                  color=INK_MUTED)
    titles(fig, f"IPC: {label}", subtitle)
    return save(fig, out)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run", help="the run's results.json, or the directory holding it")
    parser.add_argument("--out", help="where to write the plots (default: <run>/plots)")
    parser.add_argument("--format", default="png", choices=("png", "svg", "pdf"))
    args = parser.parse_args()

    try:
        run = load_run(args.run)
    except (OSError, ValueError) as e:
        sys.exit(f"error: {e}")
    label = run_label(run)
    config = run.get("meta", {}).get("config")
    if config and config != label:
        label = f"{label} ({config})"
    out = Path(args.out) if args.out else Path(run["path"]).parent / "plots"
    out.mkdir(parents=True, exist_ok=True)

    passed = sorted((r for r in run["results"] if r["status"] == "PASS"),
                    key=lambda r: -r["ipc"])
    if not passed:
        sys.exit(f"error: no passing benchmark in {run['path']}")
    geo = geomean(r["ipc"] for r in passed)
    written = [plot_ipc(passed, label, out / f"ipc.{args.format}")]

    with open(out / "ipc.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["benchmark", "ipc", "cycles", "instret"])
        for r in passed:
            writer.writerow([r["name"], r["ipc"], r["cycles"], r["instret"]])
    written.append(out / "ipc.csv")

    print(f"geomean IPC over {len(passed)} benchmarks: {geo:.4f}")
    suite_ipc = by_suite({r["name"]: r["ipc"] for r in passed})
    if len(suite_ipc) > 1:
        print("  by suite: " + ", ".join(f"{t} {geomean(v.values()):.4f}"
                                         for t, v in suite_ipc.items()))
    print("written: " + ", ".join(str(p) for p in written))


if __name__ == "__main__":
    main()
