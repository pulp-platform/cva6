#!/usr/bin/env python3
# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
"""Compare benchmark runs: core configurations, the points of a design space
exploration, a change against its baseline.

    ./util/compare-runs.py <run> <run> [<run> ...] [--labels <label> ...]
                           [--baseline <label>] [--out <dir>] [--format png|svg|pdf]

Each <run> is a results.json written by run-bench.py, or the directory holding
it. The first run is the baseline unless --baseline names another. Writes, by
default to verilator/bench/compare/:

    speedup         each run's geometric mean speedup over the baseline
    speedup-map     the speedup per benchmark and run
    cpi             each run's cycles per instruction, split by cause
    runs.csv        per run: configuration, simulation arguments, IPC, speedup,
                    scores, CPI and its parts, events per 1000 instructions
    suites.csv      per suite: the geometric mean speedup of every run
    benchmarks.csv  per benchmark: IPC and speedup in every run

and prints the same as tables. Speedups come from the regions' cycles, over the
benchmarks that pass in both runs and run the same binary there. The scores
weigh suites alike: compute is the geometric mean of the CoreMark, Embench and
Lua speedups, memory that of RaiderSTREAM and pointer chase (benchresults.SCORES). The CPI stacks
need cycle accounting (models built with PERF=1); they sum all the benchmarks
that every such run has. Runs in the uv environment (see ../README.md):
uv run ./util/compare-runs.py <runs>.
"""

import argparse
import csv
import sys
from pathlib import Path

from benchplot import (DIVERGING, FONT, INK_MUTED, INK_SECONDARY, NEUTRAL, PT_PER_PX,
                       SERIES, SURFACE, DPI, bar_height, ink_on, legend, plt, row_figure,
                       rows_layout, save, stacked, style, titles)
from benchresults import (LOSS_CAUSES, SCORES, accounted, by_suite, cpi_stack, geomean,
                          load_run, per_kilo_insn, run_label, scores, speedups, total)

TEST_DIR = Path(__file__).resolve().parent.parent
STACK = (("base", "Issue at full width"),) + LOSS_CAUSES


def print_table(rows, columns):
    widths = [max(len(str(h)), *(len(str(r[i])) for r in rows)) for i, h in enumerate(columns)]
    line = "  ".join(f"{{:<{w}}}" if i == 0 else f"{{:>{w}}}" for i, w in enumerate(widths))
    print(line.format(*columns))
    print(line.format(*("-" * w for w in widths)))
    for r in rows:
        print(line.format(*r))


def pct(x):
    return f"{(x - 1) * 100:+.2f}%"


def plot_speedup(labels, gm, base, out):
    fig, ax = row_figure(len(labels), header_in=0.8, left=0.22)
    values = [(gm[l] - 1) * 100 if gm[l] else 0.0 for l in labels]
    ys = rows_layout(len(labels), header_rows=0)
    span = max(1.0, *(abs(v) for v in values)) * 1.25
    ax.set_ylim(ys[-1] + 0.8, -0.8)
    ax.set_xlim(-span if min(values) < 0 else 0, span)
    style(ax)
    ax.barh(ys, values, height=bar_height(ax), color=SERIES[0])
    ax.axvline(0, color=INK_SECONDARY, linewidth=PT_PER_PX)
    for y, v, l in zip(ys, values, labels):
        text = "baseline" if l == base else f"{v:+.1f}%"
        ax.text(v + (span * 0.01 if v >= 0 else -span * 0.01), y, text, va="center",
                ha="left" if v >= 0 else "right", fontsize=FONT - 1, color=INK_SECONDARY)
    ax.set_yticks(ys, labels)
    ax.xaxis.set_major_formatter(lambda v, _: f"{v:+.0f}%" if v else "0")
    ax.set_xlabel("geometric mean speedup over the baseline, from region cycles", fontsize=FONT,
                  color=INK_MUTED)
    titles(fig, f"Speedup over {base}", "Per run, over the benchmarks it shares with the baseline.")
    return save(fig, out)


def plot_speedup_map(names, others, per_bench, base, out):
    """Benchmarks (rows) by runs (columns), colored and labelled by speedup."""
    width = max(5.0, 2.0 + 1.1 * len(others))
    height = 1.3 + len(names) * 0.26 + 0.4
    fig, ax = plt.subplots(figsize=(width, height), dpi=DPI)
    fig.patch.set_facecolor(SURFACE)
    fig.subplots_adjust(left=1.6 / width, right=1 - 0.2 / width, top=1 - 1.3 / height,
                        bottom=0.3 / height)
    values = [[(per_bench[o].get(n, float("nan")) - 1) * 100 for o in others] for n in names]
    finite = [abs(v) for row in values for v in row if v == v]
    limit = max(finite + [1.0])
    ax.imshow(values, cmap=DIVERGING, vmin=-limit, vmax=limit, aspect="auto")
    for i, row in enumerate(values):
        for j, v in enumerate(row):
            text = "-" if v != v else f"{v:+.1f}"
            color = INK_MUTED if v != v else ink_on(DIVERGING((v + limit) / (2 * limit)))
            ax.text(j, i, text, ha="center", va="center", fontsize=FONT - 1, color=color)
    ax.set_xticks(range(len(others)), others)
    ax.xaxis.tick_top()
    ax.set_yticks(range(len(names)), names)
    ax.tick_params(colors=INK_SECONDARY, labelsize=FONT, length=0)
    for spine in ax.spines.values():
        spine.set_visible(False)
    # Surface lines between cells
    ax.set_xticks([x - 0.5 for x in range(1, len(others))], minor=True)
    ax.set_yticks([y - 0.5 for y in range(1, len(names))], minor=True)
    ax.grid(which="minor", color=SURFACE, linewidth=2 * PT_PER_PX)
    ax.tick_params(which="minor", length=0)
    titles(fig, f"Speedup over {base}, per benchmark",
           f"In percent; blue faster, red slower, scale to ±{limit:.0f}%; - where not comparable.")
    return save(fig, out)


def plot_cpi(labels, stacks, base, n_bench, out):
    parts = [[stacks[l][key] for l in labels] for key, _ in STACK]
    totals = [sum(stacks[l].values()) for l in labels]
    xmax = max(totals) * 1.12
    fig, ax = row_figure(len(labels), header_in=1.25, left=0.22)
    ys, ends = stacked(ax, labels, parts, (NEUTRAL,) + SERIES, xmax, header_rows=0,
                       value_format=lambda w: f"{w:.2f}")
    for y, end in zip(ys, ends):
        ax.text(end + xmax * 0.01, y, f"{end:.3f}", va="center", fontsize=FONT - 1,
                color=INK_SECONDARY)
    ax.set_xlabel("cycles per instruction", fontsize=FONT, color=INK_MUTED)
    titles(fig, "Cycles per instruction, by cause",
           f"All {n_bench} benchmarks the runs share, together. The base is the CPI at full "
           "issue width; each cause adds its lost slots, in cycles.")
    legend(fig, [text for _, text in STACK], (NEUTRAL,) + SERIES, 5)
    return save(fig, out)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("runs", nargs="+", help="results.json files, or the directories holding them")
    parser.add_argument("--labels", nargs="+", help="names for the runs (default: their tags)")
    parser.add_argument("--baseline", help="label of the baseline run (default: the first)")
    parser.add_argument("--out", default=TEST_DIR / "verilator/bench/compare",
                        help="where to write plots and tables (default: verilator/bench/compare)")
    parser.add_argument("--format", default="png", choices=("png", "svg", "pdf"))
    args = parser.parse_args()

    try:
        runs = [load_run(p) for p in args.runs]
    except (OSError, ValueError) as e:
        sys.exit(f"error: {e}")
    if len(runs) < 2:
        sys.exit("error: give at least two runs")
    labels = args.labels or [run_label(r) for r in runs]
    if len(labels) != len(runs):
        sys.exit("error: give one label per run")
    labels = [l if labels.count(l) == 1 else f"{l}#{i}" for i, l in enumerate(labels)]
    by_label = dict(zip(labels, runs))
    base = args.baseline or labels[0]
    if base not in by_label:
        sys.exit(f"error: no run labelled {base}")
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    # Speedups over the baseline
    per_bench, gm, differ = {}, {}, {}
    for l in labels:
        per_bench[l], differ[l] = speedups(by_label[base], by_label[l])
        gm[l] = geomean(per_bench[l].values())
    ipc = {l: {r["name"]: r["ipc"] for r in by_label[l]["results"] if r["status"] == "PASS"}
           for l in labels}
    others = [l for l in labels if l != base]
    names = sorted({n for o in others for n in per_bench[o]})
    suite_gm = {o: {t: geomean(v.values()) for t, v in by_suite(per_bench[o]).items()}
                for o in others}
    suite_sizes = {t: len(v) for t, v in by_suite(dict.fromkeys(names)).items()}
    score = {o: scores(suite_gm[o]) for o in others}
    score_names = [n for n, _ in SCORES if any(score[o][n] for o in others)]

    # CPI stacks over the benchmarks every accounted run shares
    acc = {l: {r["name"]: c for r, c in accounted(by_label[l])} for l in labels}
    with_perf = [l for l in labels if acc[l]]
    shared = sorted(set.intersection(*(set(acc[l]) for l in with_perf))) if with_perf else []
    totals = {l: total([acc[l][n] for n in shared]) for l in with_perf} if shared else {}
    stacks = {l: cpi_stack(totals[l]) for l in totals}

    # Tables
    print(f"baseline: {base}")
    rows = []
    for l in labels:
        meta = by_label[l].get("meta", {})
        passed = [r for r in by_label[l]["results"] if r["status"] == "PASS"]
        rows.append((l, meta.get("config") or "-", meta.get("sim_args") or "-", len(passed),
                     f"{geomean(r['ipc'] for r in passed):.4f}" if passed else "-",
                     "baseline" if l == base else (pct(gm[l]) if gm[l] else "-"),
                     *("" if l == base else (pct(score[l][n]) if score[l][n] else "-")
                       for n in score_names),
                     f"{sum(stacks[l].values()):.4f}" if l in stacks else "-"))
    print_table(rows, ["run", "config", "sim args", "passed", "geomean ipc", "speedup",
                       *score_names, "cpi"])
    for l in others:
        if differ[l]:
            print(f"{l}: not compared, different binaries: {', '.join(differ[l])}")

    if stacks:
        print()
        print(f"cycles per instruction by cause, {len(shared)} shared benchmarks together:")
        print_table([(text, *(f"{stacks[l][key]:.4f}" for l in stacks)) for key, text in STACK]
                    + [("total", *(f"{sum(stacks[l].values()):.4f}" for l in stacks))],
                    ["", *stacks])
    if len(suite_sizes) > 1:
        print()
        print(f"speedup over {base} per suite (geometric mean):")
        print_table([(f"{t} ({n})", *(pct(suite_gm[o][t]) if t in suite_gm[o] else "-"
                                       for o in others)) for t, n in suite_sizes.items()],
                    ["suite", *others])
    print()
    print(f"speedup over {base} per benchmark:")
    print_table([(n, *(f"{(per_bench[o][n] - 1) * 100:+.2f}%" if n in per_bench[o] else "-"
                       for o in others)) for n in names], ["benchmark", *others])

    # Plots
    written = [plot_speedup(labels, gm, base, out / f"speedup.{args.format}")]
    if names:
        written.append(plot_speedup_map(names, others, per_bench, base,
                                        out / f"speedup-map.{args.format}"))
    if stacks:
        written.append(plot_cpi(list(stacks), stacks, base, len(shared), out / f"cpi.{args.format}"))
    else:
        print("no cycle accounting shared by the runs (models built with PERF=1): no CPI stacks")

    # Tables as CSV
    with open(out / "runs.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["run", "results", "config", "sim_args", "passed", "geomean_ipc",
                         "speedup_vs_baseline", *(f"score_{n}" for n, _ in SCORES), "cpi",
                         *(f"cpi.{k}" for k, _ in STACK),
                         "mispredicts_pki", "icache_misses_pki", "dcache_misses_pki"])
        for l, row in zip(labels, rows):
            writer.writerow([l, by_label[l]["path"], row[1], row[2], row[3], row[4],
                             "" if l == base or not gm[l] else f"{gm[l]:.5f}",
                             *("" if l == base or not score[l][n] else f"{score[l][n]:.5f}"
                               for n, _ in SCORES),
                             *((f"{sum(stacks[l].values()):.5f}",
                                *(f"{stacks[l][k]:.5f}" for k, _ in STACK)) if l in stacks
                               else [""] * (len(STACK) + 1)),
                             *((f"{per_kilo_insn(totals[l], e):.3f}"
                                for e in ("mispredicts", "icache_misses", "dcache_misses"))
                               if l in totals else ["", "", ""])])
    with open(out / "suites.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["suite", "benchmarks", *(f"speedup.{o}" for o in others)])
        for t, n in suite_sizes.items():
            writer.writerow([t, n, *(f"{suite_gm[o][t]:.5f}" if t in suite_gm[o] else ""
                                     for o in others)])
    with open(out / "benchmarks.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["benchmark", *(f"ipc.{l}" for l in labels), *(f"speedup.{o}" for o in others)])
        for n in sorted({n for l in labels for n in ipc[l]}):
            writer.writerow([n, *(ipc[l].get(n, "") for l in labels),
                             *(f"{per_bench[o][n]:.5f}" if n in per_bench[o] else "" for o in others)])
    written += [out / "runs.csv", out / "suites.csv", out / "benchmarks.csv"]
    print()
    print("written: " + ", ".join(str(p) for p in written))


if __name__ == "__main__":
    main()
