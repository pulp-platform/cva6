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
    slots          where the issue slots go: retiring, bad speculation,
                   frontend, backend, per benchmark and per suite
    losses         the lost slots by cause, per benchmark and per suite
    events         mispredictions and cache misses per 1000 instructions
    histograms     redirect penalties and load stall lengths, all benchmarks
    breakdown.csv  every number of slots, losses and events, per benchmark and
                   per suite
    histograms.csv the two histograms, summed over the benchmarks

All but ipc need a model built with PERF=1. It also prints the breakdown. Runs
in the uv environment (see ../README.md): uv run ./util/plot-run.py <run>.
"""

import argparse
import csv
import math
import sys
from pathlib import Path

from benchplot import (DPI, FONT, INK, INK_MUTED, INK_SECONDARY, PT_PER_PX, ROW_IN, SERIES, SURFACE,
                       bar_height, legend, plt, row_figure, rows_layout, save, stacked, style,
                       titles)
from benchresults import (LOAD_HIT_MAX_STALL, LOSS_CAUSES, TOPDOWN, accounted, by_suite, geomean,
                          load_run, per_kilo_insn, print_breakdown, run_label, share, summed_hist,
                          total)

TOPDOWN_LABELS = {"retiring": "Retiring", "bad_spec": "Bad speculation",
                  "frontend": "Frontend", "backend": "Backend"}


def xlabel_share(ax):
    ax.xaxis.set_major_formatter(lambda v, _: f"{100 * v:.0f}%")
    ax.set_xlabel("share of issue slots", fontsize=FONT, color=INK_MUTED)


def plot_slots(rows, heads, label, ports, out):
    names = [n for n, _ in rows]
    parts = [[share(c, k) for _, c in rows] for k in TOPDOWN]
    fig, ax = row_figure(len(rows))
    stacked(ax, names, parts, SERIES[:4], 1.0, header_rows=heads)
    xlabel_share(ax)
    titles(fig, f"Where the issue slots go: {label}",
           f"Share of all issue slots ({ports} per cycle) in each benchmark's measured region. "
           "Bracketed rows sum their benchmarks' slots.")
    legend(fig, [TOPDOWN_LABELS[k] for k in TOPDOWN], SERIES[:4], 4)
    return save(fig, out)


def plot_losses(rows, heads, label, out):
    names = [n for n, _ in rows]
    parts = [[share(c, f"loss.{key}") for _, c in rows] for key, _ in LOSS_CAUSES]
    xmax = math.ceil(max(sum(col) for col in zip(*parts)) * 10 + 0.5) / 10
    fig, ax = row_figure(len(rows), header_in=1.2)
    stacked(ax, names, parts, SERIES, min(xmax, 1.0), header_rows=heads)
    xlabel_share(ax)
    titles(fig, f"Where the lost slots go: {label}",
           "Lost issue slots by cause, as a share of all slots; a bar's length is the "
           "benchmark's loss.")
    legend(fig, [text for _, text in LOSS_CAUSES], SERIES, 4)
    return save(fig, out)


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


def plot_events(rows, heads, label, out):
    names = [n for n, _ in rows]
    events = (("mispredicts", "Mispredictions"), ("icache_misses", "Instruction cache misses"),
              ("dcache_misses", "Data cache misses"))
    height = 1.0 + len(rows) * ROW_IN + 0.5
    fig, axes = plt.subplots(1, 3, figsize=(10, height), dpi=DPI, sharey=True)
    fig.patch.set_facecolor(SURFACE)
    fig.subplots_adjust(left=0.15, right=0.98, top=1 - 1.0 / height, bottom=0.45 / height,
                        wspace=0.12)
    ys = rows_layout(len(names), heads)
    for ax, (key, title) in zip(axes, events):
        values = [per_kilo_insn(c, key) for _, c in rows]
        top = max(values) or 1.0
        ax.set_ylim(ys[-1] + 0.8, -0.8)
        ax.set_xlim(0, top * 1.3)
        style(ax)
        ax.barh(ys, values, height=bar_height(ax), color=SERIES[0])
        for y, v in zip(ys, values):
            if v >= 0.25 * top:
                ax.text(v + top * 0.02, y, f"{v:.1f}", va="center", fontsize=FONT - 1,
                        color=INK_SECONDARY)
        ax.set_title(title, fontsize=FONT + 1, color=INK, loc="left")
        ax.set_xlabel("per 1000 instructions", fontsize=FONT, color=INK_MUTED)
    axes[0].set_yticks(ys, names)
    for tick in axes[0].get_yticklabels()[:heads]:
        tick.set_fontweight("bold")
    titles(fig, f"Events: {label}", "Per 1000 retired instructions in the measured regions; "
           "values labelled from a quarter of each panel's maximum.")
    return save(fig, out)


def plot_histograms(redirect, load, label, out):
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.4), dpi=DPI)
    fig.patch.set_facecolor(SURFACE)
    fig.subplots_adjust(left=0.07, right=0.98, top=0.72, bottom=0.17, wspace=0.18)
    panels = (
        (axes[0], redirect, 0, "Redirect penalty",
         "cycles from a redirect to the next instruction offered", None),
        (axes[1], load, 1, "Load stalls",
         "cycles one slot waits for one load", LOAD_HIT_MAX_STALL),
    )
    for ax, hist, first, title, xlabel, threshold in panels:
        last = max((i for i, n in enumerate(hist) if n), default=0)
        shown = max(last, (threshold or 0) + 1) + 1
        xs = [first + i for i in range(shown)]
        style(ax, axis="y")
        ax.set_yscale("log")
        ax.bar(xs, hist[:shown], width=0.7, color=SERIES[0])
        ax.set_xlim(first - 0.8, xs[-1] + 0.8)
        open_bin = len(hist) - 1
        ticks = [x for x in xs if (x - first) % max(1, shown // 8) == 0]
        if xs[-1] - first == open_bin:
            # The open last bin always gets its label, replacing a crowding neighbour
            ticks = [x for x in ticks if xs[-1] - x >= max(1, shown // 8) / 2] + [xs[-1]]
        ax.set_xticks(ticks, [f"≥{x}" if x - first == open_bin else str(x) for x in ticks])
        if threshold:
            ax.axvline(threshold + 0.5, color=INK_SECONDARY, linewidth=PT_PER_PX)
            ax.text(threshold + 0.3, 0.97, "hit", transform=ax.get_xaxis_transform(), ha="right",
                    va="top", fontsize=FONT - 1, color=INK_SECONDARY)
            ax.text(threshold + 0.7, 0.97, "miss", transform=ax.get_xaxis_transform(), ha="left",
                    va="top", fontsize=FONT - 1, color=INK_SECONDARY)
        ax.set_title(title, fontsize=FONT + 1, color=INK, loc="left")
        ax.set_xlabel(xlabel, fontsize=FONT, color=INK_MUTED)
        ax.set_ylabel("occurrences (log scale)", fontsize=FONT, color=INK_MUTED)
    titles(fig, f"Histograms: {label}", "Summed over all benchmarks with cycle accounting. "
           "The last bin of each histogram collects everything at and above it.")
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

    acc = accounted(run)
    rows = []
    if acc:
        everything = total([c for _, c in acc])
        acc.sort(key=lambda rc: -share(rc[1], "retiring"))
        # Totals on top: one per suite, or one of all benchmarks
        suites = by_suite({r["name"]: c for r, c in acc})
        if len(suites) > 1:
            heads = [(f"[{t}]", total(list(v.values()))) for t, v in suites.items()]
        else:
            heads = [("[all]", everything)]
        rows = heads + [(r["name"], c) for r, c in acc]
        ports = acc[0][0]["perf"]["issue_ports"]
        written += [
            plot_slots(rows, len(heads), label, ports, out / f"slots.{args.format}"),
            plot_losses(rows, len(heads), label, out / f"losses.{args.format}"),
            plot_events(rows, len(heads), label, out / f"events.{args.format}"),
            plot_histograms(summed_hist([r for r, _ in acc], "redirect_penalty_hist"),
                            summed_hist([r for r, _ in acc], "load_stall_hist"), label,
                            out / f"histograms.{args.format}"),
        ]
        with open(out / "histograms.csv", "w", newline="") as f:
            redirect = summed_hist([r for r, _ in acc], "redirect_penalty_hist")
            load = summed_hist([r for r, _ in acc], "load_stall_hist")
            writer = csv.writer(f)
            writer.writerow(["cycles", "redirect_penalty", "load_stall"])
            for i in range(max(len(redirect), len(load) + 1)):
                writer.writerow([i, redirect[i] if i < len(redirect) else "",
                                 load[i - 1] if 0 < i <= len(load) else ""])
        written.append(out / "histograms.csv")

        by_name = {r["name"]: r for r in passed}
        with open(out / "breakdown.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["benchmark", "ipc", "cycles", "instret", *TOPDOWN,
                             *(f"loss.{k}" for k, _ in LOSS_CAUSES),
                             "mispredicts_pki", "icache_misses_pki", "dcache_misses_pki"])
            for name, c in rows:
                r = by_name.get(name, {})
                writer.writerow([name, r.get("ipc", ""),
                                 r.get("cycles", ""), r.get("instret", ""),
                                 *(f"{share(c, k):.4f}" for k in TOPDOWN),
                                 *(f"{share(c, f'loss.{k}'):.4f}" for k, _ in LOSS_CAUSES),
                                 *(f"{per_kilo_insn(c, e):.3f}"
                                   for e in ("mispredicts", "icache_misses", "dcache_misses"))])
        written.append(out / "breakdown.csv")
    else:
        print("no cycle accounting in this run (build the model with PERF=1): plotting IPC only")

    if rows:
        print(f"{label}: share of issue slots, all {len(acc)} benchmarks together")
        print("  " + ", ".join(f"{k} {100 * share(everything, k):.1f}%" for k in TOPDOWN))
        print_breakdown(everything)
        if len(heads) > 1:
            print("by suite:")
            for name, c in heads:
                print(f"  {name:<16} " + ", ".join(f"{k} {100 * share(c, k):.1f}%" for k in TOPDOWN))
    print(f"geomean IPC over {len(passed)} benchmarks: {geo:.4f}")
    suite_ipc = by_suite({r["name"]: r["ipc"] for r in passed})
    if len(suite_ipc) > 1:
        print("  by suite: " + ", ".join(f"{t} {geomean(v.values()):.4f}"
                                         for t, v in suite_ipc.items()))
    print("written: " + ", ".join(str(p) for p in written))


if __name__ == "__main__":
    main()
