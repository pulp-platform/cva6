# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
"""Reading benchmark results, and what the cycle accounting means.

run-bench.py writes one results.json per run: the run's metadata, and per
benchmark its status, region-of-interest counters and, on a model built with
PERF=1, the testbench's raw cycle accounting (hw/cva6tb_perf.sv's perf.json).
Everything derived from those raw counts is defined here, so that the scripts
showing a run and those comparing runs agree, and so that a change of
definition applies to existing results without re-running them.

Only the standard library is used: run-bench.py imports this module from the
simulation environment, the plotting scripts from the uv environment.
"""

import json
import math
from pathlib import Path

RESULTS_SCHEMA = 1

# Issue-slot classes of the cycle accounting, and how they add up to the four
# top-level categories:
#   retiring   slots of instructions that retired
#   bad_spec   slots of instructions issued and then squashed, and slots lost
#              refilling the frontend after a redirect
#   frontend   slots without an instruction otherwise
#   backend    slots with an instruction that could not issue
BACKEND_CLASSES = ("sb_full", "blocked", "stall", "fu_busy", "raw", "other")
TOPDOWN = ("retiring", "bad_spec", "frontend", "backend")

# A stall waiting for a load that hits in the data cache lasts the load-to-use
# latency, a few cycles; one waiting for a miss lasts at least a memory round
# trip, some tens of cycles. Stalls up to this many cycles count as hits; the
# histogram in perf.json shows the two populations.
LOAD_HIT_MAX_STALL = 8

# The lost slots by cause: eight causes that partition all lost slots, so that
# with retiring they add up to all slots. Instruction cache misses are one
# cause whether they starve the frontend or prolong a redirect's recovery.
LOSS_CAUSES = (
    ("load_use", "Waiting for a load"),
    ("other_dependency", "Waiting for another result"),
    ("blocked", "Behind an older slot"),
    ("unit_busy", "Unit busy or taken"),
    ("mispredict", "Misprediction recovery"),
    ("icache_miss", "Instruction cache miss"),
    ("fetch", "Fetch, other"),
    ("other", "Scoreboard full, other"),
)


# Suites, by name prefix. The suites measure different things (the pipeline,
# the memory, the frontend) and number from 1 to 22 programs: a geometric mean
# over all benchmarks says little, one per suite more.
SUITES = (
    ("coremark", "CoreMark"),
    ("em-", "Embench"),
    ("dhrystone", "Dhrystone"),
    ("rs-", "RaiderSTREAM"),
    ("chase-", "Pointer chase"),
    ("lua-", "Lua"),
)


def suite(name):
    return next((title for prefix, title in SUITES if name.startswith(prefix)), "Other")


# Scores of a run against a baseline, each the geometric mean of some suites'
# geometric mean speedups, every suite weighted alike: what the core computes,
# and what the memory system serves. Dhrystone is a reference only.
SCORES = (
    ("compute", ("CoreMark", "Embench", "Lua")),
    ("memory", ("RaiderSTREAM", "Pointer chase")),
)


def scores(suite_speedups):
    """{score: value} from {suite: geometric mean speedup}; a score is None
    unless every one of its suites is there."""
    return {name: (geomean(suite_speedups[t] for t in suites)
                   if all(suite_speedups.get(t) for t in suites) else None)
            for name, suites in SCORES}


def by_suite(values):
    """{benchmark: value} split by suite, in SUITES' order: {suite: {benchmark: value}}."""
    order = [title for _, title in SUITES] + ["Other"]
    split = {}
    for name in sorted(values, key=lambda n: (order.index(suite(n)), n)):
        split.setdefault(suite(name), {})[name] = values[name]
    return split


def load_run(path):
    """A run's results: a results.json, or the directory holding it."""
    path = Path(path)
    if path.is_dir():
        path = path / "results.json"
    run = json.loads(path.read_text())
    schema = run.get("schema", 1)
    if schema > RESULTS_SCHEMA:
        raise ValueError(f"{path}: results schema {schema}, newer than this script's "
                         f"{RESULTS_SCHEMA}")
    run["path"] = str(path)
    return run


def run_label(run):
    meta = run.get("meta", {})
    return meta.get("tag") or meta.get("config") or Path(run["path"]).parent.name


def check_perf(perf):
    """Every slot is in exactly one class: each port's classes sum to the
    cycles. Returns an error message, or None."""
    for i, port in enumerate(perf["slots"]):
        if sum(port.values()) != perf["cycles"]:
            return f"perf: port {i} classes sum to {sum(port.values())}, not {perf['cycles']} cycles"
    return None


def topdown(perf):
    """Slot counts per top-level category, and below them. Counts named
    <category>.<cause> break their category down; those named
    <category>.<cause>.<detail> break a cause down further, and need not cover
    it all. events.* are not slots. loss.* partition the lost slots."""
    slot = {c: sum(port[c] for port in perf["slots"]) for c in perf["slots"][0]}
    retiring = min(perf["retired"], slot["issued"])
    counts = {
        "slots": perf["issue_ports"] * perf["cycles"],
        "retiring": retiring,
        "bad_spec": slot["issued"] - retiring + slot["recovery"],
        "frontend": slot["frontend"],
        "backend": sum(slot[c] for c in BACKEND_CLASSES),
        "bad_spec.squashed": slot["issued"] - retiring,
        "bad_spec.recovery": slot["recovery"],
        "bad_spec.recovery.icache_miss": perf["recovery_icache_miss"],
        "frontend.icache_miss": perf["frontend_icache_miss"],
        "frontend.other": slot["frontend"] - perf["frontend_icache_miss"],
        "events.cycles": perf["cycles"],
        "events.retired": perf["retired"],
        "events.icache_misses": perf["icache_misses"],
        "events.dcache_misses": perf["dcache_misses"],
        "events.mispredicts": perf["mispredicts"],
    }
    counts.update({f"backend.{c}": slot[c] for c in BACKEND_CLASSES})
    counts.update({f"backend.raw.{fu}": n for fu, n in perf["raw_by_producer_fu"].items() if n})
    counts["backend.raw.in_group"] = perf["raw_in_group"]
    hit = sum((k + 1) * n for k, n in enumerate(perf["load_stall_hist"])
              if k + 1 <= LOAD_HIT_MAX_STALL)
    load = perf["raw_by_producer_fu"]["load"]
    counts["backend.raw.load.hit"] = min(hit, load)
    counts["backend.raw.load.miss"] = load - min(hit, load)
    counts.update({f"backend.fu_busy.{fu}": n for fu, n in perf["fu_busy_by_fu"].items() if n})
    counts["backend.fu_busy.in_group"] = perf["fu_busy_in_group"]
    counts.update({f"backend.blocked.{c}": n for c, n in perf["blocked_by"].items() if n})

    icache = counts["bad_spec.recovery.icache_miss"] + counts["frontend.icache_miss"]
    counts.update({
        "loss.load_use": load,
        "loss.other_dependency": slot["raw"] - load,
        "loss.blocked": slot["blocked"],
        "loss.unit_busy": slot["fu_busy"],
        "loss.mispredict": counts["bad_spec"] - counts["bad_spec.recovery.icache_miss"],
        "loss.icache_miss": icache,
        "loss.fetch": counts["frontend.other"],
        "loss.other": slot["sb_full"] + slot["stall"] + slot["other"],
    })
    return counts


def accounted(run):
    """The passing benchmarks of a run that have cycle accounting, with their
    counts derived: [(result, counts)]."""
    return [(r, topdown(r["perf"])) for r in run["results"]
            if r["status"] == "PASS" and r.get("perf")]


def total(counts_list):
    """Counts of several benchmarks summed: all of them together, each weighted
    by its slots."""
    keys = {k for c in counts_list for k in c}
    return {k: sum(c.get(k, 0) for c in counts_list) for k in keys}


def geomean(values):
    values = list(values)
    return math.exp(sum(map(math.log, values)) / len(values)) if values else None


def speedups(base, run):
    """Speedup of run over base per benchmark, from the regions' cycles:
    {name: base cycles / run cycles}. Only benchmarks passing in both, and
    running the same binary (the same retired instructions), are compared.
    Also returns the names passing in both whose instruction counts differ."""
    a = {r["name"]: r for r in base["results"] if r["status"] == "PASS"}
    b = {r["name"]: r for r in run["results"] if r["status"] == "PASS"}
    common = sorted(set(a) & set(b))
    same = [n for n in common if a[n]["instret"] == b[n]["instret"]]
    return ({n: a[n]["cycles"] / b[n]["cycles"] for n in same},
            [n for n in common if n not in same])


def cpi_stack(counts):
    """Cycles per instruction, split into the cycles the retired instructions
    take at the core's issue width (\"base\": 1/width), and the lost slots of
    each cause, in cycles. They add up to the CPI, so stacks of cores of
    different widths compare directly: a shorter stack is a faster core."""
    retired = counts["events.retired"]
    width = counts["slots"] / counts["events.cycles"]
    stack = {"base": counts["retiring"] / width / retired}
    stack.update({key: counts[f"loss.{key}"] / width / retired for key, _ in LOSS_CAUSES})
    return stack


def share(counts, key):
    return counts.get(key, 0) / counts["slots"] if counts["slots"] else 0.0


def per_kilo_insn(counts, event):
    kilo = counts["events.retired"] / 1000
    return counts[f"events.{event}"] / kilo if kilo else 0.0


def summed_hist(results, key):
    """A histogram of perf.json summed over benchmarks."""
    hists = [r["perf"][key] for r in results]
    return [sum(h[i] for h in hists) for i in range(len(hists[0]))] if hists else []


def print_breakdown(td):
    """The categories below the top level, as shares of all issue slots."""
    pct = lambda k: f"{100 * share(td, k):.1f}%"

    def causes(prefix, exclude=()):
        keys = [k for k in td if k.startswith(prefix) and k.count(".") == prefix.count(".")
                and td[k] and k[len(prefix):] not in exclude]
        return ", ".join(f"{k[len(prefix):]} {pct(k)}"
                         for k in sorted(keys, key=lambda k: -td[k])) or "none"

    print(f"  bad_spec {pct('bad_spec')}: {causes('bad_spec.')} "
          f"(recovery on icache misses {pct('bad_spec.recovery.icache_miss')})")
    print(f"  frontend {pct('frontend')}: {causes('frontend.')}")
    print(f"  backend {pct('backend')}: {causes('backend.')}")
    print(f"    raw, by producing unit: {causes('backend.raw.', ('in_group',))}; "
          f"producer in the same issue group {pct('backend.raw.in_group')}")
    print(f"      loads: hit {pct('backend.raw.load.hit')}, miss {pct('backend.raw.load.miss')}")
    print(f"    fu_busy, by unit: {causes('backend.fu_busy.', ('in_group',))}; "
          f"taken by the same issue group {pct('backend.fu_busy.in_group')}")
    print(f"    blocked, by the older slot's class: {causes('backend.blocked.')}")
    print(f"  per 1000 instructions: {per_kilo_insn(td, 'mispredicts'):.2f} mispredicts, "
          f"{per_kilo_insn(td, 'icache_misses'):.2f} icache misses, "
          f"{per_kilo_insn(td, 'dcache_misses'):.2f} dcache misses")
