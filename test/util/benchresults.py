# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
"""Reading benchmark results.

run-bench.py writes one results.json per run: the run's metadata, and per
benchmark its status and region-of-interest counters. Everything derived from
them is defined here, so that the scripts showing a run and those comparing
runs agree.

Only the standard library is used: run-bench.py imports this module from the
simulation environment, the plotting scripts from the uv environment.
"""

import json
import math
from pathlib import Path

RESULTS_SCHEMA = 1


# Suites, by name prefix. The suites measure different things (the pipeline,
# the memory, the frontend) and number from 1 to 22 programs: a geometric mean
# over all benchmarks says little, one per suite more.
SUITES = (
    ("coremark", "CoreMark"),
    ("em-", "Embench"),
    ("dhrystone", "Dhrystone"),
    ("rs-", "RaiderSTREAM"),
)


def suite(name):
    return next((title for prefix, title in SUITES if name.startswith(prefix)), "Other")


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
