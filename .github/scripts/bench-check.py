#!/usr/bin/env python3
# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
"""Run the GitLab CI's benchmark check for a commit and report it on GitHub.

The GitLab pipeline of every commit holds the benchmark check (veri-bench-check,
one job per configuration) in its child pipeline, as manual jobs allowed to
fail. This starts them in the commit's newest pipeline, waits for them, and
passes only if they all succeeded. The GitHub run's summary lists them, with the
verdict and the benchmarks off their expected cycles from each failed job's log.

    bench-check.py <sha>

GITLAB_DOMAIN, GITLAB_REPO and GITLAB_TOKEN name the GitLab project and a token
that may start its jobs (api scope); the summary goes to $GITHUB_STEP_SUMMARY.
"""

import itertools
import json
import os
import re
import sys
import time
import urllib.parse
import urllib.request

JOB_PREFIX = "veri-bench-check"
CHECK_LOG = "test/verilator/bench-check.log"
FINISHED = ("success", "failed", "canceled", "skipped")
POLL_PERIOD = 60
# Polls for the commit's pipeline to appear
SPAWN_POLLS = 30
# Failed polls in a row (GitLab unreachable, a job that cannot be started)
# before giving up
FAILED_POLLS = 10


def api(path, raw=False, method="GET"):
    """A GitLab API call: the response's JSON, or its text if raw."""
    req = urllib.request.Request(f"https://{os.environ['GITLAB_DOMAIN']}/api/v4/{path}",
                                 headers={"PRIVATE-TOKEN": os.environ["GITLAB_TOKEN"]},
                                 method=method)
    with urllib.request.urlopen(req, timeout=60) as resp:
        body = resp.read().decode("utf-8", errors="replace")
    return body if raw else json.loads(body)


def bench_jobs(repo, sha):
    """The commit's newest pipeline and the benchmark check's jobs in its child pipelines."""
    pipelines = api(f"projects/{repo}/pipelines?sha={sha}&order_by=id&sort=desc")
    if not pipelines:
        return None, []
    jobs = []
    for bridge in api(f"projects/{repo}/pipelines/{pipelines[0]['id']}/bridges"):
        child = bridge.get("downstream_pipeline")
        page = 1
        while child:
            batch = api(f"projects/{child['project_id']}/pipelines/{child['id']}"
                        f"/jobs?per_page=100&page={page}")
            jobs += [j for j in batch if j["name"].startswith(JOB_PREFIX)]
            if len(batch) < 100:
                break
            page += 1
    return pipelines[0], jobs


def wait_for_jobs(repo, sha):
    """Start the commit's benchmark check jobs and wait until all finished.

    Returns the pipeline (None if none appeared), its jobs (none if it finished
    without them) and an error if GitLab failed too many polls in a row.
    """
    pipeline, jobs, failed = None, [], 0
    for poll in itertools.count(1):
        try:
            pipeline, jobs = bench_jobs(repo, sha)
            for j in jobs:
                if j["status"] == "manual":
                    api(f"projects/{j['pipeline']['project_id']}/jobs/{j['id']}/play",
                        raw=True, method="POST")
                    print(f"started {j['name']}", flush=True)
            failed = 0
        except (OSError, ValueError) as e:
            failed += 1
            print(f"poll {poll} failed: {e}", flush=True)
            if failed == FAILED_POLLS:
                return pipeline, jobs, f"GitLab failed {failed} polls in a row, the last with: {e}"
        if jobs and all(j["status"] in FINISHED for j in jobs):
            return pipeline, jobs, None
        if not jobs and pipeline and pipeline["status"] in FINISHED:
            return pipeline, jobs, None
        if not pipeline and poll == SPAWN_POLLS:
            return pipeline, jobs, None
        time.sleep(POLL_PERIOD)


def check_report(log):
    """The rows of the check's table with benchmarks off their expected cycles,
    and its verdict, from the report run-bench.py prints."""
    rows = []
    for line in log.splitlines():
        cols = line.split()
        # benchmark  expected  cycles  change  [note], where change is "=" when
        # unchanged, a percentage when changed and "-" when not comparable
        header = cols[:4] == ["benchmark", "expected", "cycles", "change"]
        changed = len(cols) >= 4 and cols[1].isdigit() and re.fullmatch(r"[+-]\d+\.\d+%|-", cols[3])
        if header or changed or line.startswith("**BENCH CHECK"):
            rows.append(line.rstrip())
    return rows


def report(sha, pipeline, jobs, error):
    """The summary, in Markdown."""
    out = ["## Benchmark check", ""]
    if not pipeline:
        out.append(f"No GitLab pipeline for {sha}.")
    elif not jobs:
        out.append(f"GitLab pipeline {pipeline['web_url']} has no {JOB_PREFIX} job.")
    else:
        out += [f"GitLab pipeline: {pipeline['web_url']}", ""]
        out += ["| job | status |", "| --- | --- |"]
        out += [f"| `{j['name']}` | [{j['status']}]({j['web_url']}) |"
                for j in sorted(jobs, key=lambda j: j["name"])]
    if error:
        out += ["", error]
    for j in jobs:
        if j["status"] == "failed":
            try:
                rows = check_report(api(f"projects/{j['pipeline']['project_id']}/jobs/{j['id']}"
                                        f"/artifacts/{CHECK_LOG}", raw=True))
            except OSError as e:
                rows = [f"cannot read {CHECK_LOG}: {e}"]
            out += ["", f"`{j['name']}`:", "", "```", *rows, "```"]
    return out


def main():
    repo = urllib.parse.quote_plus(os.environ["GITLAB_REPO"])
    sha = sys.argv[1]
    pipeline, jobs, error = wait_for_jobs(repo, sha)
    out = "\n".join(report(sha, pipeline, jobs, error)) + "\n"
    print(out)
    if os.environ.get("GITHUB_STEP_SUMMARY"):
        with open(os.environ["GITHUB_STEP_SUMMARY"], "a") as f:
            f.write(out)
    passed = bool(jobs) and not error and all(j["status"] == "success" for j in jobs)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
