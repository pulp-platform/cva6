// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Benchmark support: region-of-interest (ROI) measurement and reporting.
//
// A benchmark brackets the code it wants measured with bench_roi_start() and
// bench_roi_stop(). Both read mcycle and minstret, so the deltas are what the
// core itself counts. Each also retires a marker instruction, a custom-use
// HINT (slti x0, x0, <id>) that executes as a nop on any RISC-V hart but shows
// up in the RVFI retirement stream, so a testbench-side observer can measure
// the same region without trusting the core's counters.

#pragma once

#include <stdint.h>

#define BENCH_MARKER_ROI_START 1
#define BENCH_MARKER_ROI_STOP  2

#define BENCH_MARKER(id) asm volatile("slti x0, x0, %0" ::"i"(id) : "memory")

static inline uint64_t bench_read_mcycle(void) {
    uint64_t value;
    asm volatile("csrr %0, mcycle" : "=r"(value)::"memory");
    return value;
}

static inline uint64_t bench_read_minstret(void) {
    uint64_t value;
    asm volatile("csrr %0, minstret" : "=r"(value)::"memory");
    return value;
}

// Open and close the measured region. Only one region per program.
void bench_roi_start(void);
void bench_roi_stop(void);

// Print the region's counters as one machine-readable line, parsed by
// util/run-bench.py:
//   [BENCH] roi cycles=<n> instret=<n>
void bench_report(void);
