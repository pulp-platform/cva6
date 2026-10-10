// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Stands in for riscv-tests' benchmarks/common/util.h, which Dhrystone's main
// includes: its statistics hooks around the timed runs become the measured
// region, and its printf the testbench console's.

#pragma once

#include "bench.h"
#include "printf.h"

#define read_csr(reg)                                                                              \
    ({                                                                                             \
        unsigned long __v;                                                                         \
        asm volatile("csrr %0, " #reg : "=r"(__v));                                                \
        __v;                                                                                       \
    })

static inline void setStats(int enable) {
    if (enable)
        bench_roi_start();
    else
        bench_roi_stop();
}
