// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

#include <stdint.h>

#include "bench.h"
#include "printf.h"

extern char __base_regs[];

static uint64_t roi_cycle_start, roi_instret_start;
static uint64_t roi_cycles, roi_instret;
static int roi_state; // 0: never opened, 1: open, 2: closed

void __attribute__((noinline)) bench_roi_start(void) {
    BENCH_MARKER(BENCH_MARKER_ROI_START);
    roi_state = 1;
    roi_instret_start = bench_read_minstret();
    roi_cycle_start = bench_read_mcycle();
}

void __attribute__((noinline)) bench_roi_stop(void) {
    uint64_t cycle = bench_read_mcycle();
    uint64_t instret = bench_read_minstret();
    BENCH_MARKER(BENCH_MARKER_ROI_STOP);
    roi_cycles = cycle - roi_cycle_start;
    roi_instret = instret - roi_instret_start;
    roi_state = 2;
}

void bench_report(void) {
    if (roi_state != 2) {
        printf("[BENCH] roi not closed\n");
        return;
    }
    printf("[BENCH] roi cycles=%lu instret=%lu\n", (unsigned long)roi_cycles,
           (unsigned long)roi_instret);
}

// Called by the trap vector in crt0.S. Benchmarks never trap, so any trap is a
// failure: report it and end the simulation through the EOC register, with a
// return code that cannot be mistaken for a benchmark's own (0x100 + mcause).
void __attribute__((noreturn)) bench_trap(uint64_t mcause, uint64_t mepc, uint64_t mtval) {
    printf("[BENCH] trap mcause=0x%lx mepc=0x%lx mtval=0x%lx\n", (unsigned long)mcause,
           (unsigned long)mepc, (unsigned long)mtval);
    uint32_t code = 0x100 | (uint32_t)(mcause & 0xff);
    *(volatile uint32_t *)(__base_regs + 4) = (code << 1) | 1;
    for (;;)
        asm volatile("wfi");
}
