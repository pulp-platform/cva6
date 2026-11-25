// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// All D-cache ways as SPM: DRAM loads, stores, an AMO and calls go uncached,
// the SPM ways hold their data, and DRAM keeps its contents across the switches.

#include <stdint.h>
#include "spm.h"

#define N 64

static volatile uint64_t dram_buf[N];

static uint64_t __attribute__((noinline)) sum(volatile uint64_t *p, unsigned int n) {
    uint64_t s = 0;
    for (unsigned int i = 0; i < n; i++) s += p[i];
    return s;
}

int main(void) {
    uint64_t expect = 0;
    for (unsigned int i = 0; i < N; i++) dram_buf[i] = 3 * i;

    dspm_set_ways((1u << DCACHE_WAYS) - 1);
    for (unsigned int i = 0; i < N; i++) dram_buf[i] += 1;
    for (unsigned int i = 0; i < N; i++) *DSPM_WORD(i % DCACHE_WAYS, i) = i ^ 0x55;
    __atomic_fetch_add(&dram_buf[0], 5, __ATOMIC_SEQ_CST);
    for (unsigned int i = 0; i < N; i++) expect += 3 * i + 1;
    expect += 5;
    if (sum(dram_buf, N) != expect) return 1;
    for (unsigned int i = 0; i < N; i++) {
        if (*DSPM_WORD(i % DCACHE_WAYS, i) != (i ^ 0x55)) return 2 + i;
    }

    dspm_set_ways(0);
    if (sum(dram_buf, N) != expect) return 100;
    return 0;
}
