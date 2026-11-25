// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// SPM accesses right next to the D-cache way switch of spm.h: stores to way 0
// directly before it, alone (A) or queued behind an uncached store (B), and
// loads directly after it. Way 0 stays SPM while way 1 switches.
//
// Return codes: 0 pass, 10 + 10 * round + word (case A) or 50 + 10 * round +
// word (case B) for a word that does not read back.

#include <stdint.h>
#include "spm.h"

#define DRAM_MASK          0x3fffffffUL
#define DRAM_UNCACHED_BASE 0x40000000UL

static volatile uint64_t dram_word;

static inline void stores_then_switch(uint64_t v, uint64_t m) {
    asm volatile("sd %1, 0(%0)\n sd %1, 8(%0)\n sd %1, 16(%0)\n sd %1, 24(%0)\n" DSPM_SET_WAYS_ASM
                 :: "r"(DSPM_WORD(0, 0)), "r"(v), [mask] "r"(m) : "memory");
}

static inline void uc_stores_then_switch(volatile uint64_t *uc, uint64_t v, uint64_t m) {
    asm volatile("sd %1, 0(%2)\n"
                 "sd %1, 0(%0)\n sd %1, 8(%0)\n sd %1, 16(%0)\n sd %1, 24(%0)\n" DSPM_SET_WAYS_ASM
                 :: "r"(DSPM_WORD(0, 0)), "r"(v), "r"(uc), [mask] "r"(m) : "memory");
}

static int check(uint64_t v, int code) {
    for (unsigned int i = 0; i < 4; i++) {
        if (*DSPM_WORD(0, i) != v) return code + i;
    }
    return 0;
}

int main(void) {
    volatile uint64_t *uc = (volatile uint64_t *)
        (((uintptr_t)&dram_word & DRAM_MASK) | DRAM_UNCACHED_BASE);
    int ret = 0;
    dspm_set_ways(0x1);
    for (unsigned int r = 0; r < 4 && !ret; r++) {
        stores_then_switch(0x1000 * r + 7, (r & 1) ? 0x1 : 0x3);
        ret = check(0x1000 * r + 7, 10 + 10 * r);
        if (ret) break;
        uc_stores_then_switch(uc, 0x2000 * r + 9, (r & 1) ? 0x3 : 0x1);
        ret = check(0x2000 * r + 9, 50 + 10 * r);
    }
    dspm_set_ways(0);
    return ret;
}
