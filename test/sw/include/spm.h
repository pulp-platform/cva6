// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Helpers for the L1 scratchpad (SPM) mode of the cva6rt caches.
//
// Address layout of the data SPM: a contiguous window whose way is selected by
// the address bits just above the cache index, so way N covers
// DSPM_BASE + N*DSPM_WAY_SIZE .. +DSPM_WAY_SIZE-1.

#pragma once

#include <stdint.h>

#define DSPM_BASE      0x01800000UL
#define ISPM_BASE      0x01A00000UL
#define DCACHE_SIZE    32768UL
// The I$ size of the hardware build (CVA6_ICACHE_BYTE_SIZE in the Makefiles), in 4 KiB ways
#ifndef ICACHE_SIZE
#define ICACHE_SIZE    16384UL
#endif
#define DCACHE_WAYS    8
#define ICACHE_WAYS    ((ICACHE_SIZE) / 4096)
#define DSPM_WAY_SIZE  ((DCACHE_SIZE) / (DCACHE_WAYS))
#define ISPM_WAY_SIZE  ((ICACHE_SIZE) / (ICACHE_WAYS))

#define CSR_DCACHE_SPM_WAYS 0x5E0
#define CSR_ICACHE_SPM_WAYS 0x5E1
#define CSR_DCACHE_EN       0x7C1
#define CSR_ICACHE_EN       0x7C0

// Address of word `i` (64-bit) inside data-SPM way `w`
#define DSPM_WORD(w, i) \
    ((volatile uint64_t *)(DSPM_BASE + (w) * DSPM_WAY_SIZE + (i) * sizeof(uint64_t)))

// Configure `mask` ways of the D-cache as scratchpad. The cache is disabled and
// flushed around the change, as the RT tests do: ways switching mode must not
// keep stale cached lines. The fences keep SPM stores just before the disable
// and SPM loads just after the enable from running with the cache disabled.
// As one asm string, tests can place SPM accesses right next to it; the mask is
// operand %[mask].
#define DSPM_SET_WAYS_ASM \
    "fence\n csrwi 0x7C1, 0\n fence.i\n csrw 0x5E0, %[mask]\n csrwi 0x7C1, 1\n fence\n"

static inline void dspm_set_ways(uint32_t mask) {
    asm volatile(DSPM_SET_WAYS_ASM ::[mask] "r"(mask) : "memory");
}

static inline void ispm_set_ways(uint32_t mask) {
    asm volatile("csrwi 0x7C0, 0" ::: "memory");
    asm volatile("fence.i" ::: "memory");
    asm volatile("csrw  0x5E1, %0" ::"r"(mask) : "memory");
    asm volatile("csrwi 0x7C0, 1" ::: "memory");
}

// Value returned by the SPM controllers for an address whose way is not
// currently configured as scratchpad.
#define SPM_INACTIVE_MAGIC 0xCA11AB1EBADCAB1EULL

// With every way as scratchpad, cached (DRAM) accesses go to memory uncached
// (dcache_spm_allways). DSPM_SAFE_WAYS is half of the ways: the tests that use
// it keep the other half caching their own data.
#define DSPM_SAFE_WAYS 4
#define DSPM_SAFE_MASK ((1u << DSPM_SAFE_WAYS) - 1)
