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
#define ICACHE_SIZE    16384UL
#define DCACHE_WAYS    8
#define ICACHE_WAYS    4
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
// keep stale cached lines.
static inline void dspm_set_ways(uint32_t mask) {
    asm volatile("csrwi 0x7C1, 0" ::: "memory");
    asm volatile("fence.i" ::: "memory");
    asm volatile("csrw  0x5E0, %0" ::"r"(mask) : "memory");
    asm volatile("csrwi 0x7C1, 1" ::: "memory");
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

// NOTE: configuring *all* ways as scratchpad leaves the cache with no way to
// refill into, and ordinary cached (DRAM) accesses then misbehave - globals,
// stack and printf state included. Tests that touch anything outside the SPM
// must leave at least one way to the cache. Half the ways is the safe default.
#define DSPM_SAFE_WAYS 4
#define DSPM_SAFE_MASK ((1u << DSPM_SAFE_WAYS) - 1)
