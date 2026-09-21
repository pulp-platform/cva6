// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Data-SPM functional test: access widths, sub-word offsets, way boundaries and
// partial SPM configurations. Complements dcache_spm_simple.c, which only does
// aligned 64-bit accesses with all ways configured as scratchpad.
//
// Returns 0 on success, otherwise the number of the failing check.

#include <stdint.h>
#include "spm.h"
#include "printf.h"

static int checks = 0;

#define CHECK(cond)             \
    do {                        \
        checks++;               \
        if (!(cond)) {          \
            printf("FAIL check %d (line %d)\n", checks, __LINE__); \
            return checks;      \
        }                       \
    } while (0)

// --- 1. every access width, and byte merging within a word ---------------
static int test_widths(void) {
    volatile uint8_t  *b = (volatile uint8_t *)DSPM_BASE;
    volatile uint16_t *h = (volatile uint16_t *)DSPM_BASE;
    volatile uint32_t *w = (volatile uint32_t *)DSPM_BASE;
    volatile uint64_t *d = (volatile uint64_t *)DSPM_BASE;

    *d = 0x0123456789abcdefULL;
    CHECK(*d == 0x0123456789abcdefULL);
    CHECK(w[0] == 0x89abcdefUL);
    CHECK(w[1] == 0x01234567UL);
    CHECK(h[0] == 0xcdefU);
    CHECK(h[3] == 0x0123U);
    CHECK(b[0] == 0xefU);
    CHECK(b[7] == 0x01U);

    // Byte and halfword writes must merge into the surrounding word: the SPM
    // writes into a cache-line-wide SRAM through byte enables.
    b[2] = 0x5a;
    CHECK(*d == 0x01234567895acdefULL);
    h[2] = 0xbeef;
    CHECK(*d == 0x0123beef895acdefULL);
    w[1] = 0xdeadc0de;
    CHECK(*d == 0xdeadc0de895acdefULL);
    return 0;
}

// --- 2. way boundaries ---------------------------------------------------
// Way N covers DSPM_BASE + N*DSPM_WAY_SIZE. Write a distinct pattern to the
// first and last word of every way, then read them all back: a broken way
// index shows up as aliasing between ways.
static int test_ways(void) {
    for (int way = 0; way < DSPM_SAFE_WAYS; way++) {
        *DSPM_WORD(way, 0) = 0x1000ULL + way;
        *DSPM_WORD(way, DSPM_WAY_SIZE / sizeof(uint64_t) - 1) = 0x2000ULL + way;
    }
    for (int way = 0; way < DSPM_SAFE_WAYS; way++) {
        CHECK(*DSPM_WORD(way, 0) == 0x1000ULL + way);
        CHECK(*DSPM_WORD(way, DSPM_WAY_SIZE / sizeof(uint64_t) - 1) == 0x2000ULL + way);
    }
    return 0;
}

// --- 3. partial configuration: SPM and cache side by side ----------------
static uint64_t dram_buf[256] __attribute__((aligned(64)));

static int test_partial(void) {
    // Narrow the scratchpad to two ways; the rest goes back to being cache
    dspm_set_ways(0x03);

    for (int way = 0; way < 2; way++) *DSPM_WORD(way, 1) = 0xAA00ULL + way;

    // The cache must still work while half the ways are scratchpad
    for (int i = 0; i < 256; i++) dram_buf[i] = 0x5500ULL + i;
    for (int i = 0; i < 256; i++) CHECK(dram_buf[i] == 0x5500ULL + i);

    for (int way = 0; way < 2; way++) CHECK(*DSPM_WORD(way, 1) == 0xAA00ULL + way);

    // A way that is NOT configured as scratchpad: report what comes back
    // rather than asserting, so the observed behaviour is on record.
    uint64_t inactive = *DSPM_WORD(3, 0);
    printf("inactive way returns 0x%016llx (magic=%d)\n",
           (unsigned long long)inactive, inactive == SPM_INACTIVE_MAGIC);

    dspm_set_ways(DSPM_SAFE_MASK);
    return 0;
}

int main(void) {
    int rc;

    checks = 0;  // crt0 does not zero .bss
    dspm_set_ways(DSPM_SAFE_MASK);

    if ((rc = test_widths())) return rc;
    if ((rc = test_ways())) return rc;
    if ((rc = test_partial())) return rc;

    printf("dcache_spm_widths: %d checks passed\n", checks);
    return 0;
}
