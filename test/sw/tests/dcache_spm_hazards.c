// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Data-SPM hazard cases. These are the sequences a speculative SPM read (or any
// change to the request-steering FSM) is most likely to break: store-to-load
// forwarding, page-offset aliasing between SPM and cached memory, mixed
// load/store streams, and accesses around a flush.
//
// Returns 0 on success, otherwise the number of the failing check.

#include <stdint.h>
#include "spm.h"
#include "printf.h"

static int checks = 0;

#define CHECK(cond)                                                \
    do {                                                           \
        checks++;                                                  \
        if (!(cond)) {                                             \
            printf("FAIL check %d (line %d)\n", checks, __LINE__); \
            return checks;                                         \
        }                                                          \
    } while (0)

// Same, but reports what was actually read
#define CHECK_EQ(actual, expected)                                       \
    do {                                                                 \
        checks++;                                                        \
        uint64_t _a = (uint64_t)(actual);                                \
        uint64_t _e = (uint64_t)(expected);                              \
        if (_a != _e) {                                                  \
            printf("FAIL check %d (line %d): got 0x%016llx want 0x%016llx\n", \
                   checks, __LINE__, (unsigned long long)_a,             \
                   (unsigned long long)_e);                              \
            return checks;                                               \
        }                                                                \
    } while (0)

// --- 1. store then load, same address ------------------------------------
// The load must see the just-written value, whatever the store buffer and the
// SPM controller do in between.
static int test_store_to_load(void) {
    volatile uint64_t *p = DSPM_WORD(0, 4);

    for (int i = 0; i < 16; i++) {
        *p = 0xC0FFEE00ULL + i;
        CHECK(*p == 0xC0FFEE00ULL + i);
    }

    // Narrow store followed by a wide load of the same word
    volatile uint8_t *b = (volatile uint8_t *)p;
    *p = 0;
    b[3] = 0x77;
    CHECK(*p == 0x77000000ULL);
    return 0;
}

// --- 2. page-offset aliasing between SPM and cached memory ---------------
// The store buffer compares page offsets (address bits 11:3) only, so a cached
// store and an SPM load at the same offset look aliased even though they are
// different memories. Neither must see the other's data.
static uint64_t dram_buf[512] __attribute__((aligned(4096)));

static int test_alias(void) {
    volatile uint64_t *spm = DSPM_WORD(1, 3);
    volatile uint64_t *dram = &dram_buf[((uintptr_t)spm & 0xFFF) / sizeof(uint64_t)];

    // Same page offset, different memories
    CHECK((((uintptr_t)spm ^ (uintptr_t)dram) & 0xFFF) == 0);

    *spm = 0x5111511151115111ULL;
    *dram = 0xDDDDDDDDDDDDDDDDULL;
    CHECK(*spm == 0x5111511151115111ULL);
    CHECK(*dram == 0xDDDDDDDDDDDDDDDDULL);

    // Reverse order: cached store first, then the SPM one
    *dram = 0x2222222222222222ULL;
    *spm = 0x3333333333333333ULL;
    CHECK(*dram == 0x2222222222222222ULL);
    CHECK(*spm == 0x3333333333333333ULL);

    // Store to one, immediately load the other. The SPM store must not be
    // visible in the cached location and vice versa.
    printf("alias: spm=%p dram=%p\n", (void *)spm, (void *)dram);
    uint64_t dram_expect = 0x2222222222222222ULL;
    for (int i = 0; i < 8; i++) {
        *spm = 0x4000ULL + i;
        CHECK_EQ(*dram, dram_expect);
        dram_expect = 0x5000ULL + i;
        *dram = dram_expect;
        CHECK_EQ(*spm, 0x4000ULL + i);
    }
    return 0;
}

// --- 3. interleaved loads and stores --------------------------------------
// Alternating accesses keep both the load and the store path of the SPM busy,
// which is where request steering is most likely to mix up the in-flight
// metadata (way index, cacheline offset, request id).
static int test_mixed(void) {
    volatile uint64_t *a = DSPM_WORD(2, 0);
    volatile uint64_t *b = DSPM_WORD(3, 0);
    uint64_t acc = 0;

    for (int i = 0; i < 64; i++) {
        a[i] = 0x1111000ULL + i;
        acc += a[i];
        b[i] = acc;
    }

    for (int i = 0; i < 64; i++) CHECK(a[i] == 0x1111000ULL + i);

    uint64_t expect = 0;
    for (int i = 0; i < 64; i++) {
        expect += 0x1111000ULL + i;
        CHECK(b[i] == expect);
    }
    return 0;
}

// --- 4. SPM accesses around a flush --------------------------------------
// fence.i flushes the caches. SPM contents must survive it, and accesses issued
// on either side of it must complete normally.
static int test_flush(void) {
    volatile uint64_t *p = DSPM_WORD(3, 7);

    *p = 0xF10051F10051F100ULL;
    asm volatile("fence.i" ::: "memory");
    CHECK(*p == 0xF10051F10051F100ULL);

    for (int i = 0; i < 4; i++) {
        p[i] = 0x9000ULL + i;
        asm volatile("fence" ::: "memory");
        CHECK(p[i] == 0x9000ULL + i);
    }

    // Cached memory must still behave after all the mode switching
    for (int i = 0; i < 64; i++) dram_buf[i] = 0x7000ULL + i;
    asm volatile("fence.i" ::: "memory");
    for (int i = 0; i < 64; i++) CHECK(dram_buf[i] == 0x7000ULL + i);
    return 0;
}

int main(void) {
    int rc;

    checks = 0;  // crt0 does not zero .bss
    dspm_set_ways(DSPM_SAFE_MASK);

    if ((rc = test_store_to_load())) return rc;
    if ((rc = test_alias())) return rc;
    if ((rc = test_mixed())) return rc;
    if ((rc = test_flush())) return rc;

    printf("dcache_spm_hazards: %d checks passed\n", checks);
    return 0;
}
