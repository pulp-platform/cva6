// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// SPM accesses killed in flight.
//
// A load is killed when the pipeline is flushed while it is still outstanding:
// load_unit goes to WAIT_FLUSH and raises kill_req, and the request steering
// FSM in std_nbdcache answers it with a synthesised data_rvalid. In this
// configuration (MmuPresent, no translation in use) the reachable trigger is a
// mispredicted branch resolving while the load is in flight — the DTLB-miss and
// exception paths need virtual memory.
//
// Each case puts an unpredictable branch immediately after an SPM load, so the
// mispredict resolves in the cycle the load is still outstanding, then checks
// that the value that eventually arrives is the right one. A lost, duplicated
// or mismatched rvalid shows up as a wrong value or as a hang.
//
// Returns 0 on success, otherwise the number of the failing check.

#include <stdint.h>
#include "spm.h"
#include "printf.h"

#define ITERS 512

static int checks;

#define CHECK_EQ(actual, expected)                                            \
    do {                                                                      \
        checks++;                                                             \
        uint64_t _a = (uint64_t)(actual);                                     \
        uint64_t _e = (uint64_t)(expected);                                   \
        if (_a != _e) {                                                       \
            printf("FAIL check %d (line %d): got 0x%016llx want 0x%016llx\n", \
                   checks, __LINE__, (unsigned long long)_a,                  \
                   (unsigned long long)_e);                                   \
            return checks;                                                    \
        }                                                                     \
    } while (0)

// Pseudo-random bits, so the branch predictor cannot learn the pattern
static uint32_t rnd_state = 0xC0FFEE11;
static inline uint32_t rnd(void) {
    rnd_state ^= rnd_state << 13;
    rnd_state ^= rnd_state >> 17;
    rnd_state ^= rnd_state << 5;
    return rnd_state;
}

// SPM load followed immediately by an unpredictable branch: about half the
// iterations flush the pipeline while the load is still in flight.
static uint64_t load_then_branch(volatile uint64_t *p, uint64_t taken) {
    uint64_t v;
    asm volatile(
        "  ld   %0, 0(%1)\n"
        "  beqz %2, 1f\n"
        "  nop\n"
        "  nop\n"
        "1:\n"
        : "=&r"(v)
        : "r"(p), "r"(taken)
        : "memory");
    return v;
}

// Same, with the load's own result deciding the branch: the branch cannot
// resolve until the load has returned, which exercises the opposite ordering.
static uint64_t load_then_dependent_branch(volatile uint64_t *p) {
    uint64_t v;
    asm volatile(
        "  ld   %0, 0(%1)\n"
        "  andi t0, %0, 1\n"
        "  beqz t0, 1f\n"
        "  nop\n"
        "1:\n"
        : "=&r"(v)
        : "r"(p)
        : "t0", "memory");
    return v;
}

// Trap handler: skip the faulting instruction. Exceptions flush the pipeline,
// so anything issued behind the faulting instruction is killed in flight.
extern void spm_kill_trap(void);
// It must not clobber any register: the interrupted code is inline asm whose
// operands the compiler may have placed anywhere, so t6 is saved and restored.
asm(".align 8\n"
    "spm_kill_trap:\n"
    "  addi sp, sp, -8\n"
    "  sd   t6, 0(sp)\n"
    "  csrr t6, mepc\n"
    "  addi t6, t6, 4\n"
    "  csrw mepc, t6\n"
    "  ld   t6, 0(sp)\n"
    "  addi sp, sp, 8\n"
    "  mret\n");

// A cache miss with an unpredictable branch right after it: the flush arrives
// while the refill is still outstanding, which is the case cache_ctrl covers
// with its sticky `killed` flag.
#define MISS_WORDS (64 * 1024 / 8)
static uint64_t miss_buf[MISS_WORDS];

int main(void) {
    checks = 0;  // crt0 does not zero .bss

    dspm_set_ways(DSPM_SAFE_MASK);

    volatile uint64_t *a = DSPM_WORD(0, 0);
    volatile uint64_t *b = DSPM_WORD(2, 5);

    for (int i = 0; i < 64; i++) a[i] = 0xA5A50000ULL + i;
    *b = 0x1234567812345678ULL;

    // --- 1. unpredictable branch right after an SPM load -----------------
    for (int i = 0; i < ITERS; i++) {
        int idx = i & 63;
        uint64_t v = load_then_branch(&a[idx], rnd() & 1);
        CHECK_EQ(v, 0xA5A50000ULL + idx);
    }

    // --- 2. branch that depends on the loaded value ----------------------
    for (int i = 0; i < ITERS / 4; i++) {
        CHECK_EQ(load_then_dependent_branch(b), 0x1234567812345678ULL);
    }

    // --- 3. flushes from fence.i with SPM traffic around them ------------
    for (int i = 0; i < 64; i++) {
        uint64_t v = a[i & 63];
        asm volatile("fence.i" ::: "memory");
        CHECK_EQ(v, 0xA5A50000ULL + (i & 63));
        a[i & 63] = 0xA5A50000ULL + (i & 63);  // keep the contents stable
    }

    // --- 4. mixed SPM and cached loads around unpredictable branches -----
    // Both memories have a request in flight across the same flush.
    static uint64_t dram[64];
    for (int i = 0; i < 64; i++) dram[i] = 0xD0D00000ULL + i;
    for (int i = 0; i < ITERS; i++) {
        int idx = i & 63;
        uint64_t s = load_then_branch(&a[idx], rnd() & 1);
        uint64_t d = load_then_branch(&dram[idx], rnd() & 1);
        CHECK_EQ(s, 0xA5A50000ULL + idx);
        CHECK_EQ(d, 0xD0D00000ULL + idx);
    }

    // --- 5. loads in the shadow of a trap --------------------------------
    // The ecall faults, the loads behind it are already issued and get killed.
    asm volatile("csrw mtvec, %0" ::"r"((uintptr_t)&spm_kill_trap) : "memory");
    for (int i = 0; i < 128; i++) {
        int idx = i & 63;
        uint64_t v0, v1;
        asm volatile(
            "  ecall\n"
            "  ld %0, 0(%2)\n"
            "  ld %1, 0(%3)\n"
            : "=&r"(v0), "=&r"(v1)
            : "r"(&a[idx]), "r"(b)
            : "memory");
        CHECK_EQ(v0, 0xA5A50000ULL + idx);
        CHECK_EQ(v1, 0x1234567812345678ULL);
    }

    // --- 6. cache miss killed in flight ----------------------------------
    // Every iteration misses (the buffer is far larger than the cache left
    // over with 4 ways as scratchpad), so the mispredict resolves while the
    // refill is outstanding.
    for (int i = 0; i < MISS_WORDS; i += 64) miss_buf[i] = 0xBEEF0000ULL + i;
    for (int i = 0; i < 256; i++) {
        int idx = (i * 64) & (MISS_WORDS - 1);
        uint64_t m = load_then_branch(&miss_buf[idx], rnd() & 1);
        CHECK_EQ(m, 0xBEEF0000ULL + idx);
        // and an SPM load right behind the killed miss
        CHECK_EQ(*b, 0x1234567812345678ULL);
    }

    printf("dcache_spm_kill: %d checks passed\n", checks);
    return 0;
}
