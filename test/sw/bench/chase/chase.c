// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Pointer chase: the load-to-use latency of a working set of CHASE_BYTES.
//
// The working set is an array of 16-byte nodes, each pointing to the next
// along one cycle through all of them, in pseudo-random order: every load
// depends on the previous one, and no stride gives the next address away. The
// measured region follows CHASE_LOADS pointers, after a warm-up lap of at most
// as many, so its cycles per load are the latency of wherever the working set
// fits. A node is the smallest data cache line of any CVA6 configuration, so
// the working set occupies CHASE_BYTES of cache whatever the line size.
//
// The nodes live past the program image, from __base_heap. The visit order is
// a bijective hash of the visit number; the nodes are linked in address order,
// through its inverse, so building the cycle takes no random accesses.

#include <stdint.h>

#include "bench.h"
#include "printf.h"

#ifndef CHASE_BYTES
#define CHASE_BYTES (64 * 1024)
#endif
#ifndef CHASE_LOADS
#define CHASE_LOADS 16384
#endif

struct node {
    struct node *next;
    uint64_t pad;
};

_Static_assert(sizeof(struct node) == 16, "a node is 16 bytes");

#define NODES ((uint64_t)CHASE_BYTES / 16)
#define UNROLL 16

#if (CHASE_BYTES / 16) & (CHASE_BYTES / 16 - 1)
#error "CHASE_BYTES / 16 must be a power of two"
#endif
#if CHASE_LOADS % UNROLL
#error "CHASE_LOADS must be a multiple of 16"
#endif

extern char __base_heap[];
extern char __stack_pointer$[];

#define STACK_RESERVE (64 * 1024)
#define BITS __builtin_ctzl(NODES)
#define MASK (NODES - 1)
#define MUL 0x9e3779b97f4a7c15ull
#define ROUNDS 3

// The visit order: a bijection on [0, NODES), rounds of key mixing, an odd
// multiplier and a right xorshift by more than half the bits, each invertible.
static uint64_t key_of(int round) {
    uint64_t key = 0x2545f4914f6cdd1dull;
    for (int r = 0; r < round; r++)
        key = key * 0xbf58476d1ce4e5b9ull + 0x94d049bb133111ebull;
    return key;
}

static uint64_t order(uint64_t v) {
    for (int r = 0; r < ROUNDS; r++) {
        v = (v ^ key_of(r)) & MASK;
        v = (v * MUL) & MASK;
        v ^= v >> (BITS / 2 + 1);
    }
    return v;
}

static uint64_t order_inverse(uint64_t v) {
    // MUL's inverse modulo 2^64, by Newton's iteration; modulo 2^BITS as well.
    uint64_t inv = MUL;
    for (int i = 0; i < 5; i++)
        inv *= 2 - MUL * inv;
    for (int r = ROUNDS - 1; r >= 0; r--) {
        v ^= v >> (BITS / 2 + 1); // its own inverse: shifted twice, nothing is left
        v = (v * inv) & MASK;
        v = (v ^ key_of(r)) & MASK;
    }
    return v;
}

static struct node *chase(struct node *p, uint64_t loads) {
    for (uint64_t i = 0; i < loads; i += UNROLL) {
#pragma GCC unroll 16
        for (int u = 0; u < UNROLL; u++)
            p = p->next;
    }
    return p;
}

int main(void) {
    struct node *nodes = (struct node *)(((uintptr_t)__base_heap + 63) & ~(uintptr_t)63);
    if ((uintptr_t)(nodes + NODES) > (uintptr_t)__stack_pointer$ - STACK_RESERVE) {
        printf("[CHASE] %lu bytes do not fit in memory\n", (unsigned long)CHASE_BYTES);
        return 1;
    }

    // Node order(k) points to node order(k + 1): node j, visited k-th, to the
    // node visited next.
    for (uint64_t j = 0; j < NODES; j++)
        nodes[j].next = &nodes[order((order_inverse(j) + 1) & MASK)];

    const uint64_t warmup = NODES < CHASE_LOADS ? NODES : CHASE_LOADS;
    struct node *p = chase(&nodes[order(0)], warmup - warmup % UNROLL);

    bench_roi_start();
    uint64_t t0 = bench_read_mcycle();
    p = chase(p, CHASE_LOADS);
    uint64_t t1 = bench_read_mcycle();
    bench_roi_stop();

    uint64_t cycles = t1 - t0;
    printf("[CHASE] bytes=%lu loads=%lu cycles=%lu cycles_per_100_loads=%lu\n",
           (unsigned long)CHASE_BYTES, (unsigned long)CHASE_LOADS, (unsigned long)cycles,
           (unsigned long)(100 * cycles / CHASE_LOADS));
    bench_report();

    // Where the chase must have ended: after the warm-up and the region's loads
    struct node *expected = &nodes[order((warmup - warmup % UNROLL + CHASE_LOADS) & MASK)];
    if (p != expected) {
        printf("[CHASE] ended at node %ld, expected %ld\n", (long)(p - nodes),
               (long)(expected - nodes));
        return 1;
    }
    printf("[CHASE] chain verified\n");
    return 0;
}
