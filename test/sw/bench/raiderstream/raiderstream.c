// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// RaiderSTREAM on the standalone testbench: the four kernels of one access
// pattern (copy, scale, add, triad, in STREAM's order) on arrays several times
// the data cache, all four in the measured region. The kernels are upstream's
// OpenMP ones (Impl/RS_OMP/RS_OMP_IMPL.c) compiled without OpenMP; this driver
// replaces upstream's C++ harness, which needs an OS.
//
// RS_PATTERN selects the pattern, RS_ARRAY_SIZE the elements per array, a power
// of two. The arrays live past the program image, from __base_heap, so neither
// the image nor the startup code's .bss clearing pays for them.
//
// The index arrays are pseudo-random permutations, like upstream's
// Fisher-Yates shuffles, but each element is computed from its position by a
// bijective hash: no random accesses before the region starts.
//
// Upstream's validation is disabled; this driver checks the results exactly.
// The arrays start as STREAM's, except a[j] = j + 1 instead of 1, so that every
// element is distinct and the values stay integers: the sums over each array,
// and the relations each kernel establishes, sampled every RS_CHECK_STRIDE
// elements, must hold exactly.

#include <stdint.h>
#include <sys/types.h>

#include "bench.h"
#include "printf.h"

#define RS_SEQ     0
#define RS_GATHER  1
#define RS_SCATTER 2
#define RS_SG      3
#define RS_CENTRAL 4

#ifndef RS_PATTERN
#error "define RS_PATTERN as one of RS_SEQ, RS_GATHER, RS_SCATTER, RS_SG, RS_CENTRAL"
#endif
#ifndef RS_ARRAY_SIZE
#define RS_ARRAY_SIZE (1 << 15)
#endif
#ifndef RS_CHECK_STRIDE
#define RS_CHECK_STRIDE 61 // odd, so the samples spread over cache sets
#endif

#if RS_ARRAY_SIZE & (RS_ARRAY_SIZE - 1)
#error "RS_ARRAY_SIZE must be a power of two"
#endif

#define N ((ssize_t)RS_ARRAY_SIZE)

typedef STREAM_TYPE T;

// Upstream's kernels (Impl/RS_OMP/RS_OMP_IMPL.c)
void seqCopy(T *a, T *b, T *c, ssize_t n);
void seqScale(T *a, T *b, T *c, ssize_t n, T scalar);
void seqAdd(T *a, T *b, T *c, ssize_t n);
void seqTriad(T *a, T *b, T *c, ssize_t n, T scalar);
void gatherCopy(T *a, T *b, T *c, ssize_t *idx1, ssize_t n);
void gatherScale(T *a, T *b, T *c, ssize_t *idx1, ssize_t n, T scalar);
void gatherAdd(T *a, T *b, T *c, ssize_t *idx1, ssize_t *idx2, ssize_t n);
void gatherTriad(T *a, T *b, T *c, ssize_t *idx1, ssize_t *idx2, ssize_t n, T scalar);
void scatterCopy(T *a, T *b, T *c, ssize_t *idx1, ssize_t n);
void scatterScale(T *a, T *b, T *c, ssize_t *idx1, ssize_t n, T scalar);
void scatterAdd(T *a, T *b, T *c, ssize_t *idx1, ssize_t n);
void scatterTriad(T *a, T *b, T *c, ssize_t *idx1, ssize_t n, T scalar);
void sgCopy(T *a, T *b, T *c, ssize_t *idx1, ssize_t *idx2, ssize_t n);
void sgScale(T *a, T *b, T *c, ssize_t *idx1, ssize_t *idx2, ssize_t n, T scalar);
void sgAdd(T *a, T *b, T *c, ssize_t *idx1, ssize_t *idx2, ssize_t *idx3, ssize_t n);
void sgTriad(T *a, T *b, T *c, ssize_t *idx1, ssize_t *idx2, ssize_t *idx3, ssize_t n,
             T scalar);
void centralCopy(T *a, T *b, T *c, ssize_t n);
void centralScale(T *a, T *b, T *c, ssize_t n, T scalar);
void centralAdd(T *a, T *b, T *c, ssize_t n);
void centralTriad(T *a, T *b, T *c, ssize_t n, T scalar);

extern char __base_heap[];
extern char __stack_pointer$[];

#define STACK_RESERVE (64 * 1024)

static const T scalar = 3.0;

static T *a, *b, *c;
static ssize_t *idx1, *idx2, *idx3;

// A bijection on [0, n) for n a power of two: rounds of key mixing, an odd
// multiplier and a right xorshift, each invertible modulo n.
static ssize_t permute(ssize_t x, uint64_t key) {
    const unsigned bits = __builtin_ctzl(N);
    const uint64_t mask = (uint64_t)N - 1;
    uint64_t v = (uint64_t)x;
    for (int round = 0; round < 3; round++) {
        v = (v ^ key) & mask;
        v = (v * 0x9e3779b97f4a7c15ull) & mask;
        v ^= v >> (bits / 2 + 1);
        key = key * 0xbf58476d1ce4e5b9ull + 0x94d049bb133111ebull;
    }
    return (ssize_t)v;
}

static __attribute__((unused)) void init_idx(ssize_t *idx, uint64_t key) {
    for (ssize_t j = 0; j < N; j++)
        idx[j] = permute(j, key);
}

static int errors;

static void expect(const char *what, ssize_t j, T got, T want) {
    if (got != want) {
        if (errors < 10)
            printf("[RS] check failed: %s at %ld: %ld, expected %ld\n", what, (long)j,
                   (long)got, (long)want);
        errors++;
    }
}

static T sum(const T *x) {
    T s = 0.0;
    for (ssize_t j = 0; j < N; j++)
        s += x[j];
    return s;
}

static void check(void) {
    const T s = (T)N * (T)(N + 1) / 2; // the sum of a's initial values
#if RS_PATTERN == RS_CENTRAL
    expect("a", 0, a[0], 15.0);
    expect("b", 0, b[0], 3.0);
    expect("c", 0, c[0], 4.0);
    expect("sum(a)", -1, sum(a), s - 1.0 + 15.0);
    expect("sum(b)", -1, sum(b), 2.0 * (T)(N - 1) + 3.0);
    expect("sum(c)", -1, sum(c), 4.0);
    for (ssize_t j = 1; j < N; j += RS_CHECK_STRIDE) {
        expect("a", j, a[j], (T)(j + 1));
        expect("b", j, b[j], 2.0);
        expect("c", j, c[j], 0.0);
    }
#else
    // Every element is written by each kernel exactly once, whatever the
    // pattern: the sums are those of the sequential kernels.
    expect("sum(a)", -1, sum(a), 15.0 * s);
    expect("sum(b)", -1, sum(b), 3.0 * s);
    expect("sum(c)", -1, sum(c), 4.0 * s);
    for (ssize_t j = 0; j < N; j += RS_CHECK_STRIDE) {
        __attribute__((unused)) const T a0 = (T)(j + 1);
#if RS_PATTERN == RS_SEQ
        expect("a", j, a[j], 15.0 * a0);
        expect("b", j, b[j], 3.0 * a0);
        expect("c", j, c[j], 4.0 * a0);
#elif RS_PATTERN == RS_GATHER
        // copy: c'[j] = a0[i1[j]], scale: b[j] = 3 c'[i1[j]],
        // add: c[j] = a0[i1[j]] + b[i2[j]], triad: a[j] = b[i1[j]] + 3 c[i2[j]]
        expect("b", j, b[j], 3.0 * (T)(idx1[idx1[j]] + 1));
        expect("c", j, c[j], (T)(idx1[j] + 1) + b[idx2[j]]);
        expect("a", j, a[j], b[idx1[j]] + 3.0 * c[idx2[j]]);
#elif RS_PATTERN == RS_SCATTER
        // copy: c'[i1[j]] = a0[j], scale: b[i1[j]] = 3 c'[j],
        // add: c[i1[j]] = a0[j] + b[j], triad: a[i1[j]] = b[j] + 3 c[j]
        expect("b", j, b[idx1[idx1[j]]], 3.0 * a0);
        expect("c", j, c[idx1[j]], a0 + b[j]);
        expect("a", j, a[idx1[j]], b[j] + 3.0 * c[j]);
#elif RS_PATTERN == RS_SG
        // copy: c'[i1[j]] = a0[i2[j]], scale: b[i2[j]] = 3 c'[i1[j]],
        // add: c[i1[j]] = a0[i2[j]] + b[i3[j]], triad: a[i2[j]] = b[i3[j]] + 3 c[i1[j]]
        expect("b", j, b[j], 3.0 * a0);
        expect("c", j, c[idx1[j]], (T)(idx2[j] + 1) + b[idx3[j]]);
        expect("a", j, a[idx2[j]], b[idx3[j]] + 3.0 * c[idx1[j]]);
#endif
    }
#endif
}

// Bytes each kernel moves per element, counted as upstream does (RSOpts.h):
// the values read and written, and the indices read.
#define V sizeof(T)
#define I sizeof(ssize_t)
#if RS_PATTERN == RS_SEQ || RS_PATTERN == RS_CENTRAL
static const unsigned bytes[4] = {2 * V, 2 * V, 3 * V, 3 * V};
#elif RS_PATTERN == RS_GATHER
static const unsigned bytes[4] = {2 * V + I, 2 * V + I, 3 * V + 2 * I, 3 * V + 2 * I};
#elif RS_PATTERN == RS_SCATTER
static const unsigned bytes[4] = {2 * V + I, 2 * V + I, 3 * V + I, 3 * V + I};
#elif RS_PATTERN == RS_SG
static const unsigned bytes[4] = {2 * V + 2 * I, 2 * V + 2 * I, 3 * V + 3 * I, 3 * V + 3 * I};
#endif

static const char *const pattern_name[] = {"seq", "gather", "scatter", "sg", "central"};
static const char *const kernel_name[4] = {"copy", "scale", "add", "triad"};

int main(void) {
    char *heap = (char *)(((uintptr_t)__base_heap + 63) & ~(uintptr_t)63);
    const size_t array = (size_t)N * sizeof(T);
    if ((uintptr_t)heap + 3 * array + 3 * (size_t)N * sizeof(ssize_t) >
        (uintptr_t)__stack_pointer$ - STACK_RESERVE) {
        printf("[RS] %ld elements per array do not fit in memory\n", (long)N);
        return 1;
    }
    a = (T *)heap;
    b = (T *)(heap + array);
    c = (T *)(heap + 2 * array);
    idx1 = (ssize_t *)(heap + 3 * array);
    idx2 = idx1 + N;
    idx3 = idx2 + N;

#if RS_PATTERN == RS_GATHER || RS_PATTERN == RS_SCATTER || RS_PATTERN == RS_SG
    init_idx(idx1, 1);
#endif
#if RS_PATTERN == RS_GATHER || RS_PATTERN == RS_SG
    init_idx(idx2, 2);
#endif
#if RS_PATTERN == RS_SG
    init_idx(idx3, 3);
#endif
    for (ssize_t j = 0; j < N; j++) {
        a[j] = (T)(j + 1);
        b[j] = 2.0;
        c[j] = 0.0;
    }

    uint64_t t[5];
    bench_roi_start();
    t[0] = bench_read_mcycle();
#if RS_PATTERN == RS_SEQ
    seqCopy(a, b, c, N);
    t[1] = bench_read_mcycle();
    seqScale(a, b, c, N, scalar);
    t[2] = bench_read_mcycle();
    seqAdd(a, b, c, N);
    t[3] = bench_read_mcycle();
    seqTriad(a, b, c, N, scalar);
#elif RS_PATTERN == RS_GATHER
    gatherCopy(a, b, c, idx1, N);
    t[1] = bench_read_mcycle();
    gatherScale(a, b, c, idx1, N, scalar);
    t[2] = bench_read_mcycle();
    gatherAdd(a, b, c, idx1, idx2, N);
    t[3] = bench_read_mcycle();
    gatherTriad(a, b, c, idx1, idx2, N, scalar);
#elif RS_PATTERN == RS_SCATTER
    scatterCopy(a, b, c, idx1, N);
    t[1] = bench_read_mcycle();
    scatterScale(a, b, c, idx1, N, scalar);
    t[2] = bench_read_mcycle();
    scatterAdd(a, b, c, idx1, N);
    t[3] = bench_read_mcycle();
    scatterTriad(a, b, c, idx1, N, scalar);
#elif RS_PATTERN == RS_SG
    sgCopy(a, b, c, idx1, idx2, N);
    t[1] = bench_read_mcycle();
    sgScale(a, b, c, idx1, idx2, N, scalar);
    t[2] = bench_read_mcycle();
    sgAdd(a, b, c, idx1, idx2, idx3, N);
    t[3] = bench_read_mcycle();
    sgTriad(a, b, c, idx1, idx2, idx3, N, scalar);
#elif RS_PATTERN == RS_CENTRAL
    centralCopy(a, b, c, N);
    t[1] = bench_read_mcycle();
    centralScale(a, b, c, N, scalar);
    t[2] = bench_read_mcycle();
    centralAdd(a, b, c, N);
    t[3] = bench_read_mcycle();
    centralTriad(a, b, c, N, scalar);
#endif
    t[4] = bench_read_mcycle();
    bench_roi_stop();

    check();

    // Per kernel: cycles, and bytes moved per 100 cycles
    for (int k = 0; k < 4; k++) {
        uint64_t cycles = t[k + 1] - t[k];
        uint64_t moved = (uint64_t)bytes[k] * (uint64_t)N;
        printf("[RS] %s %s elements=%ld bytes=%lu cycles=%lu bytes_per_100_cycles=%lu\n",
               pattern_name[RS_PATTERN], kernel_name[k], (long)N, (unsigned long)moved,
               (unsigned long)cycles, (unsigned long)(cycles ? 100 * moved / cycles : 0));
    }
    bench_report();
    if (errors) {
        printf("[RS] %d checks failed\n", errors);
        return 1;
    }
    printf("[RS] results verified\n");
    return 0;
}
