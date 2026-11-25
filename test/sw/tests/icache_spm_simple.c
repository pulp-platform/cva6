// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Christopher Reinwardt <creinwar@iis.ee.ethz.ch>
// Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// Configure the I-cache ways as SPM, half of them at a time, and write and read
// back data in them. The other half caches the test's code.

#include <stdint.h>
#include "spm.h"

#define WORDS_PER_WAY (ISPM_WAY_SIZE / sizeof(uint64_t))

volatile uint64_t *spm_data = (volatile uint64_t *) ISPM_BASE;

// Test ways first .. first + count - 1; returns 0, or the failing word index + 1
static int test_ways(unsigned int first, unsigned int count) {
    ispm_set_ways(((1u << count) - 1) << first);

    for (unsigned int i = first * WORDS_PER_WAY; i < (first + count) * WORDS_PER_WAY; i += 1) {
        spm_data[i] = i;
    }

    for (unsigned int i = first * WORDS_PER_WAY; i < (first + count) * WORDS_PER_WAY; i += 1) {
        if (spm_data[i] != i) return i + 1;
    }

    return 0;
}

int main(void) {
    int ret = test_ways(0, ICACHE_WAYS / 2);
    if (ret == 0) ret = test_ways(ICACHE_WAYS / 2, ICACHE_WAYS / 2);
    ispm_set_ways(0);
    return ret;
}
