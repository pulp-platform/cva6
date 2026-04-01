// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Christopher Reinwardt <creinwar@iis.ee.ethz.ch>
// Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// Configure the I-cache in SPM mode and try to write and read back data from it.

#include <stdint.h>

#define ICACHE_SPM_ADDR 0x01A00000UL
#define ICACHE_SIZE     16384UL
#define ICACHE_NUM_WAYS 4
#define ICACHE_WAY_SIZE ((ICACHE_SIZE) / (ICACHE_NUM_WAYS))

#define data_t uint64_t

volatile data_t *spm_data = (volatile data_t *) ICACHE_SPM_ADDR;

int main(void) {

    // Disable I-cache
    asm volatile ("csrwi 0x7C0, 0");
    // Configure all ways as scratchpad
    uint32_t way_mask = (1 << ICACHE_NUM_WAYS) - 1;
    asm volatile ("csrw 0x5E1, %0" : : "r"(way_mask));
    // Re-enable the I-cache
    asm volatile ("csrwi 0x7C0, 1");

    for(unsigned int i = 0; i < (ICACHE_WAY_SIZE / sizeof(data_t)) * ICACHE_NUM_WAYS; i += 1) {
        spm_data[i] = i;
    }

    for(unsigned int i = 0; i < (ICACHE_WAY_SIZE / sizeof(data_t)) * ICACHE_NUM_WAYS; i += 1) {
        if (spm_data[i] != i) return i + 1;
    }

    return 0;
}
