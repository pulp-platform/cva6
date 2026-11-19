// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>

#pragma once

#include <stdint.h>

static inline uint32_t read32(uintptr_t address) {
    return *(volatile uint32_t *)address;
}

static inline void write32(uintptr_t address, uint32_t value) {
    *(volatile uint32_t *)address = value;
}

static inline void set_mtvec(uintptr_t address) {
    uint64_t value = address & ~0x3; // Ensure address is aligned to 4 bytes
    asm volatile ("csrw mtvec, %0" :: "r"(value));
}

// Enables or disables M-mode global interrupts
static inline void set_global_mie(int enable) {
    if (enable)
        asm volatile("csrsi mstatus, 8" ::: "memory");
    else
        asm volatile("csrci mstatus, 8" ::: "memory");
}
