// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// Takes the CLINT's timer and software interrupts, and the external interrupt the platform
// control registers raise, each in M-mode

#include <stdint.h>

#include "util.h"

// Word-aligned peripheral bases, for word accesses under -mstrict-align
extern char __base_clint[] __attribute__((aligned(4)));
extern char __base_regs[] __attribute__((aligned(4)));

#define CLINT_MSIP        ((uintptr_t)__base_clint + 0x0000)
#define CLINT_MTIMECMP_LO ((uintptr_t)__base_clint + 0x4000)
#define CLINT_MTIMECMP_HI ((uintptr_t)__base_clint + 0x4004)
#define CLINT_MTIME_LO    ((uintptr_t)__base_clint + 0xBFF8)
#define REGS_IRQ          ((uintptr_t)__base_regs + 0x8)

#define IRQ_M_SOFT  3
#define IRQ_M_TIMER 7
#define IRQ_M_EXT   11
#define MCAUSE_IRQ  (1ULL << 63)

static volatile uint64_t last_cause;

// Records the cause and silences its source; the read-back completes the write before mret
__attribute__((interrupt("machine"), aligned(4))) static void irq_handler(void) {
    uint64_t cause;
    asm volatile("csrr %0, mcause" : "=r"(cause));
    uintptr_t source = 0;
    uint32_t quiet = 0;
    if (cause == (MCAUSE_IRQ | IRQ_M_TIMER)) {
        source = CLINT_MTIMECMP_HI;
        quiet = 0xffffffff;
    } else if (cause == (MCAUSE_IRQ | IRQ_M_SOFT)) {
        source = CLINT_MSIP;
    } else if (cause == (MCAUSE_IRQ | IRQ_M_EXT)) {
        source = REGS_IRQ;
    }
    if (source) {
        write32(source, quiet);
        (void)read32(source);
    }
    last_cause = cause;
}

static inline uint64_t read_mip(void) {
    uint64_t mip;
    asm volatile("csrr %0, mip" : "=r"(mip));
    return mip;
}

// Sleeps until the interrupt is pending, then takes it. wfi wakes on a pending enabled interrupt
// with global interrupts off, so the interrupt cannot be taken between the check and the wfi.
static void take(int irq) {
    while (!(read_mip() & (1ULL << irq))) asm volatile("wfi");
    set_global_mie(1);
    while (last_cause != (MCAUSE_IRQ | irq));
    set_global_mie(0);
}

int main(void) {
    // mtime counts the RTC ticks
    uint32_t start = read32(CLINT_MTIME_LO);
    while (read32(CLINT_MTIME_LO) == start);

    // The compare value resets to zero: the timer stays pending until it is set out of reach
    write32(CLINT_MTIMECMP_HI, 0xffffffff);
    set_mtvec((uintptr_t)irq_handler);
    asm volatile("csrs mie, %0" ::"r"((1 << IRQ_M_SOFT) | (1 << IRQ_M_TIMER) | (1 << IRQ_M_EXT)));

    // Timer interrupt, two ticks from now and not before
    uint32_t deadline = read32(CLINT_MTIME_LO) + 2;
    write32(CLINT_MTIMECMP_LO, deadline);
    write32(CLINT_MTIMECMP_HI, 0);
    take(IRQ_M_TIMER);
    if (read32(CLINT_MTIME_LO) < deadline) return 1;

    // Software interrupt
    write32(CLINT_MSIP, 1);
    take(IRQ_M_SOFT);

    // External interrupt, raised through the platform control registers
    write32(REGS_IRQ, 1);
    take(IRQ_M_EXT);

    return 0;
}
