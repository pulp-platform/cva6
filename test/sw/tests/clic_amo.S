// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// CLIC interrupt taken while an AMO is in flight.
//
// CLIC interrupts are taken at the commit stage. An AMO at the head of the
// scoreboard sends its request to the data cache and waits there, for tens of
// cycles, for the response. An interrupt taken in that window must not abandon
// the AMO: its memory update is already under way, so trapping on it would
// replay it after mret (applying it twice), and the store unit's AMO buffer,
// flushed by the trap, would later be popped by the stale response.
//
// The loop below runs back-to-back amoadd.d on one DRAM counter, counting its
// iterations in a register. Run with the testbench interrupt generator enabled
// (+InterruptsEnabled=1, external line 1 = CLIC ID 19, e.g. +IrqPeriod=3001;
// with the interrupt logger that requires it, also +IsrAddr=<irq_handler>),
// which raises NUM_IRQS interrupts. At the end the counter must equal the
// iteration count: each AMO applied exactly once.
//
// Return codes: 0 pass, 2 counter != iterations (a1 = counter, a2 = iterations),
// 3 unexpected exception (a1 = mcause, a2 = mepc).

#define CLIC_BASE              0x10040000
#define CLIC_CLICCFG_REG       (CLIC_BASE + 0x0)
#define CLIC_CLICINT_REG(id)   (CLIC_BASE + 0x1000 + 0x4 * (id))
#define CLICINT_IE             (1 << 8)
#define CLICINT_SHV            (1 << 16)
#define CLICINT_TRIG_EDGE      (1 << 17)
#define CLICINT_CTL(l)         ((l) << 24)

#define IRQ_ID                 19   // testbench external line 1 (NumClicIntIrqs + 1)
#define NUM_IRQS               50   // testbench NumInterrupts

  .text
  .global main
main:
  // Keep ra/s0/s1 for the return to crt0; s10 belongs to the handler.
  mv    s1, ra
  li    s10, 0

  // CLIC mode: mtvec.mode = 3, exceptions go to the mtvec base
  la    t0, trap_fail
  ori   t0, t0, 0x3
  csrrw s0, mtvec, t0
  la    t0, vector_table
  csrw  0x307, t0                     // mtvt

  // 4 level bits; the interrupt at the top level, threshold 0
  li    t0, CLIC_CLICCFG_REG
  li    t1, 0x4 << 1
  sw    t1, 0(t0)
  li    t0, CLIC_CLICINT_REG(IRQ_ID)
  li    t1, CLICINT_IE | CLICINT_SHV | CLICINT_TRIG_EDGE | CLICINT_CTL(0xff)
  sw    t1, 0(t0)
  csrwi 0x347, 0                      // mintthresh

  la    a3, counter
  sd    zero, 0(a3)
  li    t2, 1
  li    s2, 0                         // iterations
  li    t1, NUM_IRQS
  csrsi mstatus, 0x8                  // MIE
1:
  amoadd.d zero, t2, (a3)
  addi  s2, s2, 1
  blt   s10, t1, 1b
  csrci mstatus, 0x8

  ld    a1, 0(a3)
  mv    a2, s2
  csrw  mtvec, s0
  li    a0, 2
  bne   a1, a2, 2f
  li    a0, 0
2:
  mv    ra, s1
  ret

  // Vectored handler for IRQ_ID (SHV: entered from the mtvt entry)
  .balign 64
irq_handler:
  addi  s10, s10, 1
  mret

  .balign 64
trap_fail:
  csrr  a1, mcause
  csrr  a2, mepc
  csrw  mtvec, s0
  li    a0, 3
  mv    ra, s1
  ret

  .data
  .balign 512
vector_table:
  .rept IRQ_ID
  .dword trap_fail
  .endr
  .dword irq_handler

  .balign 64
counter:
  .dword 0
