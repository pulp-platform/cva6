// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// CLIC interrupt arriving during a fence.t.
//
// A fence.t ends with a micro-reset of the pipeline, which restarts it after
// the fence.t; the CSR file keeps its state. An interrupt arriving while the
// sequence runs must be taken after it, with its handler running, and leave
// the interrupt enable and level as they were once the handler returns.
//
// The loop below executes fence.t back to back, so most cycles are inside a
// fence.t sequence. Run with the testbench interrupt generator enabled
// (+InterruptsEnabled=1, external line 1 = CLIC ID 19, e.g. +IrqPeriod=3001,
// and +IsrAddr=<irq_handler>), which raises NUM_IRQS interrupts. After every
// fence.t, mstatus.MIE and mintstatus.mil must still be 1 and 0.
//
// Return codes: 0 pass, 2 MIE or mil wrong after a fence.t (a1 = mstatus,
// a2 = mintstatus), 3 unexpected exception (a1 = mcause, a2 = mepc).

#define CLIC_BASE              0x10040000
#define CLIC_CLICCFG_REG       (CLIC_BASE + 0x0)
#define CLIC_CLICINT_REG(id)   (CLIC_BASE + 0x1000 + 0x4 * (id))
#define CLICINT_IE             (1 << 8)
#define CLICINT_TRIG_EDGE      (1 << 17)
#define CLICINT_CTL(l)         ((l) << 24)

#define IRQ_ID                 19           // testbench external line 1 (NumClicIntIrqs + 1)
#define NUM_IRQS               50           // testbench NumInterrupts
#define CSR_MINTSTATUS         0x346

  .text
  .global main
main:
  // Keep ra/s0/s1 for the return to crt0; s5 and s10 belong to the handler.
  mv    s1, ra
  li    s10, 0

  // CLIC mode: mtvec.mode = 3; the interrupt is not vectored, so it enters at
  // the mtvec base, as exceptions do
  la    t0, irq_handler
  ori   t0, t0, 0x3
  csrrw s0, mtvec, t0
  li    t0, CLIC_CLICCFG_REG
  li    t1, 0x4 << 1
  sw    t1, 0(t0)
  li    t0, CLIC_CLICINT_REG(IRQ_ID)
  li    t1, CLICINT_IE | CLICINT_TRIG_EDGE | CLICINT_CTL(0xff)
  sw    t1, 0(t0)
  csrwi 0x347, 0                      // mintthresh

  li    s6, NUM_IRQS
  csrsi mstatus, 0x8                  // MIE
1:
  .word 0x0000000b                    // fence.t
  csrr  a1, mstatus
  csrr  a2, CSR_MINTSTATUS
  andi  t1, a1, 0x8
  beqz  t1, 2f
  srli  t1, a2, 24
  bnez  t1, 2f
  blt   s10, s6, 1b
  li    a0, 0
  j     done
2:
  li    a0, 2
done:
  csrci mstatus, 0x8
  csrw  mtvec, s0
  mv    ra, s1
  ret

  .balign 256                         // the mtvec base in CLIC mode
irq_handler:
  csrr  s5, mcause
  bgez  s5, trap_fail                 // exceptions have mcause[XLEN-1] clear
  addi  s10, s10, 1
  mret
trap_fail:
  csrr  a1, mcause
  csrr  a2, mepc
  li    a0, 3
  j     done
