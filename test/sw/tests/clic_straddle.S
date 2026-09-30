// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Return address of a CLIC interrupt taken while a 32-bit instruction that
// straddles a fetch boundary is half fetched.
//
// CLIC interrupts are taken at the commit stage. With an empty pipeline the
// return address comes from the frontend. If the instruction realigner holds
// the lower half of a 32-bit instruction while its upper half is still being
// fetched (an I-cache miss), the next instruction starts at the realigner's
// address, not at the fetch address -- which is 2 bytes into the instruction.
//
// The loop below runs only 32-bit instructions placed at PC = 2 (mod 4), so each
// one straddles a fetch boundary, and fence.i empties the I-cache on every pass
// so the fetches miss. Run with the testbench interrupt generator enabled
// (+InterruptsEnabled=1, external line 1 = CLIC ID 19, e.g. +IrqPeriod=3001;
// with the interrupt logger that requires it, also +IsrAddr=<irq_handler>),
// which raises NUM_IRQS interrupts. The handler checks that mepc is an
// instruction boundary inside the loop (2 mod 4) and stops the test at the
// first one that is not.
//
// Return codes: 0 pass, 2 mepc not an instruction boundary (a1 = mepc),
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
  // Keep ra/s0/s1 for the return to crt0; t4-t6 and s10 belong to the handler.
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

  .balign 64
  c.nop                               // everything below sits at PC = 2 (mod 4)
  .option push
  .option norvc
loop_start:
  csrsi mstatus, 0x8                  // MIE
1:
  .rept 1024
  addi  t0, t0, 1
  .endr
  fence.i                             // next pass misses on every line
  li    t1, NUM_IRQS
  blt   s10, t1, 1b
  csrci mstatus, 0x8
loop_end:
  .option pop

  csrw  mtvec, s0
  li    a0, 0
  mv    ra, s1
  ret

  // Vectored handler for IRQ_ID (SHV: entered from the mtvt entry)
  .balign 64
irq_handler:
  csrr  t6, mepc
  andi  t5, t6, 3
  li    t4, 2
  bne   t5, t4, bad_mepc
  la    t5, loop_start
  bltu  t6, t5, bad_mepc
  la    t5, loop_end
  bgeu  t6, t5, bad_mepc
  addi  s10, s10, 1
  mret

bad_mepc:
  csrci mstatus, 0x8
  csrw  mtvec, s0
  mv    a1, t6
  li    a0, 2
  mv    ra, s1
  ret

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
