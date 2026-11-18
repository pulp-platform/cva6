// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// An M-mode interrupt arriving while an S-mode hardware-vectored interrupt
// fetches its handler address from the vector table.
//
// In S-mode, M-mode interrupts are always enabled, so one can arrive while the
// hart fetches the vector table entry of an S-mode interrupt it has just taken.
// The M-mode handler must then run first and return to the S-mode handler,
// which then runs in S-mode.
//
// S-mode code raises the S-mode interrupt repeatedly, each time after emptying
// the I-cache so that the table fetch misses and lasts long, while the
// testbench interrupt generator raises the M-mode one (+InterruptsEnabled=1,
// external line 1 = CLIC ID 19, e.g. +IrqPeriod=3001, and +IsrAddr=<irq_handler>),
// NUM_IRQS times, so some land in a table fetch. Every S-mode interrupt must
// run its handler, in S-mode, once, and every M-mode one its handler: a lost
// M-mode interrupt leaves the loop waiting, and the testbench ends the run
// without a result.
//
// Return codes: 0 pass, 2 S-mode handler count wrong (a1 = S count,
// a2 = M count, a3 = iteration), 4 unexpected exception (a1 = mcause,
// a2 = mepc).

#define CLIC_BASE              0x10040000
#define CLIC_CLICCFG_REG       (CLIC_BASE + 0x0)
#define CLIC_CLICINT_REG(id)   (CLIC_BASE + 0x1000 + 0x4 * (id))
#define CLICINT_IP             (1 << 0)
#define CLICINT_IE             (1 << 8)
#define CLICINT_SHV            (1 << 16)
#define CLICINT_TRIG_EDGE      (1 << 17)
#define CLICINT_MODE(m)        ((m) << 22)
#define CLICINT_CTL(l)         ((l) << 24)

#define IRQ_S                  9            // S-mode, vectored
#define IRQ_M                  19           // M-mode, not vectored: testbench external line 1
#define NUM_IRQS               50           // testbench NumInterrupts
#define CFG_S                  (CLICINT_IE | CLICINT_SHV | CLICINT_TRIG_EDGE | CLICINT_MODE(1) | CLICINT_CTL(0xff))
#define CFG_M                  (CLICINT_IE | CLICINT_TRIG_EDGE | CLICINT_MODE(3) | CLICINT_CTL(0xff))
#define CSR_STVT               0x107
#define CSR_SINTTHRESH         0x147
#define CSR_MINTTHRESH         0x347

  .text
  .global main
main:
  // Keep ra in mscratch and mtvec in s0; s10/s11 count the S/M handlers.
  csrw  mscratch, ra
  li    s10, 0
  li    s11, 0
  la    t0, irq_handler
  ori   t0, t0, 0x3
  csrrw s0, mtvec, t0
  li    t0, CLIC_CLICCFG_REG
  li    t1, 0x4 | (0x3 << 4)          // 4 level bits, 2 mode bits
  sw    t1, 0(t0)
  csrw  CSR_MINTTHRESH, zero
  li    t0, CLIC_CLICINT_REG(IRQ_S)
  li    t1, CFG_S
  sw    t1, 0(t0)
  li    t0, CLIC_CLICINT_REG(IRQ_M)
  li    t1, CFG_M
  sw    t1, 0(t0)

  // Enter S-mode with access to all memory
  li    t0, -1
  srli  t0, t0, 2
  csrw  pmpaddr0, t0
  li    t0, 0x0f                      // R, W, X, TOR
  csrw  pmpcfg0, t0
  la    t0, s_entry
  csrw  mepc, t0
  li    t0, 0x1800
  csrc  mstatus, t0
  li    t0, 0x0800                    // MPP = S
  csrs  mstatus, t0
  mret

s_entry:
  la    t0, s_trap_fail
  ori   t0, t0, 0x3
  csrw  stvec, t0
  la    t0, s_vector_table
  csrw  CSR_STVT, t0
  csrw  CSR_SINTTHRESH, zero
  li    s3, CLIC_CLICINT_REG(IRQ_S)
  li    s4, CFG_S | CLICINT_IP
  li    s2, 0                         // iteration
  li    s6, NUM_IRQS
  csrsi sstatus, 0x2                  // SIE
1:
  fence.i                             // the table fetch misses in the I-cache
  sw    s4, 0(s3)                     // raise the S-mode interrupt
  addi  s2, s2, 1
  li    t0, 1000                      // wait for its handler
2:
  bge   s10, s2, 3f
  addi  t0, t0, -1
  bnez  t0, 2b
3:
  mv    a1, s10
  mv    a2, s11
  mv    a3, s2
  li    a0, 2
  bne   s10, s2, 4f
  // A delay that varies with the iteration, so that the iterations do not lock
  // to the period of the interrupt generator
  andi  t0, s2, 0x3f
5:
  beqz  t0, 6f
  addi  t0, t0, -1
  j     5b
6:
  blt   s11, s6, 1b                   // until every M-mode interrupt was handled
  li    a0, 0
4:
  csrci sstatus, 0x2
  ecall                               // back to M-mode with the result in a0

  // S-mode handler of IRQ_S, entered from its table entry
  .balign 64
s_handler:
  addi  s10, s10, 1
  sret

  .balign 256                         // the stvec base in CLIC mode
s_trap_fail:
  li    a0, 4
  ecall

  // M-mode trap entry: M-mode interrupts, the final ecall from S-mode, and
  // unexpected exceptions
  .balign 256                         // the mtvec base in CLIC mode
irq_handler:
  csrr  t5, mcause
  bgez  t5, 1f
  addi  s11, s11, 1
  mret
1:
  li    t4, 0xfff                     // exception code; CLIC mode adds mpp/mpie/mpil
  and   t6, t5, t4
  li    t4, 9                         // ecall from S-mode: finish
  beq   t6, t4, done
  mv    a1, t5
  csrr  a2, mepc
  li    a0, 4
done:
  csrw  mtvec, s0
  csrr  ra, mscratch
  ret

  .data
  .balign 2048
s_vector_table:
  .rept IRQ_S
  .dword s_trap_fail
  .endr
  .dword s_handler
