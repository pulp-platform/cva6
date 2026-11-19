// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Nested CLIC interrupts with hardware stacking.
//
// Interrupt X (software, lower level) stacks the interrupted registers, saves
// mepc and mcause in its frame, bumps mtfa and re-enables interrupts at once.
// Interrupt B (testbench interrupt generator, higher level) then nests on X's
// handler, possibly while X's frame is still being stacked. Every frame is on
// cache lines that were never used before, so the stacking drain is slow and B
// often arrives during it. Both handlers clobber all caller-saved registers
// and restore them from their frames, so a frame left incomplete or missing
// changes the registers that main finds after X returns.
//
// Run with the interrupt generator (+InterruptsEnabled=1, external line 1 =
// CLIC ID 19, e.g. +IrqPeriod=3001, +IsrAddr=<irq_handler>). The test ends once
// NUM_IRQS generator interrupts have been handled.
//
// Return codes: 0 pass, 2 a caller-saved register changed across X
// (a1 = expected value, which names the register: 0x100 + register number;
// 0x102 for sp), 3 X was not handled, 4 unexpected exception (a1 = mcause,
// a2 = mepc).

#define CLIC_BASE              0x10040000
#define CLIC_CLICCFG_REG       (CLIC_BASE + 0x0)
#define CLIC_CLICINT_REG(id)   (CLIC_BASE + 0x1000 + 0x4 * (id))
#define CLICINT_IP             (1 << 0)
#define CLICINT_IE             (1 << 8)
#define CLICINT_SHV            (1 << 16)
#define CLICINT_TRIG_EDGE      (1 << 17)
#define CLICINT_RSTK_CALLER    (1 << 19)
#define CLICINT_CTL(l)         ((l) << 24)

#define IRQ_X                  9
#define IRQ_B                  19           // testbench external line 1
#define NUM_IRQS               50           // testbench NumInterrupts
#define CFG_X                  (CLICINT_IE | CLICINT_SHV | CLICINT_TRIG_EDGE | CLICINT_RSTK_CALLER | CLICINT_CTL(0x7f))
#define CFG_B                  (CLICINT_IE | CLICINT_SHV | CLICINT_TRIG_EDGE | CLICINT_RSTK_CALLER | CLICINT_CTL(0xff))
#define CSR_MTVT               0x307
#define CSR_MTFA               0x308
#define CSR_MTFA_INC           0x7C6
#define CSR_MINTTHRESH         0x347
#define COLD_BASE              0x81000000   // frames on lines never used before
#define COLD_MASK              0x000fffff   // 1 MiB of frames, then wrap

// Caller-saved frame layout of the stacking (16 registers, ABI order)
#define F_RA   0
#define F_SP   8
#define F_T0   16
#define F_T1   24
#define F_T2   32
#define F_A0   40
#define F_A1   48
#define F_A2   56
#define F_A3   64
#define F_A4   72
#define F_A5   80
#define F_A6   88
#define F_A7   96
#define F_T3   104
#define F_T4   112
#define F_T5   120
#define F_T6   128
#define F_EPC  136
#define F_CAUSE 144
#define F_START 152

.macro SET_PATTERN
  li    ra, 0x101
  li    t0, 0x105
  li    t1, 0x106
  li    t2, 0x107
  li    a0, 0x10a
  li    a1, 0x10b
  li    a2, 0x10c
  li    a3, 0x10d
  li    a4, 0x10e
  li    a5, 0x10f
  li    a6, 0x110
  li    a7, 0x111
  li    t3, 0x11c
  li    t4, 0x11d
  li    t5, 0x11e
  li    t6, 0x11f
.endm

.macro CHECK reg, val
  li    s7, \val
  bne   \reg, s7, reg_fail
.endm

.macro CLOBBER val
  li    ra, \val
  li    t1, \val
  li    t2, \val
  li    a0, \val
  li    a1, \val
  li    a2, \val
  li    a3, \val
  li    a4, \val
  li    a5, \val
  li    a6, \val
  li    a7, \val
  li    t3, \val
  li    t4, \val
  li    t5, \val
  li    t6, \val
.endm

// Restore the caller-saved registers from the frame at t0, t0 last
.macro RESTORE
  ld    ra, F_RA(t0)
  ld    sp, F_SP(t0)
  ld    t1, F_T1(t0)
  ld    t2, F_T2(t0)
  ld    a0, F_A0(t0)
  ld    a1, F_A1(t0)
  ld    a2, F_A2(t0)
  ld    a3, F_A3(t0)
  ld    a4, F_A4(t0)
  ld    a5, F_A5(t0)
  ld    a6, F_A6(t0)
  ld    a7, F_A7(t0)
  ld    t3, F_T3(t0)
  ld    t4, F_T4(t0)
  ld    t5, F_T5(t0)
  ld    t6, F_T6(t0)
  ld    t0, F_T0(t0)
.endm

  .text
  .global main
main:
  // s0 = mtvec, s1 = ra for crt0, s2 = iteration, s3/s4 = raise X,
  // s5 = frame offset, s6 = spin counter, s7 = check value, s8 = NUM_IRQS,
  // s9 = frame base, s10 = X count, s11 = B count; mscratch = sp
  mv    s1, ra
  csrw  mscratch, sp
  la    t0, trap_fail
  ori   t0, t0, 0x3
  csrrw s0, mtvec, t0
  la    t0, vector_table
  csrw  CSR_MTVT, t0
  li    t0, CLIC_CLICCFG_REG
  li    t1, 0x4 << 1
  sw    t1, 0(t0)
  csrw  CSR_MINTTHRESH, zero
  li    t0, CLIC_CLICINT_REG(IRQ_X)
  li    t1, CFG_X
  sw    t1, 0(t0)
  li    t0, CLIC_CLICINT_REG(IRQ_B)
  li    t1, CFG_B
  sw    t1, 0(t0)
  li    s2, 0
  li    s3, CLIC_CLICINT_REG(IRQ_X)
  li    s4, CFG_X | CLICINT_IP
  li    s5, 0
  li    s8, NUM_IRQS
  li    s10, 0
  li    s11, 0
  csrsi mstatus, 0x8                  // MIE

loop:
  li    s9, COLD_BASE
  add   s9, s9, s5
  csrw  CSR_MTFA, s9                  // fresh frames for X and a nested B
  li    s6, 512
  add   s5, s5, s6
  li    s6, COLD_MASK
  and   s5, s5, s6
  SET_PATTERN
  sw    s4, 0(s3)                     // raise X
  addi  s2, s2, 1
  li    s6, 2000
1:
  bge   s10, s2, 2f                   // X handled
  addi  s6, s6, -1
  bnez  s6, 1b
  mv    a1, s10
  li    a0, 3
  j     done
2:
  li    s7, 0x102
  csrr  s6, mscratch
  bne   sp, s6, reg_fail
  CHECK ra, 0x101
  CHECK t0, 0x105
  CHECK t1, 0x106
  CHECK t2, 0x107
  CHECK a0, 0x10a
  CHECK a1, 0x10b
  CHECK a2, 0x10c
  CHECK a3, 0x10d
  CHECK a4, 0x10e
  CHECK a5, 0x10f
  CHECK a6, 0x110
  CHECK a7, 0x111
  CHECK t3, 0x11c
  CHECK t4, 0x11d
  CHECK t5, 0x11e
  CHECK t6, 0x11f
  // A delay that varies with the iteration, so that the iterations do not lock
  // to the period of the interrupt generator
  andi  s6, s2, 0x3f
3:
  beqz  s6, 4f
  addi  s6, s6, -1
  j     3b
4:
  blt   s11, s8, loop                 // until every generator interrupt was handled
  li    a0, 0
  j     done

reg_fail:
  mv    a1, s7
  li    a0, 2
done:
  csrci mstatus, 0x8
  csrw  mtvec, s0
  csrr  sp, mscratch
  mv    ra, s1
  ret

  // Handler of X: own frame, mepc/mcause saved, interrupts re-enabled at once
  .balign 64
x_handler:
  csrrw t0, CSR_MTFA_INC, zero        // t0 = this frame; mtfa moves to the next one
  csrr  t1, mepc
  sd    t1, F_EPC(t0)
  csrr  t1, mcause
  sd    t1, F_CAUSE(t0)
  csrsi mstatus, 0x8                  // B may nest from here
  addi  s10, s10, 1
  CLOBBER -1
  csrr  t1, mcycle                    // stay 80 cycles, so that B also lands in here
  sd    t1, F_START(t0)
5:
  csrr  t1, mcycle
  ld    t2, F_START(t0)
  sub   t1, t1, t2
  li    t2, 80
  blt   t1, t2, 5b
  csrci mstatus, 0x8
  ld    t1, F_EPC(t0)
  csrw  mepc, t1
  ld    t1, F_CAUSE(t0)
  csrw  mcause, t1
  csrw  CSR_MTFA, t0                  // back to this frame
  RESTORE
  mret

  // Handler of B (the generator interrupt): nothing nests on it
  .balign 64
irq_handler:
  csrr  t0, CSR_MTFA                  // t0 = this frame
  addi  s11, s11, 1
  CLOBBER -2
  RESTORE
  mret

  .balign 256                         // the mtvec base in CLIC mode
trap_fail:
  csrr  a1, mcause
  csrr  a2, mepc
  li    a0, 4
  j     done

  .data
  .balign 2048                        // 256 entries of 8 bytes
vector_table:
  .rept IRQ_X
  .dword trap_fail
  .endr
  .dword x_handler                    // 9
  .rept IRQ_B - IRQ_X - 1
  .dword trap_fail
  .endr
  .dword irq_handler                  // 19
  .rept 256 - IRQ_B - 1
  .dword trap_fail
  .endr
