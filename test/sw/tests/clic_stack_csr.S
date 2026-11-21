// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// CSR instructions at the start of a handler whose registers are being stacked.
//
// The interrupt stacks all GPRs (rstk = 2) into a task context on cache lines
// never used before, so stacking runs for many cycles after its handler has
// started. The handler starts with CSR instructions into the registers stacked
// last (s11, t3-t6), so they wait at commit for most of the stacking: reads
// of mepc and mcause, a swap with mscratch, a bump of mtfa and a write of
// mstatus, which flushes the pipeline. Each must take effect once, with its own result. The
// interrupt is taken twice, on two fresh pages: the first run brings the
// handler into the I-cache.
//
// Return codes: 0 pass, 2 a CSR result is wrong (a1 = check number, a2 = value),
// 4 unexpected exception (a1 = mcause, a2 = mepc).

#define CLIC_BASE              0x10040000
#define CLIC_CLICCFG_REG       (CLIC_BASE + 0x0)
#define CLIC_CLICINT_REG(id)   (CLIC_BASE + 0x1000 + 0x4 * (id))
#define CLICINT_IP             (1 << 0)
#define CLICINT_IE             (1 << 8)
#define CLICINT_SHV            (1 << 16)
#define CLICINT_TRIG_EDGE      (1 << 17)
#define CLICINT_RSTK_ALL       (2 << 19)
#define CLICINT_CTL(l)         ((l) << 24)

#define IRQ_ID                 9
#define IRQ_ADDR               CLIC_CLICINT_REG(IRQ_ID)
#define IRQ_CFG                (CLICINT_IP | CLICINT_IE | CLICINT_SHV | CLICINT_TRIG_EDGE | CLICINT_RSTK_ALL | CLICINT_CTL(0xff))
#define CSR_MTVT               0x307
#define CSR_MTFA               0x308
#define CSR_MTCA               0x309
#define CSR_MTFA_INC           0x7C6
#define TCA_BASE               0x81600000   // a fresh page each run, never used before
#define TFA_VAL                0x81700000
#define MSCRATCH_VAL           0x5c5c5c5c5c5c5c5c

  .text
  .global main
main:
  // Save every register the C runtime relies on; the interrupted context is
  // clobbered by the handler.
  la    t0, saved
  sd    ra, 0(t0)
  sd    sp, 8(t0)
  sd    gp, 16(t0)
  sd    tp, 24(t0)
  sd    s0, 32(t0)
  sd    s1, 40(t0)
  sd    s2, 48(t0)
  sd    s3, 56(t0)
  sd    s4, 64(t0)
  sd    s5, 72(t0)
  sd    s6, 80(t0)
  sd    s7, 88(t0)
  sd    s8, 96(t0)
  sd    s9, 104(t0)
  sd    s10, 112(t0)
  sd    s11, 120(t0)
  csrr  t1, mtvec
  sd    t1, 128(t0)
  csrr  t1, mscratch
  sd    t1, 136(t0)

  la    t0, trap_fail
  ori   t0, t0, 0x3
  csrw  mtvec, t0
  la    t0, vector_table
  csrw  CSR_MTVT, t0
  li    t0, CLIC_CLICCFG_REG
  li    t1, 0x4 << 1
  sw    t1, 0(t0)
  csrwi 0x347, 0                      // mintthresh
  la    t0, iter
  sd    zero, 0(t0)

run:
  la    t0, iter
  ld    t1, 0(t0)
  li    t2, TCA_BASE
  slli  t1, t1, 12                    // a fresh page each run
  add   t2, t2, t1
  csrw  CSR_MTCA, t2
  li    t0, TFA_VAL
  csrw  CSR_MTFA, t0
  li    t0, MSCRATCH_VAL
  csrw  mscratch, t0
  csrsi mstatus, 0x8

  // Every GPR gets 0x100 + its number, except t0/t1, which raise the interrupt
  li    x1, 0x101
  li    x2, 0x102
  li    x3, 0x103
  li    x4, 0x104
  li    x7, 0x107
  li    x8, 0x108
  li    x9, 0x109
  li    x10, 0x10a
  li    x11, 0x10b
  li    x12, 0x10c
  li    x13, 0x10d
  li    x14, 0x10e
  li    x15, 0x10f
  li    x16, 0x110
  li    x17, 0x111
  li    x18, 0x112
  li    x19, 0x113
  li    x20, 0x114
  li    x21, 0x115
  li    x22, 0x116
  li    x23, 0x117
  li    x24, 0x118
  li    x25, 0x119
  li    x26, 0x11a
  li    x27, 0x11b
  li    x28, 0x11c
  li    x29, 0x11d
  li    x30, 0x11e
  li    x31, 0x11f
  li    t0, IRQ_ADDR
  li    t1, IRQ_CFG
  sw    t1, 0(t0)                     // raise the interrupt
spin:
  j     spin                          // the handler returns to done

  // Handler: CSR instructions into registers that are stacked last
  .balign 64
irq_handler:
  csrr  t3, mepc                      // the interrupted instruction
  csrr  t4, mcause
  csrrw t5, mscratch, t5              // t5 held 0x11e
  csrrw t6, CSR_MTFA_INC, zero        // bumps mtfa once
  csrrci s11, mstatus, 0x2            // a write, which flushes; s11 held 0x11b

  li    a0, 2
  li    a1, 1
  mv    a2, t3
  la    t0, spin
  bne   t3, t0, 5f
  li    a1, 2
  mv    a2, t4
  li    t0, 0x8000000000000fff
  and   t1, t4, t0
  li    t0, 0x8000000000000000 | IRQ_ID
  bne   t1, t0, 5f
  li    a1, 3
  mv    a2, t5
  li    t0, MSCRATCH_VAL
  bne   t5, t0, 5f
  li    a1, 4
  csrr  a2, mscratch
  li    t0, 0x11e
  bne   a2, t0, 5f
  li    a1, 5
  mv    a2, t6
  li    t0, TFA_VAL
  bne   t6, t0, 5f
  li    a1, 6
  csrr  a2, CSR_MTFA
  li    t0, TFA_VAL + 256
  bne   a2, t0, 5f
  li    a1, 7
  mv    a2, s11
  li    t0, 0x1800                    // MPP = M: the value of mstatus, not 0x11b
  and   t1, s11, t0
  bne   t1, t0, 5f
  li    a0, 0
5:
  la    t2, result
  sd    a0, 0(t2)
  sd    a1, 8(t2)
  sd    a2, 16(t2)
  la    t2, done
  csrw  mepc, t2
  li    t2, 0x1800                    // return to M-mode
  csrs  mstatus, t2
  mret

done:
  csrci mstatus, 0x8
  la    t0, result
  ld    a0, 0(t0)
  bnez  a0, finish
  la    t0, iter                      // run again once the first run passed
  ld    t1, 0(t0)
  bnez  t1, finish
  li    t1, 1
  sd    t1, 0(t0)
  la    t0, result
  li    t1, 4
  sd    t1, 0(t0)
  j     run
finish:
  la    t0, result
  ld    a0, 0(t0)
  ld    a1, 8(t0)
  ld    a2, 16(t0)
  la    t0, saved
  ld    t1, 128(t0)
  csrw  mtvec, t1
  ld    t1, 136(t0)
  csrw  mscratch, t1
  ld    ra, 0(t0)
  ld    sp, 8(t0)
  ld    gp, 16(t0)
  ld    tp, 24(t0)
  ld    s0, 32(t0)
  ld    s1, 40(t0)
  ld    s2, 48(t0)
  ld    s3, 56(t0)
  ld    s4, 64(t0)
  ld    s5, 72(t0)
  ld    s6, 80(t0)
  ld    s7, 88(t0)
  ld    s8, 96(t0)
  ld    s9, 104(t0)
  ld    s10, 112(t0)
  ld    s11, 120(t0)
  ret

  .balign 256                         // the mtvec base in CLIC mode
trap_fail:
  csrr  a1, mcause
  csrr  a2, mepc
  li    a0, 4
  la    t0, result
  sd    a0, 0(t0)
  sd    a1, 8(t0)
  sd    a2, 16(t0)
  j     finish

  .data
  .balign 8
saved:
  .rept 18
  .dword 0
  .endr
result:
  .dword 4, 0, 0
iter:
  .dword 0
  .balign 2048                        // 256 entries of 8 bytes
vector_table:
  .rept IRQ_ID
  .dword trap_fail
  .endr
  .dword irq_handler
  .rept 256 - IRQ_ID - 1
  .dword trap_fail
  .endr
