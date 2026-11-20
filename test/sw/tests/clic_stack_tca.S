// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// CLIC interrupt stacking all GPRs into the task context (rstk = 2).
//
// mtca is written with an address 128 bytes before the end of a page, on cache
// lines never used before. The task context base is 256-byte aligned, so it
// reads back aligned down and the 256-byte frame stays within the page. The
// interrupt arrives while every GPR holds a known value; its handler reads the
// frame at once, from the last slot down, while the frame is still being
// drained, and each slot must hold the interrupted register. The interrupt is
// taken twice, on two fresh pages: the first run brings the handler into the
// I-cache, so that in the second one it reads the frame early in the drain.
//
// Return codes: 0 pass, 2 wrong frame slot (a1 = slot, a2 = value read),
// 3 mtca not 256-byte aligned (a1 = mtca), 4 unexpected exception
// (a1 = mcause, a2 = mepc).

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
#define CSR_MTCA               0x309
#define TCA_REQ                0x81200f80   // 128 bytes before a page end, never used before

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
  li    t2, TCA_REQ
  slli  t1, t1, 12                    // a fresh page each run
  add   t2, t2, t1
  csrw  CSR_MTCA, t2
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
1:
  j     1b                            // the handler returns to done

  // Handler: check the frame from the last slot down, using registers the
  // stacking saves first (t0-t2, a0-a2)
  .balign 64
irq_handler:
  csrr  t0, CSR_MTCA
  li    t1, 31
2:
  slli  t2, t1, 3
  add   t2, t0, t2
  ld    a0, 0(t2)
  addi  a1, t1, 0x100                 // expected: 0x100 + slot
  li    a2, 5
  bne   t1, a2, 3f
  li    a1, IRQ_ADDR                  // t0 held the CLIC address
3:
  li    a2, 6
  bne   t1, a2, 4f
  li    a1, IRQ_CFG                   // t1 held the CLIC value
4:
  bne   a0, a1, slot_fail
  addi  t1, t1, -1
  bnez  t1, 2b
  ld    a0, 0(t0)                     // slot 0 holds x0
  bnez  a0, slot_fail
  // After the second run, the base must read back 256-byte aligned
  li    a0, 0
  la    t2, iter
  ld    t2, 0(t2)
  beqz  t2, 5f
  li    a0, 3
  mv    a1, t0
  andi  t2, t0, 0xff
  bnez  t2, 5f
  li    a0, 0
  j     5f
slot_fail:
  mv    a2, a0
  mv    a1, t1
  li    a0, 2
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
  j     run
finish:
  la    t0, result
  ld    a0, 0(t0)
  ld    a1, 8(t0)
  ld    a2, 16(t0)
  la    t0, saved
  ld    t1, 128(t0)
  csrw  mtvec, t1
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
  j     done

  .data
  .balign 8
saved:
  .rept 17
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
