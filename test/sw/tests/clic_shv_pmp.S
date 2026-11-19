// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// CLIC vector table readable but not executable.
//
// The vector table read is a load with the privilege of the handler, so it
// needs read permission only. An S-mode hardware-vectored interrupt whose
// table sits in a PMP region granting R but not X must reach its handler.
//
// Return codes: 0 pass, 2 S-mode handler not run once (a1 = handler count),
// 4 unexpected exception (a1 = mcause, a2 = mepc).

#define CLIC_BASE              0x10040000
#define CLIC_CLICCFG_REG       (CLIC_BASE + 0x0)
#define CLIC_CLICINT_REG(id)   (CLIC_BASE + 0x1000 + 0x4 * (id))
#define CLICINT_IP             (1 << 0)
#define CLICINT_IE             (1 << 8)
#define CLICINT_SHV            (1 << 16)
#define CLICINT_TRIG_EDGE      (1 << 17)
#define CLICINT_MODE(m)        ((m) << 22)
#define CLICINT_CTL(l)         ((l) << 24)

#define IRQ_S                  9
#define CFG_S                  (CLICINT_IE | CLICINT_SHV | CLICINT_TRIG_EDGE | CLICINT_MODE(1) | CLICINT_CTL(0xff))
#define CSR_STVT               0x107
#define CSR_SINTTHRESH         0x147
#define CSR_MINTTHRESH         0x347
#define PMP_TOR_RWX            0x0f
#define PMP_TOR_R              0x09

  .text
  .global main
main:
  // Keep ra in mscratch and mtvec in s0; s10 counts the S-mode handlers.
  csrw  mscratch, ra
  li    s10, 0
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

  // PMP: all memory RWX except the vector table, which is R only
  la    t0, s_vector_table
  srli  t1, t0, 2
  csrw  pmpaddr0, t1                  // [0, table): RWX
  li    t2, 2048
  add   t1, t0, t2
  srli  t1, t1, 2
  csrw  pmpaddr1, t1                  // [table, table + 2 KiB): R
  li    t1, -1
  srli  t1, t1, 2
  csrw  pmpaddr2, t1                  // the rest: RWX
  li    t1, PMP_TOR_RWX | (PMP_TOR_R << 8) | (PMP_TOR_RWX << 16)
  csrw  pmpcfg0, t1

  // Enter S-mode
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
  csrsi sstatus, 0x2                  // SIE
  li    t0, CLIC_CLICINT_REG(IRQ_S)
  li    t1, CFG_S | CLICINT_IP
  sw    t1, 0(t0)                     // raise the S-mode interrupt
  li    t0, 200
1:
  bnez  s10, 2f
  addi  t0, t0, -1
  bnez  t0, 1b
2:
  mv    a1, s10
  li    a0, 2
  li    t1, 1
  bne   s10, t1, 3f
  li    a0, 0
3:
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

  // M-mode trap entry: the final ecall from S-mode, and unexpected traps
  .balign 256                         // the mtvec base in CLIC mode
irq_handler:
  csrr  t5, mcause
  li    t4, 0xfff                     // exception code; CLIC mode adds mpp/mpie/mpil
  and   t6, t5, t4
  li    t4, 9                         // ecall from S-mode: finish
  bgez  t5, 1f
  li    t6, 0                         // an interrupt to M-mode: fail
1:
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
  .rept 256 - IRQ_S - 1
  .dword s_trap_fail
  .endr
