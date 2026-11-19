// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Fault on a CLIC vector table read.
//
// The vector table read is a load with the privilege of the handler. An
// S-mode hardware-vectored interrupt whose table sits in a PMP region without
// read permission must take a load access fault, with epc and tval both
// holding the address of the table entry. The fault is not delegated, so it
// traps to M-mode from S-mode.
//
// Return codes: 0 pass, 2 wrong cause (a1 = mcause), 3 wrong mepc (a1 = mepc,
// a2 = expected), 4 wrong mtval (a1 = mtval, a2 = expected), 5 wrong mstatus.MPP
// (a1 = mstatus), 6 the S-mode handler ran or no fault was taken.

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
#define PMP_TOR_X              0x0c
#define CAUSE_LOAD_ACCESS      5

  .text
  .global main
main:
  // Keep ra in mscratch and mtvec in s0; s7 holds the expected entry address.
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
  la    s7, s_vector_table
  addi  s7, s7, 8 * IRQ_S

  // PMP: all memory RWX except the vector table, which is execute-only
  la    t0, s_vector_table
  srli  t1, t0, 2
  csrw  pmpaddr0, t1                  // [0, table): RWX
  li    t2, 2048
  add   t1, t0, t2
  srli  t1, t1, 2
  csrw  pmpaddr1, t1                  // [table, table + 2 KiB): X
  li    t1, -1
  srli  t1, t1, 2
  csrw  pmpaddr2, t1                  // the rest: RWX
  li    t1, PMP_TOR_RWX | (PMP_TOR_X << 8) | (PMP_TOR_RWX << 16)
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
  addi  t0, t0, -1
  bnez  t0, 1b
  li    a0, 6                         // no fault was taken
  ecall

  // S-mode handler of IRQ_S: must not run, as its table entry cannot be read
  .balign 64
s_handler:
  li    a0, 6
  ecall

  .balign 256                         // the stvec base in CLIC mode
s_trap_fail:
  li    a0, 6
  ecall

  // M-mode trap entry: the table read fault, or an ecall from S-mode
  .balign 256                         // the mtvec base in CLIC mode
irq_handler:
  csrr  t5, mcause
  mv    a1, t5
  li    t4, 0xfff                     // exception code; CLIC mode adds mpp/mpie/mpil
  and   t6, t5, t4
  li    t4, 9                         // ecall from S-mode: a0 holds the result
  bgez  t5, 1f
  li    t6, 0                         // an interrupt to M-mode: wrong cause
1:
  beq   t6, t4, done
  li    a0, 2
  li    t4, CAUSE_LOAD_ACCESS
  bne   t6, t4, done
  li    a0, 3
  csrr  a1, mepc
  mv    a2, s7
  bne   a1, a2, done
  li    a0, 4
  csrr  a1, mtval
  bne   a1, a2, done
  li    a0, 5
  csrr  a1, mstatus
  srli  t4, a1, 11
  andi  t4, t4, 0x3
  li    t6, 1                         // MPP = S
  bne   t4, t6, done
  li    a0, 0
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
