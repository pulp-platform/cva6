// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// AMOs next to other data-cache traffic, for the per-AMO latency log of the
// testbench (build with AMO_MON=1, HPDcache configurations). Each phase uses
// its own counter, which identifies it in the log:
//   A  an AMO alone
//   B  an AMO right after a store that misses (write-allocate refill)
//   C  an AMO followed by younger loads that miss
//   D  an AMO as the first instruction of an interrupt handler whose entry
//      stacks the caller-saved registers to fresh lines
//   E  an AMO followed by a chain of dependent loads that miss, which reach
//      the cache one by one
// Every counter must end at its iteration count.
//
// Return codes: 0 pass, 2 counter wrong (a1 = counter, a2 = phase),
// 3 unexpected exception (a1 = mcause, a2 = mepc).

#define CLIC_BASE              0x10040000
#define CLIC_CLICCFG_REG       (CLIC_BASE + 0x0)
#define CLIC_CLICINT_REG(id)   (CLIC_BASE + 0x1000 + 0x4 * (id))
#define CLICINT_IP             (1 << 0)
#define CLICINT_IE             (1 << 8)
#define CLICINT_SHV            (1 << 16)
#define CLICINT_TRIG_EDGE      (1 << 17)
#define CLICINT_RSTK_CALLER    (1 << 19)
#define CLICINT_CTL(l)         ((l) << 24)

#define IRQ_ID                 9
#define IRQ_CFG                (CLICINT_IE | CLICINT_SHV | CLICINT_TRIG_EDGE | CLICINT_RSTK_CALLER | CLICINT_CTL(0xff))
#define CSR_MTFA               0x308
#define COLD_BASE              0x80200000   // fresh lines, never touched before
#define N                      32

  .text
  .global main
main:
  // Keep ra/s0/s1 for the return to crt0; s9-s11 belong to the handler.
  mv    s1, ra
  la    t0, trap_fail
  ori   t0, t0, 0x3
  csrrw s0, mtvec, t0
  la    t0, vector_table
  csrw  0x307, t0                     // mtvt
  li    t0, CLIC_CLICCFG_REG
  li    t1, 0x4 << 1
  sw    t1, 0(t0)
  csrwi 0x347, 0                      // mintthresh

  li    t2, 1
  li    s2, COLD_BASE                 // next fresh line

  // A: an AMO alone
  la    s3, cnt_a
  li    s4, N
1:
  amoadd.d zero, t2, (s3)
  nop
  nop
  nop
  nop
  addi  s4, s4, -1
  bnez  s4, 1b

  // B: an AMO right after a store that misses
  la    s3, cnt_b
  li    s4, N
1:
  sd    zero, 0(s2)
  addi  s2, s2, 16
  amoadd.d zero, t2, (s3)
  addi  s4, s4, -1
  bnez  s4, 1b

  // C: an AMO followed by younger loads that miss
  la    s3, cnt_c
  li    s4, N
1:
  amoadd.d zero, t2, (s3)
  ld    t3, 0(s2)
  ld    t4, 16(s2)
  ld    t5, 32(s2)
  ld    t6, 48(s2)
  ld    a4, 64(s2)
  ld    a5, 80(s2)
  addi  s2, s2, 96
  addi  s4, s4, -1
  bnez  s4, 1b

  // E: an AMO followed by dependent loads that miss
  la    s3, cnt_e
  li    s4, N
1:
  amoadd.d zero, t2, (s3)
  ld    t3, 0(s2)
  and   t3, t3, zero
  add   t4, s2, t3
  ld    t3, 16(t4)
  and   t3, t3, zero
  add   t4, t4, t3
  ld    t3, 32(t4)
  and   t3, t3, zero
  add   t4, t4, t3
  ld    t3, 48(t4)
  addi  s2, s2, 64
  addi  s4, s4, -1
  bnez  s4, 1b

  // D: an AMO first in a handler whose entry stacks to fresh lines
  la    s9, cnt_d
  li    s11, 1
  li    s10, 0
  li    s5, CLIC_CLICINT_REG(IRQ_ID)
  li    s6, IRQ_CFG
  sw    s6, 0(s5)
  ori   s6, s6, CLICINT_IP
  li    s4, 0
  csrsi mstatus, 0x8
1:
  csrw  CSR_MTFA, s2
  addi  s2, s2, 256
  sw    s6, 0(s5)                     // raise the interrupt
  addi  s4, s4, 1
2:
  blt   s10, s4, 2b
  li    t0, N
  blt   s4, t0, 1b
  csrci mstatus, 0x8

  // Check the counters
  li    a2, 0
  la    t0, cnt_a
  li    t1, N
1:
  ld    a1, 0(t0)
  bne   a1, t1, 2f
  addi  t0, t0, 64
  addi  a2, a2, 1
  li    t3, 5
  blt   a2, t3, 1b
  li    a0, 0
  j     done
2:
  li    a0, 2
done:
  csrw  mtvec, s0
  mv    ra, s1
  ret

  // Vectored handler for IRQ_ID (SHV: entered from the mtvt entry)
  .balign 64
irq_handler:
  amoadd.d zero, s11, (s9)
  addi  s10, s10, 1
  mret

  .balign 256                         // the mtvec base in CLIC mode
trap_fail:
  csrr  a1, mcause
  csrr  a2, mepc
  li    a0, 3
  j     done

  .data
  .balign 2048                        // the vector table is aligned to its size
vector_table:
  .rept IRQ_ID
  .dword trap_fail
  .endr
  .dword irq_handler

  .balign 64
cnt_a:
  .dword 0
  .balign 64
cnt_b:
  .dword 0
  .balign 64
cnt_c:
  .dword 0
  .balign 64
cnt_d:
  .dword 0
  .balign 64
cnt_e:
  .dword 0
