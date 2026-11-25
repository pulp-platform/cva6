// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Jumps into the instruction SPM, with paging, right after stores to it.
//
// Sv39 maps the first GiB (which holds the instruction SPM) and DRAM with two
// identity gigapages. A function is copied into I-cache way 1, configured as
// SPM with way 2. In S-mode, each iteration flushes the TLBs, stores to way 2
// and jumps into the function: its fetch misses in the ITLB while the stores
// reach the SPM through the data cache, and the page walk needs the data
// cache. Every call must return; the test hangs otherwise.
//
// Return codes: 0 pass, 2 wrong result (a1 = iteration, a2 = result),
// 3 unexpected exception (a1 = mcause, a2 = mepc).

#define ISPM_BASE              0x01A00000
#define FUNC_ADDR              (ISPM_BASE + 1 * 4096 + 0x100)   // way 1
#define DATA_ADDR              (ISPM_BASE + 2 * 4096 + 0x300)   // way 2
#define CSR_ICACHE_EN          0x7C0
#define CSR_ICACHE_SPM_WAYS    0x5E1
#define ITER                   16
#define PTE_GIGA_RWX           0xEF         // D A G X W R V
#define CAUSE_ECALL_S          9
#define MSTATUS_MPP            (3 << 11)
#define MSTATUS_MPP_S          (1 << 11)

  .text
  .global main
main:
  // s0 = mtvec, s1 = ra for crt0, s10 = sp for crt0
  mv    s1, ra
  mv    s10, sp
  la    t0, m_trap
  csrrw s0, mtvec, t0
  li    t0, -1
  srli  t0, t0, 2
  csrw  pmpaddr0, t0
  li    t0, 0x0f                      // TOR, RWX
  csrw  pmpcfg0, t0
  csrwi CSR_ICACHE_EN, 0
  fence.i
  csrwi CSR_ICACHE_SPM_WAYS, 0x6      // ways 1 and 2 are SPM
  csrwi CSR_ICACHE_EN, 1

  // Copy the function into way 1
  la    t0, spm_func
  li    t1, FUNC_ADDR
  ld    t2, 0(t0)
  sd    t2, 0(t1)
  ld    t2, 8(t0)
  sd    t2, 8(t1)
  fence.i

  // Page table: identity gigapages for 0x0000_0000 and 0x8000_0000
  la    t1, root_pt
  li    t0, PTE_GIGA_RWX              // PPN 0
  sd    t0, 0(t1)
  li    t0, ((0x80000000 >> 12) << 10) | PTE_GIGA_RWX
  sd    t0, 2*8(t1)
  fence
  srli  t1, t1, 12
  li    t0, 8 << 60
  or    t0, t0, t1
  csrw  satp, t0
  sfence.vma

  // To S-mode
  la    t0, s_code
  csrw  mepc, t0
  li    t0, MSTATUS_MPP
  csrc  mstatus, t0
  li    t0, MSTATUS_MPP_S
  csrs  mstatus, t0
  mret

s_code:
  li    s2, FUNC_ADDR
  li    s3, DATA_ADDR
  li    s4, ITER
1:
  sfence.vma                          // the next fetch from the SPM misses in the ITLB
  sd    s4, 0(s3)
  sd    s4, 8(s3)
  sd    s4, 16(s3)
  sd    s4, 24(s3)
  mv    a0, s4
  jalr  ra, 0(s2)
  slli  t0, s4, 1
  addi  t0, t0, 6                     // 2 * (a0 + 3)
  mv    a1, s4
  mv    a2, a0
  li    a0, 2
  bne   a2, t0, 2f
  addi  s4, s4, -1
  bnez  s4, 1b
  li    a0, 0
2:
  ecall                               // back to M-mode with a0

  .balign 4
m_trap:
  csrr  t0, mcause
  li    t1, CAUSE_ECALL_S
  beq   t0, t1, done
  mv    a1, t0
  csrr  a2, mepc
  li    a0, 3
done:
  csrw  satp, zero
  sfence.vma
  csrwi CSR_ICACHE_EN, 0
  fence.i
  csrwi CSR_ICACHE_SPM_WAYS, 0
  csrwi CSR_ICACHE_EN, 1
  li    t0, MSTATUS_MPP               // MPP back to M for crt0
  csrs  mstatus, t0
  csrw  mtvec, s0
  mv    sp, s10
  mv    ra, s1
  ret

  // The function copied into the SPM: returns 2 * (a0 + 3)
  .balign 16
  .option push
  .option norvc
spm_func:
  addi  a0, a0, 3
  slli  a0, a0, 1
  ret
  nop
  .option pop

  .data
  .balign 4096
root_pt:
  .zero 4096
