// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Permission checks on locked TLB entries.
//
// Sv39 is enabled with an empty root page table, so only the locked entries
// translate. Three 4 KiB data pages are locked: an execute-only page, a
// read-write page with A clear, and a normal read-write page. M-mode accesses
// them through MPRV with S privilege. A load from the execute-only page must
// fault without MXR and succeed with it, a load or store to the page with A
// clear must fault, and the normal page must work.
//
// Return codes: 0 pass, 2 wrong outcome (a1 = check number, a2 = cause of the
// fault taken, 0 for none, a3 = value loaded).

#define CSR_TLB_LOCK_PTE_1     0x5C3
#define CSR_TLB_LOCK_VPN_1     0x5C4
#define CSR_TLB_LOCK_ID_1      0x5C5
#define CSR_TLB_LOCK_PTE_2     0x5C6
#define CSR_TLB_LOCK_VPN_2     0x5C7
#define CSR_TLB_LOCK_ID_2      0x5C8
#define CSR_TLB_LOCK_PTE_3     0x5C9
#define CSR_TLB_LOCK_VPN_3     0x5CA
#define CSR_TLB_LOCK_ID_3      0x5CB
#define VA_BASE                0xC0000000
#define PA_BASE                0x81200000
#define PTE(pa, flags)         ((((pa) >> 12) << 10) | (flags))
// PTE flags: V 0x01, R 0x02, W 0x04, X 0x08, G 0x20, A 0x40, D 0x80
#define PTE_XONLY              0xE9         // D A G X V
#define PTE_NOA                0xA7         // D G W R V
#define PTE_RW                 0xE7         // D A G W R V
// VPN CSR: the virtual page, S stage (bit 5), data (bit 4), 4 KiB size (1)
#define VPN(va)                ((va) | (1 << 5) | (1 << 4) | 1)
#define MSTATUS_MPRV           (1 << 17)
#define MSTATUS_MXR            (1 << 19)
#define MSTATUS_MPP            (3 << 11)
#define MSTATUS_MPP_S          (1 << 11)
#define MARKER                 0x7065726d73000000
#define CAUSE_LOAD_PF          13
#define CAUSE_STORE_PF         15

  // Set MPRV with S privilege, so that the next data access is translated
.macro MPRV_S
  li    t0, MSTATUS_MPP
  csrc  mstatus, t0
  li    t0, MSTATUS_MPRV | MSTATUS_MPP_S
  csrs  mstatus, t0
.endm

.macro MPRV_OFF
  li    t0, MSTATUS_MPRV
  csrc  mstatus, t0
  li    t0, MSTATUS_MPP
  csrs  mstatus, t0
.endm

  // Expect a1 = check number; s3 = expected cause (0 for none), s4 = expected
  // value (checked only without a fault)
.macro CHECK num
  li    a1, \num
  bne   s2, s3, fail
  bnez  s3, 1f
  bne   a3, s4, fail
1:
.endm

  .text
  .global main
main:
  // s0 = mtvec, s1 = ra for crt0, s2 = cause of the last fault (0 for none)
  mv    s1, ra
  la    t0, trap_skip
  csrrw s0, mtvec, t0
  li    t0, -1
  srli  t0, t0, 2
  csrw  pmpaddr0, t0
  li    t0, 0x0f                      // TOR, RWX
  csrw  pmpcfg0, t0

  li    t0, MARKER
  li    t1, PA_BASE + 0x008
  sd    t0, 0(t1)
  li    t1, PA_BASE + 0x1008
  addi  t2, t0, 1
  sd    t2, 0(t1)
  li    t1, PA_BASE + 0x2008
  addi  t2, t0, 2
  sd    t2, 0(t1)
  fence

  la    t0, root_pt
  srli  t0, t0, 12
  li    t1, 8 << 60
  or    t0, t0, t1
  csrw  satp, t0
  sfence.vma
  li    t0, PTE(PA_BASE, PTE_XONLY)
  csrw  CSR_TLB_LOCK_PTE_1, t0
  li    t0, VPN(VA_BASE)
  csrw  CSR_TLB_LOCK_VPN_1, t0
  csrwi CSR_TLB_LOCK_ID_1, 1
  li    t0, PTE(PA_BASE + 0x1000, PTE_NOA)
  csrw  CSR_TLB_LOCK_PTE_2, t0
  li    t0, VPN(VA_BASE + 0x1000)
  csrw  CSR_TLB_LOCK_VPN_2, t0
  csrwi CSR_TLB_LOCK_ID_2, 1
  li    t0, PTE(PA_BASE + 0x2000, PTE_RW)
  csrw  CSR_TLB_LOCK_PTE_3, t0
  li    t0, VPN(VA_BASE + 0x2000)
  csrw  CSR_TLB_LOCK_VPN_3, t0
  csrwi CSR_TLB_LOCK_ID_3, 1

  .option push
  .option norvc                       // the trap handler skips 4-byte instructions
  // 1: the normal page loads
  li    s2, 0
  li    a3, 0
  li    t1, VA_BASE + 0x2008
  MPRV_S
  ld    a3, 0(t1)
  MPRV_OFF
  li    s3, 0
  li    s4, MARKER + 2
  CHECK 1
  // 2: the execute-only page does not load without MXR
  li    s2, 0
  li    a3, 0
  li    t1, VA_BASE + 0x008
  MPRV_S
  ld    a3, 0(t1)
  MPRV_OFF
  li    s3, CAUSE_LOAD_PF
  CHECK 2
  // 3: it loads with MXR
  li    s2, 0
  li    a3, 0
  li    t1, VA_BASE + 0x008
  li    t0, MSTATUS_MXR
  csrs  mstatus, t0
  MPRV_S
  ld    a3, 0(t1)
  MPRV_OFF
  li    t0, MSTATUS_MXR
  csrc  mstatus, t0
  li    s3, 0
  li    s4, MARKER
  CHECK 3
  // 4: the page with A clear does not load
  li    s2, 0
  li    a3, 0
  li    t1, VA_BASE + 0x1008
  MPRV_S
  ld    a3, 0(t1)
  MPRV_OFF
  li    s3, CAUSE_LOAD_PF
  CHECK 4
  // 5: nor store
  li    s2, 0
  li    t1, VA_BASE + 0x1010
  MPRV_S
  sd    zero, 0(t1)
  MPRV_OFF
  li    s3, CAUSE_STORE_PF
  CHECK 5
  .option pop

  li    a0, 0
  j     done
fail:
  mv    a2, s2
  li    a0, 2
done:
  csrwi CSR_TLB_LOCK_ID_1, 0
  csrwi CSR_TLB_LOCK_ID_2, 0
  csrwi CSR_TLB_LOCK_ID_3, 0
  csrw  satp, zero
  sfence.vma
  csrw  mtvec, s0
  mv    ra, s1
  ret

  // Record the fault in s2 and resume after the faulting access in M-mode
  .balign 4
trap_skip:
  li    t0, MSTATUS_MPRV
  csrc  mstatus, t0
  csrr  s2, mcause
  csrr  t0, mepc
  addi  t0, t0, 4
  csrw  mepc, t0
  li    t0, MSTATUS_MPP
  csrs  mstatus, t0
  mret

  .data
  .balign 4096
root_pt:
  .zero 4096
