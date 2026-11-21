// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// A locked TLB entry and a superpage covering it.
//
// The page table maps the 2 MiB superpage at virtual 0xC0000000, and lock
// slot 1 maps one 4 KiB page inside it elsewhere. Through MPRV with S
// privilege, M-mode first loads from another page of the superpage, which
// fills the DTLB with the superpage, then from the locked page. The locked
// mapping must take precedence over the superpage entry.
//
// Return codes: 0 pass, 2 wrong value (a1 = check number, a2 = value),
// 3 unexpected exception (a1 = mcause, a2 = mepc, a3 = mtval).

#define CSR_CUR_CLRS           0x5C0
#define CSR_TLB_LOCK_PTE_1     0x5C3
#define CSR_TLB_LOCK_VPN_1     0x5C4
#define CSR_TLB_LOCK_ID_1      0x5C5
#define VA_BASE                0xC0000000   // a 2 MiB superpage in the page table
#define PA_SUPER               0x81400000   // where the superpage maps it
#define PA_LOCK                0x81205000   // where the lock maps its 4 KiB page
#define VA_LOCK                (VA_BASE + 0x5000)
#define PTE_PPN(pa)            (((pa) >> 12) << 10)
// VPN CSR: the virtual page, S stage (bit 5), data (bit 4), 4 KiB size (1)
#define LOCK_VPN               (VA_LOCK | (1 << 5) | (1 << 4) | 1)
#define LOCK_PTE               (PTE_PPN(PA_LOCK) | 0xE7)   // D A G W R V
#define MSTATUS_MPRV           (1 << 17)
#define MSTATUS_MPP            (3 << 11)
#define MSTATUS_MPP_S          (1 << 11)
#define MARKER_SUPER           0x7375706572000000
#define MARKER_LOCK            0x6c6f636b00000000

  .text
  .global main
main:
  // s0 = mtvec, s1 = ra for crt0
  mv    s1, ra
  la    t0, trap_fail
  csrrw s0, mtvec, t0
  li    t0, -1
  srli  t0, t0, 2
  csrw  pmpaddr0, t0
  li    t0, 0x0f                      // TOR, RWX
  csrw  pmpcfg0, t0

  // Markers: the superpage's view and the lock's view of the locked page,
  // and another page of the superpage
  li    t0, MARKER_SUPER
  li    t1, PA_SUPER + 0x5008
  sd    t0, 0(t1)
  li    t1, PA_SUPER + 0x6008
  addi  t2, t0, 1
  sd    t2, 0(t1)
  li    t0, MARKER_LOCK
  li    t1, PA_LOCK + 0x008
  sd    t0, 0(t1)
  fence

  // Page table: root[3] -> l1_pt, l1_pt[0] = 2 MiB leaf at PA_SUPER
  la    t0, l1_pt
  srli  t0, t0, 12
  slli  t0, t0, 10
  ori   t0, t0, 0x01                  // V, non-leaf
  la    t1, root_pt
  sd    t0, 3*8(t1)
  li    t0, PTE_PPN(PA_SUPER) | 0xE7  // D A G W R V
  la    t1, l1_pt
  sd    t0, 0(t1)
  fence
  la    t0, root_pt
  srli  t0, t0, 12
  li    t1, 8 << 60
  or    t0, t0, t1
  csrw  satp, t0
  sfence.vma
  li    t0, LOCK_PTE
  csrw  CSR_TLB_LOCK_PTE_1, t0
  li    t0, LOCK_VPN
  csrw  CSR_TLB_LOCK_VPN_1, t0
  csrwi CSR_TLB_LOCK_ID_1, 1          // valid, ASID 0, VMID 0
  li    t0, MSTATUS_MPP
  csrc  mstatus, t0
  li    t0, MSTATUS_MPRV | MSTATUS_MPP_S
  csrs  mstatus, t0
  li    t1, VA_BASE + 0x6008
  ld    a4, 0(t1)                     // fills the DTLB with the superpage
  li    t1, VA_LOCK + 0x008
  ld    a5, 0(t1)                     // must use the lock
  li    t0, MSTATUS_MPRV
  csrc  mstatus, t0
  li    t0, MSTATUS_MPP
  csrs  mstatus, t0

  li    a0, 2
  li    a1, 1
  mv    a2, a4
  li    t0, MARKER_SUPER + 1
  bne   a4, t0, done
  li    a1, 2
  mv    a2, a5
  li    t0, MARKER_LOCK
  bne   a5, t0, done
  li    a0, 0

done:
  li    t0, 0xffff
  csrw  CSR_CUR_CLRS, t0
  csrwi CSR_TLB_LOCK_ID_1, 0
  csrw  satp, zero
  sfence.vma
  csrw  mtvec, s0
  mv    ra, s1
  ret

  .balign 4
trap_fail:
  li    t0, MSTATUS_MPRV
  csrc  mstatus, t0
  csrr  a1, mcause
  csrr  a2, mepc
  csrr  a3, mtval
  li    a0, 3
  j     done

  .data
  .balign 4096
root_pt:
  .zero 4096
l1_pt:
  .zero 4096
