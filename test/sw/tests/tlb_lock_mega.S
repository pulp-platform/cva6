// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// A locked TLB entry for a 2 MiB megapage.
//
// Sv39 is enabled with an empty root page table, so only a locked entry
// can translate. Lock slot 1 maps the data megapage at virtual 0xC0000000 to
// physical 0x81200000. M-mode then accesses it through MPRV with S privilege:
// loads in the first, a middle and the last 4 KiB page of the megapage, and a
// store in a middle page, read back physically. Each must translate within the
// megapage; any access the lock does not cover takes a page fault.
//
// Return codes: 0 pass, 2 wrong value (a1 = check number, a2 = value),
// 3 unexpected exception (a1 = mcause, a2 = mepc, a3 = mtval).

#define CSR_TLB_LOCK_PTE_1     0x5C3
#define CSR_TLB_LOCK_VPN_1     0x5C4
#define CSR_TLB_LOCK_ID_1      0x5C5
#define VA_BASE                0xC0000000
#define PA_BASE                0x81200000   // 2 MiB aligned
// PTE: PPN, D A G W R V
#define LOCK_PTE               (((PA_BASE >> 12) << 10) | 0xE7)
// VPN CSR: the virtual page, S stage (bit 5), data (bit 4), megapage size (2)
#define LOCK_VPN               (VA_BASE | (1 << 5) | (1 << 4) | 2)
#define MSTATUS_MPRV           (1 << 17)
#define MSTATUS_MPP            (3 << 11)
#define MSTATUS_MPP_S          (1 << 11)
#define MARKER                 0x6d65676170616765

  .text
  .global main
main:
  // s0 = mtvec, s1 = ra for crt0
  mv    s1, ra
  la    t0, trap_fail
  csrrw s0, mtvec, t0
  // PMP: S-mode may access everything
  li    t0, -1
  srli  t0, t0, 2
  csrw  pmpaddr0, t0
  li    t0, 0x0f                      // TOR, RWX
  csrw  pmpcfg0, t0

  // Markers at the physical addresses the loads must reach
  li    t0, MARKER
  li    t1, PA_BASE + 0x010
  sd    t0, 0(t1)
  li    t1, PA_BASE + 0x5008
  addi  t2, t0, 1
  sd    t2, 0(t1)
  li    t1, PA_BASE + 0x1ff010
  addi  t2, t0, 2
  sd    t2, 0(t1)
  li    t1, PA_BASE + 0x3000
  sd    zero, 0(t1)
  fence

  // Sv39 with an empty root table, and the lock
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

  // Data accesses with S privilege
  li    t0, MSTATUS_MPP
  csrc  mstatus, t0
  li    t0, MSTATUS_MPRV | MSTATUS_MPP_S
  csrs  mstatus, t0
  li    t1, VA_BASE + 0x010
  ld    a4, 0(t1)
  li    t1, VA_BASE + 0x5008
  ld    a5, 0(t1)
  li    t1, VA_BASE + 0x1ff010
  ld    a6, 0(t1)
  li    t1, VA_BASE + 0x3000
  li    t2, MARKER + 3
  sd    t2, 0(t1)
  li    t0, MSTATUS_MPRV
  csrc  mstatus, t0
  li    t0, MSTATUS_MPP
  csrs  mstatus, t0
  fence
  li    t1, PA_BASE + 0x3000
  ld    a7, 0(t1)

  li    a0, 2
  li    t0, MARKER
  li    a1, 1
  mv    a2, a4
  bne   a4, t0, done
  li    a1, 2
  mv    a2, a5
  addi  t1, t0, 1
  bne   a5, t1, done
  li    a1, 3
  mv    a2, a6
  addi  t1, t0, 2
  bne   a6, t1, done
  li    a1, 4
  mv    a2, a7
  addi  t1, t0, 3
  bne   a7, t1, done
  li    a0, 0

done:
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
