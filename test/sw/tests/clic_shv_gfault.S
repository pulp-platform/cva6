// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// A guest page fault on the vector table read of a guest's interrupt.
//
// The G-stage page table maps only the 1 GiB of guest physical memory that
// holds the code, identity-mapped. An SHV interrupt is delegated to VS-mode
// as in clic_vsmode, and the guest points vstvt at an unmapped guest physical
// page (VS-stage translation is off). The table read must take a load guest
// page fault into M-mode with mtval at the table entry, mtval2 holding its
// guest physical address shifted right by 2, and mstatus.GVA set.
//
// Return codes: 0 pass, 2 wrong cause (a1 = mcause), 3 wrong mtval (a1),
// 4 wrong mtval2 (a1), 5 GVA clear (a1 = mstatus).

#define CLIC_BASE              0x10040000
#define CLIC_CLICCFG_REG       (CLIC_BASE + 0x0)
#define CLIC_CLICINT_REG(id)   (CLIC_BASE + 0x1000 + 0x4 * (id))
#define CLIC_CLICINTV_REG(id)  (CLIC_BASE + 0xd000 + (id))
#define CLICINT_IP             (1 << 0)
#define CLICINT_IE             (1 << 8)
#define CLICINT_SHV            (1 << 16)
#define CLICINT_TRIG_EDGE      (1 << 17)
#define CLICINT_MODE_S         (1 << 22)
#define CLICINT_CTL(l)         ((l) << 24)

#define IRQ_ID                 31
#define VSID                   1
#define IRQ_CFG                (CLICINT_IE | CLICINT_SHV | CLICINT_TRIG_EDGE | CLICINT_MODE_S | CLICINT_CTL(0xaa))
#define CSR_STVT               0x107        // vstvt when accessed from VS-mode
#define CSR_HSTATUS            0x600
#define CSR_HGATP              0x680
#define CSR_VSATP              0x280
#define CSR_MTVAL2             0x34B
#define VSTVT_GPA              0xC0000000   // not mapped by the G stage
#define ENTRY_GPA              (VSTVT_GPA + 8 * IRQ_ID)
#define CAUSE_LOAD_GUEST_PF    21
#define MSTATUS_MPV            (1 << 39)
#define MSTATUS_GVA            (1 << 38)
#define MSTATUS_MPP            (3 << 11)
#define MSTATUS_MPP_S          (1 << 11)
// G-stage leaf: U A D X W R V
#define GPTE_RWX               0xDF

  .text
  .global main
main:
  // s0 = mtvec, s1 = ra for crt0
  mv    s1, ra
  la    t0, m_trap
  ori   t0, t0, 0x3                   // CLIC mode
  csrrw s0, mtvec, t0
  li    t0, -1
  srli  t0, t0, 2
  csrw  pmpaddr0, t0
  li    t0, 0x0f                      // TOR, RWX
  csrw  pmpcfg0, t0
  li    t0, CLIC_CLICCFG_REG
  li    t1, (0x4 << 0) | (0x3 << 4)   // nlbits 4, nmbits 3
  sw    t1, 0(t0)
  csrwi 0x347, 0                      // mintthresh

  // G stage: Sv39x4, the guest's 1 GiB at 0x80000000 identity-mapped
  li    t0, (0x80000000 >> 12) << 10 | GPTE_RWX
  la    t1, g_root
  sd    t0, 2*8(t1)
  fence
  la    t0, g_root
  srli  t0, t0, 12
  li    t1, 8 << 60
  or    t0, t0, t1
  csrw  CSR_HGATP, t0
  .word 0x62000073                    // hfence.gvma zero, zero
  csrw  CSR_VSATP, zero

  // The interrupt: S-mode, delegated to the guest VSID, pending
  li    t0, CLIC_CLICINT_REG(IRQ_ID)
  li    t1, IRQ_CFG
  sw    t1, 0(t0)
  li    t0, CLIC_CLICINTV_REG(IRQ_ID & ~3)
  li    t1, ((VSID << 2) | 1) << (8 * (IRQ_ID & 3))
  sw    t1, 0(t0)                     // clicintv, as a word like clic_vsmode
  li    t0, VSID << 12                // hstatus.VGEIN
  csrs  CSR_HSTATUS, t0
  li    t0, CLIC_CLICINT_REG(IRQ_ID)
  li    t1, IRQ_CFG | CLICINT_IP
  sw    t1, 0(t0)

  // Enter the guest
  la    t0, guest
  csrw  mepc, t0
  li    t0, MSTATUS_MPP
  csrc  mstatus, t0
  li    t0, MSTATUS_MPV | MSTATUS_MPP_S
  csrs  mstatus, t0
  mret

guest:
  la    t0, guest_fail
  ori   t0, t0, 0x3
  csrw  stvec, t0                     // vstvec, CLIC mode
  li    t0, VSTVT_GPA
  csrw  CSR_STVT, t0                  // vstvt
  csrsi sstatus, 0x2                  // vsstatus.SIE: the interrupt is taken
1:
  j     1b

  .balign 256
guest_fail:
  j     guest_fail                    // the table read must not succeed

  .balign 256                         // the mtvec base in CLIC mode
m_trap:
  csrr  a1, mcause
  li    t0, 0x8000000000000fff
  and   t1, a1, t0
  li    a0, 2
  li    t0, CAUSE_LOAD_GUEST_PF
  bne   t1, t0, done
  csrr  a1, mtval
  li    a0, 3
  li    t0, ENTRY_GPA
  bne   a1, t0, done
  csrr  a1, CSR_MTVAL2
  li    a0, 4
  li    t0, ENTRY_GPA >> 2
  bne   a1, t0, done
  csrr  a1, mstatus
  li    a0, 5
  li    t0, MSTATUS_GVA
  and   t1, a1, t0
  beqz  t1, done
  li    a0, 0
done:
  li    t0, MSTATUS_MPV
  csrc  mstatus, t0
  li    t0, CLIC_CLICINT_REG(IRQ_ID)
  sw    zero, 0(t0)
  csrw  CSR_HGATP, zero
  csrw  mtvec, s0
  mv    ra, s1
  ret

  .data
  .balign 16384                       // the Sv39x4 root: 2048 entries
g_root:
  .zero 16384
