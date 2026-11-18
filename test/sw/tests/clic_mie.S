// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// CLIC interrupts wait for mstatus.MIE in M-mode.
//
// A pending interrupt above the threshold must not be taken while mstatus.MIE
// is clear: neither in M-mode code that cleared it, nor in a handler, which is
// entered with MIE clear, when a higher-level interrupt becomes pending. Both
// are taken once MIE is set again.
//
// Return codes: 0 pass, 2 taken with MIE clear, 3 not taken once MIE is set,
// 4 handler preempted with MIE clear, 5 preempting interrupt not taken after
// mret, 6 unexpected exception (a1 = mcause, a2 = mepc).

#define CLIC_BASE              0x10040000
#define CLIC_CLICCFG_REG       (CLIC_BASE + 0x0)
#define CLIC_CLICINT_REG(id)   (CLIC_BASE + 0x1000 + 0x4 * (id))
#define CLICINT_IP             (1 << 0)
#define CLICINT_IE             (1 << 8)
#define CLICINT_SHV            (1 << 16)
#define CLICINT_TRIG_EDGE      (1 << 17)
#define CLICINT_CTL(l)         ((l) << 24)

#define IRQ_LO                 9    // tied-off sources, raised by software
#define IRQ_HI                 10
#define CFG_LO                 (CLICINT_IE | CLICINT_SHV | CLICINT_TRIG_EDGE | CLICINT_CTL(0x5f))
#define CFG_HI                 (CLICINT_IE | CLICINT_SHV | CLICINT_TRIG_EDGE | CLICINT_CTL(0xaf))
#define WAIT                   200

  .text
  .global main
main:
  // Keep ra/s0/s1 for the return to crt0; s5, s6, s9-s11 belong to the handlers.
  mv    s1, ra
  li    s10, 0
  li    s11, 0
  li    s9, 0
  csrci mstatus, 0x8

  // CLIC mode: mtvec.mode = 3, exceptions go to the mtvec base
  la    t0, trap_fail
  ori   t0, t0, 0x3
  csrrw s0, mtvec, t0
  la    t0, vector_table
  csrw  0x307, t0                     // mtvt

  // 4 level bits, threshold 0
  li    t0, CLIC_CLICCFG_REG
  li    t1, 0x4 << 1
  sw    t1, 0(t0)
  li    t0, CLIC_CLICINT_REG(IRQ_HI)
  li    t1, CFG_HI
  sw    t1, 0(t0)
  csrwi 0x347, 0                      // mintthresh

  // Raise IRQ_LO with MIE clear: it must stay pending
  li    t0, CLIC_CLICINT_REG(IRQ_LO)
  li    t1, CFG_LO | CLICINT_IP
  sw    t1, 0(t0)
  li    t0, WAIT
1:
  addi  t0, t0, -1
  bnez  t0, 1b
  li    a0, 2
  bnez  s10, done

  // Set MIE: IRQ_LO is taken, and its handler raises IRQ_HI
  csrsi mstatus, 0x8
  li    t0, WAIT
2:
  addi  t0, t0, -1
  bnez  t0, 2b
  csrci mstatus, 0x8
  li    t0, 1
  li    a0, 3
  bne   s10, t0, done
  li    a0, 4
  bnez  s9, done
  li    a0, 5
  bne   s11, t0, done
  li    a0, 0
done:
  csrw  mtvec, s0
  mv    ra, s1
  ret

  // IRQ_LO handler, entered with MIE clear: IRQ_HI, raised here, must wait for
  // the mret. s9 records whether it ran before.
  .balign 64
irq_lo_handler:
  addi  s10, s10, 1
  li    s5, CLIC_CLICINT_REG(IRQ_HI)
  li    s6, CFG_HI | CLICINT_IP
  sw    s6, 0(s5)
  li    s5, WAIT
1:
  addi  s5, s5, -1
  bnez  s5, 1b
  mv    s9, s11
  mret

  .balign 64
irq_hi_handler:
  addi  s11, s11, 1
  mret

  .balign 256                         // the mtvec base in CLIC mode
trap_fail:
  csrr  a1, mcause
  csrr  a2, mepc
  li    a0, 6
  j     done

  .data
  .balign 2048                        // the vector table is aligned to its size
vector_table:
  .rept IRQ_LO
  .dword trap_fail
  .endr
  .dword irq_lo_handler
  .dword irq_hi_handler
