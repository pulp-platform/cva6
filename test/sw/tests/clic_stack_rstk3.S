// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// CLIC interrupt with the reserved stacking configuration (rstk = 3).
//
// rstk 3 is reserved for the FP registers and stacks nothing yet. The task
// context and the trap frame areas are filled with a pattern; after an
// interrupt with rstk 3, its handler must have run and both areas must be
// unchanged.
//
// Return codes: 0 pass, 2 task context area changed (a1 = offset,
// a2 = value), 3 trap frame area changed (a1 = offset, a2 = value),
// 4 unexpected exception (a1 = mcause, a2 = mepc), 5 handler did not run.

#define CLIC_BASE              0x10040000
#define CLIC_CLICCFG_REG       (CLIC_BASE + 0x0)
#define CLIC_CLICINT_REG(id)   (CLIC_BASE + 0x1000 + 0x4 * (id))
#define CLICINT_IP             (1 << 0)
#define CLICINT_IE             (1 << 8)
#define CLICINT_SHV            (1 << 16)
#define CLICINT_TRIG_EDGE      (1 << 17)
#define CLICINT_RSTK(r)        ((r) << 19)
#define CLICINT_CTL(l)         ((l) << 24)

#define IRQ_ID                 9
#define IRQ_CFG                (CLICINT_IP | CLICINT_IE | CLICINT_SHV | CLICINT_TRIG_EDGE | CLICINT_RSTK(3) | CLICINT_CTL(0xff))
#define CSR_MTVT               0x307
#define CSR_MTFA               0x308
#define CSR_MTCA               0x309
#define TCA_AREA               0x81300000
#define TFA_AREA               0x81310000
#define AREA_WORDS             64           // beyond a full frame
#define PATTERN                0x5a5a000000000000

  .text
  .global main
main:
  // Keep ra/s0/s1 for the return to crt0; s9 is set by the handler.
  mv    s1, ra
  la    t0, trap_fail
  ori   t0, t0, 0x3
  csrrw s0, mtvec, t0
  la    t0, vector_table
  csrw  CSR_MTVT, t0
  li    t0, CLIC_CLICCFG_REG
  li    t1, 0x4 << 1
  sw    t1, 0(t0)
  csrwi 0x347, 0                      // mintthresh

  // Fill both areas with PATTERN + offset
  li    a0, TCA_AREA
  jal   ra, fill
  li    a0, TFA_AREA
  jal   ra, fill
  li    t0, TCA_AREA
  csrw  CSR_MTCA, t0
  li    t0, TFA_AREA
  csrw  CSR_MTFA, t0

  li    s9, 0
  csrsi mstatus, 0x8
  li    t0, CLIC_CLICINT_REG(IRQ_ID)
  li    t1, IRQ_CFG
  sw    t1, 0(t0)                     // raise the interrupt
  li    t0, 200
1:
  addi  t0, t0, -1
  bnez  t0, 1b
  csrci mstatus, 0x8
  li    a0, 5
  beqz  s9, done

  li    a0, TCA_AREA
  li    a3, 2
  jal   ra, check
  bnez  a0, done
  li    a0, TFA_AREA
  li    a3, 3
  jal   ra, check
done:
  csrw  mtvec, s0
  mv    ra, s1
  ret

  // Fill AREA_WORDS words at a0 with PATTERN + offset
fill:
  li    t0, PATTERN
  li    t1, 0
2:
  add   t2, a0, t1
  add   t3, t0, t1
  sd    t3, 0(t2)
  addi  t1, t1, 8
  li    t3, AREA_WORDS * 8
  blt   t1, t3, 2b
  ret

  // Check AREA_WORDS words at a0; on a mismatch return a3 with a1 = offset,
  // a2 = value, otherwise 0
check:
  li    t0, PATTERN
  li    t1, 0
3:
  add   t2, a0, t1
  ld    a2, 0(t2)
  add   t3, t0, t1
  bne   a2, t3, 4f
  addi  t1, t1, 8
  li    t3, AREA_WORDS * 8
  blt   t1, t3, 3b
  li    a0, 0
  ret
4:
  mv    a1, t1
  mv    a0, a3
  ret

  // Handler: nothing is stacked, so it touches s9 only
  .balign 64
irq_handler:
  li    s9, 1
  mret

  .balign 256                         // the mtvec base in CLIC mode
trap_fail:
  csrr  a1, mcause
  csrr  a2, mepc
  li    a0, 4
  j     done

  .data
  .balign 2048                        // 256 entries of 8 bytes
vector_table:
  .rept IRQ_ID
  .dword trap_fail
  .endr
  .dword irq_handler
  .rept 256 - IRQ_ID - 1
  .dword trap_fail
  .endr
