// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Hardware-vectored CLIC interrupts with IDs below and above 64.
//
// A vectored interrupt fetches its handler address from the vector table entry
// mtvt + XLEN/8 * id. With 256 interrupt sources the table has 256 entries;
// the one of interrupt 100 must be used for interrupt 100, not the one of
// interrupt 36 (100 modulo 64).
//
// Return codes: 0 pass, 2 wrong handler for interrupt 9 (a1 = handler's id),
// 3 wrong handler for interrupt 100 (a1 = handler's id), 4 unexpected
// exception or table entry (a1 = mcause, a2 = mepc).

#define CLIC_BASE              0x10040000
#define CLIC_CLICCFG_REG       (CLIC_BASE + 0x0)
#define CLIC_CLICINT_REG(id)   (CLIC_BASE + 0x1000 + 0x4 * (id))
#define CLICINT_IP             (1 << 0)
#define CLICINT_IE             (1 << 8)
#define CLICINT_SHV            (1 << 16)
#define CLICINT_TRIG_EDGE      (1 << 17)
#define CLICINT_CTL(l)         ((l) << 24)
#define IRQ_CFG                (CLICINT_IP | CLICINT_IE | CLICINT_SHV | CLICINT_TRIG_EDGE | CLICINT_CTL(0xff))

  .text
  .global main
main:
  // Keep ra/s0/s1 for the return to crt0; s9 holds the id of the last handler.
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
  csrsi mstatus, 0x8

  // Interrupt 9
  li    s9, 0
  li    t0, CLIC_CLICINT_REG(9)
  li    t1, IRQ_CFG
  sw    t1, 0(t0)
  li    t0, 200
1:
  addi  t0, t0, -1
  bnez  t0, 1b
  mv    a1, s9
  li    a0, 2
  li    t1, 9
  bne   a1, t1, done

  // Interrupt 100
  li    s9, 0
  li    t0, CLIC_CLICINT_REG(100)
  li    t1, IRQ_CFG
  sw    t1, 0(t0)
  li    t0, 200
1:
  addi  t0, t0, -1
  bnez  t0, 1b
  mv    a1, s9
  li    a0, 3
  li    t1, 100
  bne   a1, t1, done
  li    a0, 0
done:
  csrci mstatus, 0x8
  csrw  mtvec, s0
  mv    ra, s1
  ret

  .balign 64
handler_9:
  li    s9, 9
  mret
  .balign 64
handler_36:
  li    s9, 36
  mret
  .balign 64
handler_100:
  li    s9, 100
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
  .rept 9
  .dword trap_fail
  .endr
  .dword handler_9                    // 9
  .rept 36 - 10
  .dword trap_fail
  .endr
  .dword handler_36                   // 36
  .rept 100 - 37
  .dword trap_fail
  .endr
  .dword handler_100                  // 100
  .rept 256 - 101
  .dword trap_fail
  .endr
