// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// CLIC vector tables in main memory, the data SPM and the instruction SPM.
//
// A hardware-vectored interrupt reads its handler address from the vector
// table as a data load, so the table can sit wherever loads reach:
// - in main memory, where a store to an entry takes effect for the next
//   interrupt without a fence.i;
// - in the data SPM;
// - in the instruction SPM.
// Bit 0 of an entry is ignored.
//
// Return codes: 0 pass, 2 wrong handler with the table in main memory, before
// or after a store to the entry (a1 = handler id), 3 wrong handler for an
// entry with bit 0 set (a1 = handler id), 5 wrong handler with the table in
// the data SPM (a1 = handler id), 6 wrong handler with the table in the
// instruction SPM (a1 = handler id), 4 unexpected exception (a1 = mcause,
// a2 = mepc).

#define CLIC_BASE              0x10040000
#define CLIC_CLICCFG_REG       (CLIC_BASE + 0x0)
#define CLIC_CLICINT_REG(id)   (CLIC_BASE + 0x1000 + 0x4 * (id))
#define CLICINT_IP             (1 << 0)
#define CLICINT_IE             (1 << 8)
#define CLICINT_SHV            (1 << 16)
#define CLICINT_TRIG_EDGE      (1 << 17)
#define CLICINT_CTL(l)         ((l) << 24)

#define IRQ_ID                 9
#define IRQ_CFG                (CLICINT_IP | CLICINT_IE | CLICINT_SHV | CLICINT_TRIG_EDGE | CLICINT_CTL(0xff))
#define ENTRY                  (8 * IRQ_ID)
#define CSR_MTVT               0x307
#define CSR_DCACHE_SPM_WAYS    0x5E0
#define CSR_ICACHE_SPM_WAYS    0x5E1
#define CSR_ICACHE_EN          0x7C0
#define CSR_DCACHE_EN          0x7C1
#define DSPM_BASE              0x01800000
#define ISPM_BASE              0x01A00000

// Raise the interrupt and check that the handler with id `expected` ran;
// otherwise finish with return code `code`.
.macro RAISE expected, code
  li    s9, 0
  li    t0, CLIC_CLICINT_REG(IRQ_ID)
  li    t1, IRQ_CFG
  sw    t1, 0(t0)
  li    t0, 200
1:
  addi  t0, t0, -1
  bnez  t0, 1b
  mv    a1, s9
  li    a0, \code
  li    t1, \expected
  bne   a1, t1, done
.endm

  .text
  .global main
main:
  // Keep ra/s0/s1 for the return to crt0; s9 holds the id of the last handler.
  mv    s1, ra
  la    t0, trap_fail
  ori   t0, t0, 0x3
  csrrw s0, mtvec, t0
  li    t0, CLIC_CLICCFG_REG
  li    t1, 0x4 << 1
  sw    t1, 0(t0)
  csrwi 0x347, 0                      // mintthresh
  csrsi mstatus, 0x8

  // Table in main memory; a store to the entry is seen by the next interrupt
  la    s2, vector_table
  csrw  CSR_MTVT, s2
  la    t0, handler_1
  sd    t0, ENTRY(s2)
  RAISE 1, 2
  la    t0, handler_2
  sd    t0, ENTRY(s2)
  RAISE 2, 2

  // Entry with bit 0 set
  la    t0, handler_3
  ori   t0, t0, 0x1
  sd    t0, ENTRY(s2)
  RAISE 3, 3

  // Table in the data SPM (way 0)
  csrwi CSR_DCACHE_EN, 0
  fence.i
  csrwi CSR_DCACHE_SPM_WAYS, 1
  csrwi CSR_DCACHE_EN, 1
  li    s3, DSPM_BASE
  la    t0, handler_4
  sd    t0, ENTRY(s3)
  csrw  CSR_MTVT, s3
  RAISE 4, 5
  csrwi CSR_DCACHE_EN, 0
  fence.i
  csrwi CSR_DCACHE_SPM_WAYS, 0
  csrwi CSR_DCACHE_EN, 1

  // Table in the instruction SPM (way 0)
  csrwi CSR_ICACHE_EN, 0
  fence.i
  csrwi CSR_ICACHE_SPM_WAYS, 1
  csrwi CSR_ICACHE_EN, 1
  li    s3, ISPM_BASE
  la    t0, handler_5
  sd    t0, ENTRY(s3)
  csrw  CSR_MTVT, s3
  RAISE 5, 6
  csrwi CSR_ICACHE_EN, 0
  fence.i
  csrwi CSR_ICACHE_SPM_WAYS, 0
  csrwi CSR_ICACHE_EN, 1

  li    a0, 0
done:
  csrci mstatus, 0x8
  csrw  mtvec, s0
  mv    ra, s1
  ret

  .balign 64
handler_1:
  li    s9, 1
  mret
  .balign 64
handler_2:
  li    s9, 2
  mret
  .balign 64
handler_3:
  li    s9, 3
  mret
  .balign 64
handler_4:
  li    s9, 4
  mret
  .balign 64
handler_5:
  li    s9, 5
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
  .rept 256
  .dword trap_fail
  .endr
