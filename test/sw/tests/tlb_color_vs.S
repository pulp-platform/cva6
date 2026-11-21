// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// The TLB colour CSRs are not accessible to guests.
//
// M-mode enters VS-mode without translation. The guest reads and then writes
// CUR_CLRS; both accesses must trap, and the colours must stay unchanged. The
// trap handler skips each faulting access and returns to the guest, whose
// ecall ends the test.
//
// Return codes: 0 pass, 2 not both accesses trapped (a1 = traps taken),
// 4 CUR_CLRS changed (a1 = value), 5 unexpected trap (a1 = mcause, a2 = mepc).

#define CSR_CUR_CLRS           0x5C0
#define CSR_HGATP              0x680
#define CSR_VSATP              0x280
#define CAUSE_ILLEGAL          2
#define CAUSE_VIRT_INSTR       22
#define CAUSE_ECALL_VS         10
#define MSTATUS_MPV            (1 << 39)
#define MSTATUS_MPP            (3 << 11)
#define MSTATUS_MPP_S          (1 << 11)

  .text
  .global main
main:
  // s0 = mtvec, s1 = ra for crt0, s2/s3 = causes of the two accesses,
  // s4 = number of traps taken from the guest, s5 = CUR_CLRS before
  mv    s1, ra
  la    t0, trap
  csrrw s0, mtvec, t0
  li    t0, -1
  srli  t0, t0, 2
  csrw  pmpaddr0, t0
  li    t0, 0x0f                      // TOR, RWX
  csrw  pmpcfg0, t0
  csrw  CSR_HGATP, zero
  csrw  CSR_VSATP, zero
  li    t0, 0xffff
  csrw  CSR_CUR_CLRS, t0
  csrr  s5, CSR_CUR_CLRS
  li    s2, 0
  li    s3, 0
  li    s4, 0

  la    t0, guest
  csrw  mepc, t0
  li    t0, MSTATUS_MPP
  csrc  mstatus, t0
  li    t0, MSTATUS_MPV | MSTATUS_MPP_S
  csrs  mstatus, t0
  mret

  .option push
  .option norvc                       // the trap handler skips 4-byte instructions
guest:
  csrr  t1, CSR_CUR_CLRS
  csrw  CSR_CUR_CLRS, zero
  ecall
  .option pop

  .balign 4
trap:
  csrr  t0, mcause
  li    t1, CAUSE_ECALL_VS
  beq   t0, t1, check
  li    t1, CAUSE_ILLEGAL
  beq   t0, t1, 1f
  li    t1, CAUSE_VIRT_INSTR
  beq   t0, t1, 1f
  mv    a1, t0
  csrr  a2, mepc
  li    a0, 5
  j     done
1:
  // Record the cause of the n-th access, skip it and return to the guest
  bnez  s4, 2f
  mv    s2, t0
  j     3f
2:
  mv    s3, t0
3:
  addi  s4, s4, 1
  csrr  t0, mepc
  addi  t0, t0, 4
  csrw  mepc, t0
  mret

  // The guest's ecall: in M-mode again
check:
  li    t0, MSTATUS_MPV
  csrc  mstatus, t0
  li    a0, 4
  csrr  a1, CSR_CUR_CLRS
  bne   a1, s5, done
  li    a0, 2
  mv    a1, s4
  li    t0, 2
  bne   s4, t0, done
  li    a0, 0
done:
  li    t0, 0xffff
  csrw  CSR_CUR_CLRS, t0
  csrw  mtvec, s0
  mv    ra, s1
  ret
