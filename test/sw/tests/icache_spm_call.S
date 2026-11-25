// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Repeated jumps from DRAM code into a function in the instruction SPM.
//
// I-cache way 1 is configured as SPM and a short function is copied into it,
// at a line index different from the calling code's. Code in DRAM calls it in
// a loop through a trampoline whose tail jump into the SPM is the last
// instruction of a cache line; the next line is never executed, so it is never
// cached. Once the branch predictor knows the target, the jump into the SPM
// kills the sequential fetch of that line, which has missed, and the SPM fetch
// is accepted a cycle later. Two trampolines at different line indices are
// used. Every call must execute the function as copied.
//
// Return codes: 0 pass, 2 wrong sum of results (a1 = trampoline, a2 = sum),
// 3 unexpected exception (a1 = mcause, a2 = mepc).

#define ISPM_BASE              0x01A00000
#define ISPM_WAY_SIZE          4096
#define FUNC_ADDR              (ISPM_BASE + 1 * ISPM_WAY_SIZE + 0x400)   // way 1, line 0x40
#define CSR_ICACHE_EN          0x7C0
#define CSR_ICACHE_SPM_WAYS    0x5E1
#define CALLS                  64
// The function returns 2 * (a0 + 3): sum over a0 = 1..CALLS
#define EXPECTED               (2 * (CALLS * (CALLS + 1) / 2 + 3 * CALLS))

  .text
  .global main
main:
  // s0 = mtvec, s1 = ra for crt0, s2 = function, s6 = trampoline
  mv    s1, ra
  la    t0, trap_fail
  csrrw s0, mtvec, t0

  csrwi CSR_ICACHE_EN, 0
  fence.i
  csrwi CSR_ICACHE_SPM_WAYS, 0x2      // way 1 is SPM
  csrwi CSR_ICACHE_EN, 1

  // Copy the function into the SPM
  la    t0, spm_func
  li    t1, FUNC_ADDR
  ld    t2, 0(t0)
  sd    t2, 0(t1)
  ld    t2, 8(t0)
  sd    t2, 8(t1)
  fence.i

  li    s2, FUNC_ADDR
  li    a1, 1
  la    s6, tramp_a
  jal   calls
  bnez  a0, done
  li    a1, 2
  la    s6, tramp_b
  jal   calls
  bnez  a0, done
  li    a0, 0

done:
  csrwi CSR_ICACHE_EN, 0
  fence.i
  csrwi CSR_ICACHE_SPM_WAYS, 0
  csrwi CSR_ICACHE_EN, 1
  csrw  mtvec, s0
  mv    ra, s1
  ret

  // Call the function CALLS times through the trampoline at s6; returns 0, or
  // 2 with a2 = the sum
calls:
  mv    s5, ra
  li    s3, CALLS
  li    s4, 0
1:
  mv    a0, s3
  jalr  ra, 0(s6)
  add   s4, s4, a0
  addi  s3, s3, -1
  bnez  s3, 1b
  li    a0, 0
  li    t0, EXPECTED
  beq   s4, t0, 2f
  li    a0, 2
  mv    a2, s4
2:
  mv    ra, s5
  ret

  // Trampolines: the tail jump into the SPM ends a 16-byte cache line, and
  // the following line, never executed, holds illegal instructions
  .option push
  .option norvc
  .balign 64
  nop
  nop
  nop
tramp_a:
  jr    s2
  .rept 4
  .word 0
  .endr
  .balign 64
  .rept 9
  nop
  .endr
  .balign 16
  nop
  nop
  nop
tramp_b:
  jr    s2
  .rept 4
  .word 0
  .endr

  // The function copied into the SPM: returns 2 * (a0 + 3)
  .balign 16
spm_func:
  addi  a0, a0, 3
  slli  a0, a0, 1
  ret
  nop
  .option pop

  .balign 4
trap_fail:
  csrr  a1, mcause
  csrr  a2, mepc
  li    a0, 3
  j     done
