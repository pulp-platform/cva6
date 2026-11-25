// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Stores to the instruction SPM from code running in it.
//
// I-cache ways 1 and 2 are configured as SPM. A function copied into way 1
// stores four values into way 2 and computes a result while it runs, so its
// own fetches from the SPM alternate with the stores waiting for the SPM. It
// is called in a loop; every result and every stored value must be right.
//
// Return codes: 0 pass, 2 wrong result (a1 = iteration, a2 = result),
// 4 wrong stored value (a1 = iteration, a2 = value),
// 3 unexpected exception (a1 = mcause, a2 = mepc).

#define ISPM_BASE              0x01A00000
#define FUNC_ADDR              (ISPM_BASE + 1 * 4096 + 0x100)   // way 1
#define DATA_ADDR              (ISPM_BASE + 2 * 4096 + 0x300)   // way 2
#define CSR_ICACHE_EN          0x7C0
#define CSR_ICACHE_SPM_WAYS    0x5E1
#define ITER                   16

  .text
  .global main
main:
  // s0 = mtvec, s1 = ra for crt0, s2 = function, s3 = data, s4 = iteration
  mv    s1, ra
  la    t0, trap_fail
  csrrw s0, mtvec, t0
  csrwi CSR_ICACHE_EN, 0
  fence.i
  csrwi CSR_ICACHE_SPM_WAYS, 0x6      // ways 1 and 2 are SPM
  csrwi CSR_ICACHE_EN, 1

  // Copy the function into way 1
  la    t0, spm_func
  li    t1, FUNC_ADDR
  li    t2, 6                         // 6 dwords
1:
  ld    t3, 0(t0)
  sd    t3, 0(t1)
  addi  t0, t0, 8
  addi  t1, t1, 8
  addi  t2, t2, -1
  bnez  t2, 1b
  fence.i

  li    s2, FUNC_ADDR
  li    s3, DATA_ADDR
  li    s4, ITER
loop:
  slli  a0, s4, 8                     // the argument: iteration * 256
  mv    a1, s3
  jalr  ra, 0(s2)
  slli  t0, s4, 8
  addi  t1, t0, 13                    // the function returns its argument + 13
  mv    a1, s4
  mv    a2, a0
  li    t6, 2
  bne   a0, t1, fail
  li    t6, 4
  ld    a2, 0(s3)
  bne   a2, t0, fail
  ld    a2, 8(s3)
  addi  t1, t0, 1
  bne   a2, t1, fail
  ld    a2, 16(s3)
  addi  t1, t0, 2
  bne   a2, t1, fail
  ld    a2, 24(s3)
  addi  t1, t0, 3
  bne   a2, t1, fail
  addi  s4, s4, -1
  bnez  s4, loop
  li    a0, 0
  j     done
fail:
  mv    a0, t6
done:
  csrwi CSR_ICACHE_EN, 0
  fence.i
  csrwi CSR_ICACHE_SPM_WAYS, 0
  csrwi CSR_ICACHE_EN, 1
  csrw  mtvec, s0
  mv    ra, s1
  ret

  // Copied into the SPM: stores a0 .. a0+3 to a1[0..3], returns a0 + 13
  .balign 8
  .option push
  .option norvc
spm_func:
  sd    a0, 0(a1)
  addi  a0, a0, 1
  sd    a0, 8(a1)
  addi  a0, a0, 1
  sd    a0, 16(a1)
  addi  a0, a0, 1
  sd    a0, 24(a1)
  addi  a0, a0, 10
  ret
  nop
  nop
  nop
  .option pop

  .balign 4
trap_fail:
  csrr  a1, mcause
  csrr  a2, mepc
  li    a0, 3
  j     done
