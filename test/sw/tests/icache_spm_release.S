// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// An I-cache way returning from SPM mode without the cache being disabled.
//
// A function in DRAM is called once, so that its line is cached. All I-cache
// ways are then switched to SPM with a plain CSR write, the function is
// patched in memory and fence.i is executed, which flushes the cache ways only.
// The ways are switched back to cache mode with a plain CSR write, and the
// function is called again: it must run the patched code.
//
// Return codes: 0 pass, 2 wrong result (a1 = call, a2 = result),
// 3 unexpected exception (a1 = mcause, a2 = mepc).

#define CSR_ICACHE_SPM_WAYS    0x5E1
#define LI_A0_2                0x00200513   // addi a0, zero, 2

  .text
  .global main
main:
  // s0 = mtvec, s1 = ra for crt0
  mv    s1, ra
  la    t0, trap_fail
  csrrw s0, mtvec, t0

  jal   func                          // caches the function's line
  li    a1, 1
  mv    a2, a0
  li    t0, 1
  bne   a0, t0, fail

  li    t0, -1
  csrw  CSR_ICACHE_SPM_WAYS, t0       // all implemented ways SPM, without disabling the cache
  li    t0, LI_A0_2
  la    t1, func
  sw    t0, 0(t1)                     // patch: return 2
  fence.i
  csrw  CSR_ICACHE_SPM_WAYS, zero     // back to cache mode, without disabling

  jal   func
  li    a1, 2
  mv    a2, a0
  li    t0, 2
  bne   a0, t0, fail
  li    a0, 0
  j     done
fail:
  li    a0, 2
done:
  csrw  mtvec, s0
  mv    ra, s1
  ret

  .balign 64
  .option push
  .option norvc
func:
  addi  a0, zero, 1
  ret
  .option pop

  .balign 4
trap_fail:
  csrr  a1, mcause
  csrr  a2, mepc
  li    a0, 3
  j     done
