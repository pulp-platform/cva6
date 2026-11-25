// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// An AMO to an uncacheable address right after a store to the data SPM.
//
// D-cache way 0 is configured as SPM. A word in DRAM is written back to memory,
// then the code stores to the data SPM and issues an AMO on the word through
// DRAM's uncached alias. The AMO must reach memory: it returns the old value,
// and an uncached load afterwards returns the new one.
//
// Return codes: 0 pass, 2 wrong value (a1 = check number, a2 = value),
// 3 unexpected exception (a1 = mcause, a2 = mepc).

#define DSPM_BASE              0x01800000
#define DRAM_UNCACHED_BASE     0x40000000   // DRAM's uncached alias
#define DRAM_MASK              0x3fffffff
#define CSR_DCACHE_EN          0x7C1
#define CSR_DCACHE_SPM_WAYS    0x5E0

  .text
  .global main
main:
  // s0 = mtvec, s1 = ra for crt0
  mv    s1, ra
  la    t0, trap_fail
  csrrw s0, mtvec, t0
  csrwi CSR_DCACHE_EN, 0
  fence.i
  csrwi CSR_DCACHE_SPM_WAYS, 0x1
  csrwi CSR_DCACHE_EN, 1

  la    t0, amo_word
  li    t1, 5
  sd    t1, 0(t0)
  fence.i                             // write it back to memory
  li    t2, DRAM_MASK
  and   t0, t0, t2
  li    t2, DRAM_UNCACHED_BASE
  or    s2, t0, t2                    // s2 = the word's uncached alias

  li    t0, DSPM_BASE + 0x80
  li    t1, 0x5a
  sd    t1, 0(t0)                     // the store port's last address: the data SPM
  li    t1, 1
  amoadd.d a4, t1, (s2)
  ld    a5, 0(s2)

  li    a0, 2
  li    a1, 1
  mv    a2, a4
  li    t0, 5
  bne   a4, t0, done
  li    a1, 2
  mv    a2, a5
  li    t0, 6
  bne   a5, t0, done
  li    a0, 0

done:
  csrwi CSR_DCACHE_EN, 0
  fence.i
  csrwi CSR_DCACHE_SPM_WAYS, 0
  csrwi CSR_DCACHE_EN, 1
  csrw  mtvec, s0
  mv    ra, s1
  ret

  .balign 4
trap_fail:
  csrr  a1, mcause
  csrr  a2, mepc
  li    a0, 3
  j     done

  .data
  .balign 64
amo_word:
  .dword 0
