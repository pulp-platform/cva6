// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// An AMO right behind a fence.t.
//
// Each iteration stores to a cached line, so that the fence.t waits at commit
// for the store buffer, and puts an AMO right behind the fence.t, so that the
// AMO issues before the fence.t's micro-reset. The AMO must execute once per
// iteration after the reset; the test hangs if loads and stores stay held back.
//
// Return codes: 0 pass, 2 wrong AMO count (a1 = count).

#define ITER 8

  .text
  .global main
main:
  la    t2, amo_word
  sd    zero, 0(t2)
  la    t3, store_word
  ld    t0, 0(t3)                     // bring the line into the D$
  li    t1, 1
  li    t4, ITER
1:
  sd    t4, 0(t3)
  .word 0x0000000b                    // fence.t
  amoadd.d t5, t1, (t2)
  ld    t6, 0(t3)
  addi  t4, t4, -1
  bnez  t4, 1b

  ld    a1, 0(t2)
  li    a0, 2
  li    t0, ITER
  bne   a1, t0, 2f
  li    a0, 0
2:
  ret

  .data
  .balign 64
amo_word:
  .dword 0
  .balign 64
store_word:
  .dword 0
