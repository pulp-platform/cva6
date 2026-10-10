// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Benchmark startup. Unlike lib/crt0.S it zeroes .bss, since simulation memory
// starts out random, and turns on the FPU, since several benchmarks use it.
// The boot ROM jumps here with ra pointing at its exit routine, which writes
// main's return value to the EOC register, so ra must survive until main.

.section .text._start

.global _start
.align 4
_start:
    // The boot ROM cleared the integer registers.
    la sp, __stack_pointer$
    .option push
    .option norelax
    la gp, __global_pointer$
    .option pop

    la   t0, bench_trap_vector
    csrw mtvec, t0

    // Zero .bss: doublewords while at least 8 bytes remain, then bytes.
    la   t0, __bss_start
    la   t1, __bss_end
1:  addi t2, t0, 8
    bgtu t2, t1, 2f
    sd   zero, 0(t0)
    mv   t0, t2
    j    1b
2:  bgeu t0, t1, 3f
    sb   zero, 0(t0)
    addi t0, t0, 1
    j    2b
3:

    // FPU on (mstatus.FS = Initial), rounding mode and flags cleared, and the
    // registers zeroed so that no run depends on their reset value.
    li   t0, (1 << 13)
    csrs mstatus, t0
    csrwi fcsr, 0
    fmv.d.x f0,  zero
    fmv.d.x f1,  zero
    fmv.d.x f2,  zero
    fmv.d.x f3,  zero
    fmv.d.x f4,  zero
    fmv.d.x f5,  zero
    fmv.d.x f6,  zero
    fmv.d.x f7,  zero
    fmv.d.x f8,  zero
    fmv.d.x f9,  zero
    fmv.d.x f10, zero
    fmv.d.x f11, zero
    fmv.d.x f12, zero
    fmv.d.x f13, zero
    fmv.d.x f14, zero
    fmv.d.x f15, zero
    fmv.d.x f16, zero
    fmv.d.x f17, zero
    fmv.d.x f18, zero
    fmv.d.x f19, zero
    fmv.d.x f20, zero
    fmv.d.x f21, zero
    fmv.d.x f22, zero
    fmv.d.x f23, zero
    fmv.d.x f24, zero
    fmv.d.x f25, zero
    fmv.d.x f26, zero
    fmv.d.x f27, zero
    fmv.d.x f28, zero
    fmv.d.x f29, zero
    fmv.d.x f30, zero
    fmv.d.x f31, zero

    // main(0, NULL); it returns straight to the boot ROM.
    li   a0, 0
    li   a1, 0
    tail main

// Any trap ends the run: see bench_trap() in bench.c.
.align 4
.global bench_trap_vector
bench_trap_vector:
    la   sp, __stack_pointer$
    csrr a0, mcause
    csrr a1, mepc
    csrr a2, mtval
    j    bench_trap
