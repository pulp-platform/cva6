// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Embench-IoT board support for the CVA6 standalone testbench. The triggers
// bracket exactly the timed benchmark() call in Embench's support/main.c.

#include <support.h>

#include "bench.h"

void initialise_board(void) {}

void __attribute__((noinline)) __attribute__((externally_visible)) start_trigger(void) {
    bench_roi_start();
}

void __attribute__((noinline)) __attribute__((externally_visible)) stop_trigger(void) {
    bench_roi_stop();
    bench_report();
}
