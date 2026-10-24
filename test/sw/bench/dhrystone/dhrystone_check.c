// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Runs riscv-tests' Dhrystone, whose main is renamed dhrystone_main, and checks
// the final values of its globals against those the program states they should
// have. It does not check them itself, and its main returns 0 regardless.

#include "dhrystone.h"
#include "bench.h"
#include "printf.h"

int dhrystone_main(int argc, char **argv);

extern int Int_Glob;
extern Boolean Bool_Glob;
extern char Ch_1_Glob, Ch_2_Glob;
extern int Arr_1_Glob[50];
extern int Arr_2_Glob[50][50];

static int errors;

static void expect(const char *what, long got, long want) {
    if (got != want) {
        printf("[DHRY] check failed: %s is %ld, expected %ld\n", what, got, want);
        errors++;
    }
}

int main(void) {
    dhrystone_main(0, 0);
    bench_report();
    printf("[DHRY] runs=%d\n", NUMBER_OF_RUNS);

    expect("Int_Glob", Int_Glob, 5);
    expect("Bool_Glob", Bool_Glob, 1);
    expect("Ch_1_Glob", Ch_1_Glob, 'A');
    expect("Ch_2_Glob", Ch_2_Glob, 'B');
    expect("Arr_1_Glob[8]", Arr_1_Glob[8], 7);
    expect("Arr_2_Glob[8][7]", Arr_2_Glob[8][7], NUMBER_OF_RUNS + 10);
    if (errors)
        return 1;
    printf("[DHRY] results verified\n");
    return 0;
}
