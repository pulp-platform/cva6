// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// The heap for newlib's malloc: the memory past the program image, up to a
// reserve below the stack. Linked as an object by the benchmarks that allocate,
// so that it replaces libgloss's _sbrk, which would make a system call.

#include <errno.h>
#include <stddef.h>
#include <stdint.h>

extern char __base_heap[];
extern char __stack_pointer$[];

#define STACK_RESERVE (256 * 1024)

static char *brk = __base_heap;

void *_sbrk(ptrdiff_t increment) {
    char *old = brk;
    if ((uintptr_t)brk + increment > (uintptr_t)__stack_pointer$ - STACK_RESERVE) {
        errno = ENOMEM;
        return (void *)-1;
    }
    brk += increment;
    return old;
}
