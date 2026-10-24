// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// The are-we-fast-yet Lua sources, compiled to bytecode on the host (luadump.c):
// every benchmark and the modules they require, each between <symbol> and
// <symbol>_end. The assembler finds them through -I.

.macro chunk symbol, file
    .global \symbol, \symbol\()_end
\symbol:
    .incbin "\file"
\symbol\()_end:
.endm

.section .rodata.lua_chunks, "a"
chunk lua_benchmark,          benchmark.luac
chunk lua_som,                som.luac
chunk lua_bounce,             bounce.luac
chunk lua_cd,                 cd.luac
chunk lua_deltablue,          deltablue.luac
chunk lua_hashindextable_53,  hashindextable-53.luac
chunk lua_havlak,             havlak.luac
chunk lua_json,               json.luac
chunk lua_list,               list.luac
chunk lua_mandelbrot,         mandelbrot.luac
chunk lua_mandelbrot_fn_53,   mandelbrot-fn-53.luac
chunk lua_nbody,              nbody.luac
chunk lua_permute,            permute.luac
chunk lua_queens,             queens.luac
chunk lua_richards,           richards.luac
chunk lua_sieve,              sieve.luac
chunk lua_storage,            storage.luac
chunk lua_towers,             towers.luac
