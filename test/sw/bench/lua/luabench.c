// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// The Lua interpreter running one are-we-fast-yet benchmark: LUA_BENCHMARK, the
// name of its module, for LUA_ITERATIONS of its inner iterations, all in the
// measured region. Each inner iteration verifies its own result.
//
// The sources are compiled to bytecode on the host and embedded (sources.S),
// and served to require through package.preload, each loaded when first
// required: the benchmark module and those it requires, before the region
// starts. Compiling them here would take millions of cycles of simulation. require searches nothing else:
// its other searchers open files, a system call here, and a module that is not
// preloaded must be an error a script can catch (som.lua probes for LuaJIT's
// table.new). The standard libraries are opened except io, os and debug, which
// need an operating system.

#include <stdint.h>
#include <time.h>

#include "lauxlib.h"
#include "lua.h"
#include "lualib.h"

#include "bench.h"
#include "printf.h"

#ifndef LUA_BENCHMARK
#error "define LUA_BENCHMARK as the name of an are-we-fast-yet module, e.g. \"deltablue\""
#endif
#ifndef LUA_ITERATIONS
#define LUA_ITERATIONS 1
#endif

void luaport_write(const char *s, size_t len) {
    for (size_t i = 0; i < len; i++)
        _putchar(s[i]);
}

void luaport_error(const char *format, const char *arg) {
    printf(format, arg);
}

// There is no clock. Lua reads one only to seed its string hashes, math.random
// and table.sort's pivots, and runs must be the same every time anyway.
time_t time(time_t *t) {
    if (t)
        *t = 0;
    return 0;
}

clock_t clock(void) {
    return 0;
}

#define SOURCES(X)                                                                                 \
    X("benchmark", benchmark)                                                                      \
    X("som", som)                                                                                  \
    X("bounce", bounce)                                                                            \
    X("cd", cd)                                                                                    \
    X("deltablue", deltablue)                                                                      \
    X("hashindextable-53", hashindextable_53)                                                      \
    X("havlak", havlak)                                                                            \
    X("json", json)                                                                                \
    X("list", list)                                                                                \
    X("mandelbrot", mandelbrot)                                                                    \
    X("mandelbrot-fn-53", mandelbrot_fn_53)                                                        \
    X("nbody", nbody)                                                                              \
    X("permute", permute)                                                                          \
    X("queens", queens)                                                                            \
    X("richards", richards)                                                                        \
    X("sieve", sieve)                                                                              \
    X("storage", storage)                                                                          \
    X("towers", towers)

#define DECLARE(name, symbol) extern const char lua_##symbol[], lua_##symbol##_end[];
SOURCES(DECLARE)

struct module {
    const char *name;
    const char *chunk, *end;
};

#define ENTRY(name, symbol) {name, lua_##symbol, lua_##symbol##_end},
static const struct module modules[] = {SOURCES(ENTRY)};

// A package.preload loader: load the module's bytecode and run it, as require
// would a file it found.
static int load_module(lua_State *L) {
    const struct module *m = lua_touserdata(L, lua_upvalueindex(1));
    if (luaL_loadbufferx(L, m->chunk, m->end - m->chunk, m->name, "b") != LUA_OK)
        return lua_error(L);
    lua_pushvalue(L, 1);
    lua_call(L, 1, 1);
    return 1;
}

static const luaL_Reg libraries[] = {
    {LUA_GNAME, luaopen_base},        {LUA_LOADLIBNAME, luaopen_package},
    {LUA_COLIBNAME, luaopen_coroutine}, {LUA_TABLIBNAME, luaopen_table},
    {LUA_STRLIBNAME, luaopen_string}, {LUA_MATHLIBNAME, luaopen_math},
    {LUA_UTF8LIBNAME, luaopen_utf8},
};

static int error(lua_State *L) {
    printf("[LUA] %s: %s\n", LUA_BENCHMARK, lua_tostring(L, -1));
    return 1;
}

// Requires the benchmark, and returns a function running its inner iterations.
static const char driver[] = "local benchmark = require(...)\n"
                             "return function(n) return benchmark:inner_benchmark_loop(n) end\n";

int main(void) {
    lua_State *L = luaL_newstate();
    if (!L) {
        printf("[LUA] cannot create a state\n");
        return 1;
    }
    for (size_t i = 0; i < sizeof(libraries) / sizeof(libraries[0]); i++) {
        luaL_requiref(L, libraries[i].name, libraries[i].func, 1);
        lua_pop(L, 1);
    }
    lua_getglobal(L, LUA_LOADLIBNAME);
    lua_getfield(L, -1, "preload");
    for (size_t i = 0; i < sizeof(modules) / sizeof(modules[0]); i++) {
        lua_pushlightuserdata(L, (void *)&modules[i]);
        lua_pushcclosure(L, load_module, 1);
        lua_setfield(L, -2, modules[i].name);
    }
    lua_pop(L, 1);
    // package.searchers = {package.searchers[1]}, the preload searcher
    lua_createtable(L, 1, 0);
    lua_getfield(L, -2, "searchers");
    lua_rawgeti(L, -1, 1);
    lua_rawseti(L, -3, 1);
    lua_pop(L, 1);
    lua_setfield(L, -2, "searchers");
    lua_pop(L, 1);

    if (luaL_loadstring(L, driver) != LUA_OK)
        return error(L);
    lua_pushstring(L, LUA_BENCHMARK);
    if (lua_pcall(L, 1, 1, 0) != LUA_OK)
        return error(L);
    lua_pushinteger(L, LUA_ITERATIONS);

    bench_roi_start();
    int status = lua_pcall(L, 1, 1, 0);
    bench_roi_stop();

    if (status != LUA_OK)
        return error(L);
    bench_report();
    printf("[LUA] %s: %d iterations, memory in use %d KiB\n", LUA_BENCHMARK, LUA_ITERATIONS,
           lua_gc(L, LUA_GCCOUNT));
    if (!lua_toboolean(L, -1)) {
        printf("[LUA] %s: incorrect result\n", LUA_BENCHMARK);
        return 1;
    }
    printf("[LUA] results verified\n");
    return 0;
}
