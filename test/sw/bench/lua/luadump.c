// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Host tool: compiles a Lua source to bytecode, as luac would, built from the
// same Lua sources as the benchmarks, so that the programs load the scripts
// instead of compiling them in simulation. The x86-64 host and RV64 share
// Lua's bytecode format (little endian, 8-byte integers and doubles), which the
// loader checks.
//
//     luadump <name> <source.lua> <out.luac>
//
// The chunk is named "@<name>.lua", not after the source's path, so that the
// bytecode does not depend on where the sources are.

#include <stdio.h>
#include <stdlib.h>

#include "lauxlib.h"
#include "lua.h"

static int writer(lua_State *L, const void *p, size_t size, void *f) {
    (void)L;
    return fwrite(p, 1, size, f) != size;
}

int main(int argc, char **argv) {
    if (argc != 4) {
        fprintf(stderr, "usage: %s <name> <source.lua> <out.luac>\n", argv[0]);
        return 2;
    }
    FILE *in = fopen(argv[2], "rb");
    if (!in) {
        perror(argv[2]);
        return 1;
    }
    fseek(in, 0, SEEK_END);
    long size = ftell(in);
    rewind(in);
    char *source = malloc(size);
    if (!source || fread(source, 1, size, in) != (size_t)size) {
        perror(argv[2]);
        return 1;
    }
    fclose(in);

    lua_State *L = luaL_newstate();
    char chunkname[256];
    snprintf(chunkname, sizeof(chunkname), "@%s.lua", argv[1]);
    if (luaL_loadbufferx(L, source, size, chunkname, "t") != LUA_OK) {
        fprintf(stderr, "%s\n", lua_tostring(L, -1));
        return 1;
    }
    FILE *out = fopen(argv[3], "wb");
    if (!out || lua_dump(L, writer, out, 0) || fclose(out)) {
        perror(argv[3]);
        return 1;
    }
    return 0;
}
