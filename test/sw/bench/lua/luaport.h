// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Included before every Lua source: Lua writes through stdio by default, which
// has no device behind it here; it writes to the testbench console instead.

#pragma once

#include <stddef.h>

void luaport_write(const char *s, size_t len);
void luaport_error(const char *format, const char *arg);

#define lua_writestring(s, len)        luaport_write((s), (len))
#define lua_writeline()                luaport_write("\n", 1)
#define lua_writestringerror(fmt, arg) luaport_error((fmt), (arg))
