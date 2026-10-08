// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// Spike co-simulation backend: the DPI entry points and the handle table.
//
// Everything of substance lives in rv_cosim; this file is the boundary. Keeping
// it thin is what lets the same lockstep code be driven by something other than
// a simulator -- see rv_cosim_standalone.

#include "cva6tb_cosim_dpi.h"

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "rv_cosim.h"

namespace cva6tb_cosim {

// The handle is an index into this table, so several harts can co-simulate
// independently without the backend holding any global state of its own.
std::vector<rv_cosim *> g_cosims;
std::vector<bool> g_reported;

rv_cosim *get(int handle) {
  if (handle < 0 || (size_t)handle >= g_cosims.size()) return nullptr;
  return g_cosims[handle];
}

}  // namespace cva6tb_cosim

using namespace cva6tb_cosim;

int cva6tb_cosim_open(int xlen, int vlen, int plen, int extensions) {
  rv_cosim *c = new rv_cosim(xlen, vlen, plen, (uint32_t)extensions);
  g_cosims.push_back(c);
  g_reported.push_back(false);
  return (int)g_cosims.size() - 1;
}

int cva6tb_cosim_config(int handle, const char *key, const char *value) {
  rv_cosim *c = get(handle);
  if (c == nullptr || key == nullptr) return -1;
  // configure() has already said what was wrong with a known option, so this
  // only reports that the option was refused, not why.
  if (!c->configure(key, value != nullptr ? value : "")) {
    fprintf(stderr, "*** [cva6tb_cosim] ERROR: option '%s' rejected\n", key);
    return -1;
  }
  return 0;
}

void cva6tb_cosim_retire(int handle, unsigned long long cycle, unsigned long long sim_time_ns,
                         unsigned char commit_port, unsigned long long order, unsigned char priv,
                         unsigned char virt, unsigned long long pc, unsigned int insn,
                         unsigned char rd_addr,
                         unsigned long long rd_wdata, unsigned char rs1_addr,
                         unsigned long long rs1_rdata, unsigned char rs2_addr,
                         unsigned long long rs2_rdata, unsigned long long mem_vaddr,
                         unsigned long long mem_paddr, unsigned int mem_rmask,
                         unsigned int mem_wmask, unsigned long long mem_wdata,
                         unsigned long long mem_rdata) {
  rv_cosim *c = get(handle);
  if (c == nullptr) return;

  rv_retire r;
  r.cycle = cycle;
  r.sim_time_ns = sim_time_ns;
  r.commit_port = commit_port;
  r.order = order;
  r.virt = virt != 0;
  r.priv = priv;
  r.pc = pc;
  r.insn = insn;
  r.rd_addr = rd_addr;
  r.rd_wdata = rd_wdata;
  r.rs1_addr = rs1_addr;
  r.rs1_rdata = rs1_rdata;
  r.rs2_addr = rs2_addr;
  r.rs2_rdata = rs2_rdata;
  r.mem_vaddr = mem_vaddr;
  r.mem_paddr = mem_paddr;
  r.mem_rmask = mem_rmask;
  r.mem_wmask = mem_wmask;
  r.mem_wdata = mem_wdata;
  r.mem_rdata = mem_rdata;

  c->retire(r);
}

void cva6tb_cosim_trap(int handle, unsigned long long cycle, unsigned long long sim_time_ns,
                       unsigned char priv, unsigned long long pc, unsigned int insn,
                       unsigned long long cause, unsigned long long tval,
                       unsigned char is_interrupt) {
  rv_cosim *c = get(handle);
  if (c == nullptr) return;
  (void)sim_time_ns;
  c->trap(cycle, priv, pc, insn, cause, tval, is_interrupt != 0);
}

void cva6tb_cosim_interrupt(int handle, unsigned long long cycle, unsigned long long cause) {
  rv_cosim *c = get(handle);
  if (c == nullptr) return;
  c->interrupt(cycle, cause);
}

void cva6tb_cosim_csr(int handle, unsigned int addr, unsigned long long rdata,
                      unsigned long long wdata, unsigned long long wmask) {
  rv_cosim *c = get(handle);
  if (c == nullptr) return;
  c->csr_write(addr, rdata, wdata, wmask);
}

unsigned long long cva6tb_cosim_mismatches(int handle) {
  rv_cosim *c = get(handle);
  return c != nullptr ? c->mismatches() : 0;
}

unsigned char cva6tb_cosim_budget_spent(int handle) {
  rv_cosim *c = get(handle);
  return (c != nullptr && c->budget_spent()) ? 1 : 0;
}

void cva6tb_cosim_close(int handle) {
  rv_cosim *c = get(handle);
  if (c == nullptr || g_reported[handle]) return;
  g_reported[handle] = true;
  c->report(stdout);
  fflush(stdout);
}
