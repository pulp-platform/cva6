// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// The DPI-C interface between cva6tb_cosim.sv and cva6tb_cosim.cc.
//
// Every argument is a scalar: no strings beyond the config channel, no
// structs, no open arrays. That is what lets the same backend work
// with Verilator, Questa, VCS and Xcelium, and it is the same shape the tracer
// uses. The retirement argument list mirrors cva6tb_tracer_retire() field for
// field, so a signal that one consumer can see the other can see too.

#ifndef CVA6TB_COSIM_DPI_H
#define CVA6TB_COSIM_DPI_H

#ifdef __cplusplus
extern "C" {
#endif

// The DUT's extension set, as a bitmask, so the RTL half stays free of string
// building and the ISA string is spelled in one place -- naming an extension the hardware lacks makes the reference execute
// what the DUT traps on, and omitting one does the reverse.
#define CVA6_COSIM_EXT_A      (1u << 0)
#define CVA6_COSIM_EXT_B      (1u << 1)
#define CVA6_COSIM_EXT_C      (1u << 2)
#define CVA6_COSIM_EXT_D      (1u << 3)
#define CVA6_COSIM_EXT_F      (1u << 4)
#define CVA6_COSIM_EXT_H      (1u << 5)
#define CVA6_COSIM_EXT_M      (1u << 6)
#define CVA6_COSIM_EXT_S      (1u << 7)
#define CVA6_COSIM_EXT_U      (1u << 8)
#define CVA6_COSIM_EXT_V      (1u << 9)
#define CVA6_COSIM_EXT_ZCB    (1u << 10)
#define CVA6_COSIM_EXT_ZCMP   (1u << 11)
#define CVA6_COSIM_EXT_ZCMT   (1u << 12)
#define CVA6_COSIM_EXT_ZICOND (1u << 13)

// Creates a co-simulation instance and returns its handle, or -1 if it could
// not be created. A -1 handle makes every other call here a no-op.
int cva6tb_cosim_open(int xlen, int vlen, int plen, int extensions);

// Generic key/value option channel, so a new option costs one plusarg in the
// RTL and one key here. Returns 0 on success and -1 for an unknown key or a bad
// value; the caller stops the simulation.
int cva6tb_cosim_config(int handle, const char *key, const char *value);

// One retired instruction, in commit order within a cycle: port 0 then port 1.
void cva6tb_cosim_retire(int handle, unsigned long long cycle, unsigned long long sim_time_ns,
                         unsigned char commit_port, unsigned long long order, unsigned char priv,
                         unsigned char virt, unsigned long long pc, unsigned int insn,
                         unsigned char rd_addr,
                         unsigned long long rd_wdata, unsigned char rs1_addr,
                         unsigned long long rs1_rdata, unsigned char rs2_addr,
                         unsigned long long rs2_rdata, unsigned long long mem_vaddr,
                         unsigned long long mem_paddr, unsigned int mem_rmask,
                         unsigned int mem_wmask, unsigned long long mem_wdata,
                         unsigned long long mem_rdata);

// A trap taken by the DUT. `cause` is in mcause format, so bit XLEN-1 marks an
// interrupt; `is_interrupt` carries that bit already extracted. `tval` does not
// come from RVFI -- see the note on trap_tval_i in cva6tb_cosim.sv.
void cva6tb_cosim_trap(int handle, unsigned long long cycle, unsigned long long sim_time_ns,
                       unsigned char priv, unsigned long long pc, unsigned int insn,
                       unsigned long long cause, unsigned long long tval,
                       unsigned char is_interrupt);

// One CSR the DUT wrote this cycle, from cva6_rvfi's rvfi_csr_o snapshot. RVFI
// proper carries no CSR state per instruction, so the trap registers -- which
// are the whole architectural effect of taking a trap -- are only checkable
// through this. Reported per register, so the interface stays scalar and adding
// one costs a line at each end.
void cva6tb_cosim_csr(int handle, unsigned int addr, unsigned long long rdata,
                      unsigned long long wdata, unsigned long long wmask);

// An interrupt the DUT took, reported on the first instruction of the handler.
// RVFI's trap bit is synchronous only -- cva6_rvfi.sv gates it on the cause's
// interrupt bit being clear -- so an interrupt is not a trap event at all: it is
// visible as rvfi.intr on the retirement that follows it. Called immediately
// before that retirement.
void cva6tb_cosim_interrupt(int handle, unsigned long long cycle, unsigned long long cause);

// Number of mismatches reported so far. The RTL polls this to fail the run at
// the instruction that diverged.
unsigned long long cva6tb_cosim_mismatches(int handle);

// Non-zero once the retirement budget set by +cosim_stop_after is spent, or 0
// when no budget was asked for. A generated program never reports being
// finished -- the comparison is the result -- so a fuzzing run asks for a
// number of instructions and the RTL ends the simulation when they are done.
unsigned char cva6tb_cosim_budget_spent(int handle);

// Writes the end of run report. Idempotent.
void cva6tb_cosim_close(int handle);

#ifdef __cplusplus
}
#endif

#endif  // CVA6TB_COSIM_DPI_H
