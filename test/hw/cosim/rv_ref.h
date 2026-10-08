// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// What the co-simulation needs from a reference model, and nothing more.
//
// Spike is the only implementation today. The interface exists so a second one
// can be added without rewriting the comparison: when the DUT and one model
// disagree, a second model is what says which of them is wrong. That is worth
// keeping the seam for even while there is only one engine behind it.

#ifndef CVA6TB_COSIM_RV_REF_H
#define CVA6TB_COSIM_RV_REF_H

#include <cstdint>

namespace cva6tb_cosim {

// One register write, as the reference reports it. `kind` matters because RVFI
// carries a 5-bit rd_addr with no register-file tag: CVA6 muxes the FP result
// onto rd_wdata, so which file was written has to come from somewhere else.
enum class rv_reg_kind : uint8_t { none, gpr, fpr, vector, csr };

struct rv_reg_write {
  rv_reg_kind kind = rv_reg_kind::none;
  uint32_t index = 0;  // register number, or CSR address when kind == csr
  uint64_t value = 0;
};

// One memory access the reference made. A read carries no value: spike records
// reads with the address and the length only (mmu.cc), so a loaded value can
// only be checked through the register it lands in.
struct rv_mem_access {
  uint64_t addr = 0;
  uint64_t value = 0;
  uint8_t bytes = 0;
  bool is_write = false;
};

// Everything one step of the reference produced. Flat, because the policy
// engine compares fields, and a field it cannot name it cannot check.
struct rv_ref_effects {
  uint64_t pc = 0;        // PC of the instruction that was executed
  uint64_t next_pc = 0;   // where the reference went afterwards
  uint32_t insn = 0;
  uint8_t priv = 0;       // state->prv
  bool virt = false;      // state->v; H makes privilege meaningless without it

  // Register writes this instruction made. More than one is possible: a CSR
  // instruction writes both a GPR and a CSR.
  static constexpr unsigned kMaxWrites = 8;
  rv_reg_write writes[kMaxWrites];
  unsigned n_writes = 0;

  // A misaligned access splits, and an AMO both reads and writes, so there is
  // more than one of these for a single instruction.
  static constexpr unsigned kMaxAccesses = 4;
  rv_mem_access accesses[kMaxAccesses];
  unsigned n_accesses = 0;

  bool trapped = false;
  uint64_t trap_cause = 0;
  uint64_t trap_tval = 0;
};

class rv_ref {
 public:
  virtual ~rv_ref() = default;

  // Executes one instruction and reports what it did. Returns false if the
  // reference could not step at all, which is a co-simulation failure rather
  // than a mismatch.
  virtual bool step(rv_ref_effects &out) = 0;

  // Advances past an instruction without executing it, for a `skip` rule: the
  // DUT's architectural effects are applied to the reference instead.
  virtual void skip(uint64_t next_pc) = 0;

  // Raises interrupts as *pending*, leaving the reference to decide delegation,
  // vectoring and the handler PC from its own state. A mask, so an interrupt
  // controller with its own numbering fits in it.
  virtual void set_pending_irq(uint64_t mask) = 0;

  // Whether the reference implements this register at all. A CSR the DUT has
  // and the reference does not cannot be compared, and reading it raises an
  // illegal instruction inside the reference, so the caller finds out first.
  virtual bool has_csr(uint32_t addr) = 0;
  virtual uint64_t get_csr(uint32_t addr) = 0;
  virtual void put_csr(uint32_t addr, uint64_t value) = 0;
  virtual uint64_t get_xpr(unsigned i) = 0;
  virtual void put_xpr(unsigned i, uint64_t value) = 0;
  virtual uint64_t get_fpr(unsigned i) = 0;
  virtual void put_fpr(unsigned i, uint64_t value) = 0;

  virtual uint64_t pc() = 0;
  // Forces the PC. Needed for an `inject` rule, where the DUT entered a handler
  // the reference cannot derive -- CLIC's hardware vectoring, for one.
  virtual void set_pc(uint64_t pc) = 0;

  virtual uint8_t priv() = 0;
  virtual bool virt() = 0;
};

}  // namespace cva6tb_cosim

#endif  // CVA6TB_COSIM_RV_REF_H
