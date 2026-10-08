// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// Spike behind the rv_ref interface.
//
// The processor is driven directly, not through sim_t: sim_t also inherits
// htif_t, which brings the front-end-server console and syscall protocol that
// this testbench has its own hardware for. processor_t only ever needs a
// simif_t, which rv_memif provides.

#ifndef CVA6TB_COSIM_RV_REF_SPIKE_H
#define CVA6TB_COSIM_RV_REF_SPIKE_H

#include <cstdio>
#include <memory>
#include <string>

#include <riscv/processor.h>

#include "rv_memif.h"
#include "rv_ref.h"

namespace cva6tb_cosim {

class rv_ref_spike : public rv_ref {
 public:
  // `isa` and `priv` are spike's own spellings, e.g. "rv64gc_zicbom" and "MSU".
  // They come from the CVA6 configuration, so that the reference implements the
  // same ISA the DUT does.
  // `log_path` is where spike writes its own commit log. Enabling commit
  // logging is not optional -- it is what populates log_reg_write, and without
  // it the reference reports no register writes at all -- and spike prints
  // through the same switch, so there is always a file. Default it to /dev/null
  // and pass a real path only when that log is wanted for debugging.
  // `vaddr_bits` is the width of a virtual address: 39 for Sv39, 32 for Sv32, 0
  // for a configuration without paging. It is set explicitly: spike sets it only
  // from a device tree, in sim_t, which this backend does not use, and at its
  // default satp refuses every mode.
  rv_ref_spike(rv_memif &mem, const std::string &isa, const std::string &priv,
               uint64_t start_pc, unsigned vaddr_bits,
               const std::string &log_path = "/dev/null");
  ~rv_ref_spike() override;

  bool step(rv_ref_effects &out) override;
  void skip(uint64_t next_pc) override;
  void set_pending_irq(uint64_t mask) override;

  bool has_csr(uint32_t addr) override;
  uint64_t get_csr(uint32_t addr) override;
  void put_csr(uint32_t addr, uint64_t value) override;
  uint64_t get_xpr(unsigned i) override;
  void put_xpr(unsigned i, uint64_t value) override;
  uint64_t get_fpr(unsigned i) override;
  void put_fpr(unsigned i, uint64_t value) override;

  uint64_t pc() override;
  void set_pc(uint64_t pc) override;
  uint8_t priv() override;
  bool virt() override;

  processor_t *proc() { return proc_.get(); }

 private:
  rv_memif &mem_;
  std::unique_ptr<processor_t> proc_;
  FILE *log_ = nullptr;
};

}  // namespace cva6tb_cosim

#endif  // CVA6TB_COSIM_RV_REF_SPIKE_H
