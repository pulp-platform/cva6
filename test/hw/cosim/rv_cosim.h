// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// The lockstep loop: one reference instruction per RVFI retirement, compared as
// it happens. Every comparison goes through the divergence policy in
// rv_policy.h.

#ifndef CVA6TB_COSIM_RV_COSIM_H
#define CVA6TB_COSIM_RV_COSIM_H

#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>

#include "rv_memif.h"
#include "rv_policy.h"
#include "rv_ref_spike.h"

namespace cva6tb_cosim {

// One retirement as RVFI reports it, in the spelling the DPI hands over.
struct rv_retire {
  uint64_t cycle = 0;
  uint64_t sim_time_ns = 0;
  uint64_t order = 0;
  uint8_t commit_port = 0;
  uint8_t priv = 0;
  bool virt = false;  // rvfi.virt: tells HS from VS, which mode alone cannot
  uint64_t pc = 0;
  uint32_t insn = 0;
  uint8_t rd_addr = 0;
  uint64_t rd_wdata = 0;
  uint8_t rs1_addr = 0;
  uint64_t rs1_rdata = 0;
  uint8_t rs2_addr = 0;
  uint64_t rs2_rdata = 0;
  uint64_t mem_vaddr = 0;
  uint64_t mem_paddr = 0;
  uint32_t mem_rmask = 0;
  uint32_t mem_wmask = 0;
  uint64_t mem_wdata = 0;
  uint64_t mem_rdata = 0;
};

// The extension set of the DUT, passed as a bitmask so the RTL half stays free
// of strings and the ISA string is derived in one place. Mirrors the bits
// cva6_rvfi.sv already ORs into misa, plus the ones misa has no room for.
struct rv_isa_bits {
  static constexpr uint32_t kA = 1u << 0;
  static constexpr uint32_t kB = 1u << 1;
  static constexpr uint32_t kC = 1u << 2;
  static constexpr uint32_t kD = 1u << 3;
  static constexpr uint32_t kF = 1u << 4;
  static constexpr uint32_t kH = 1u << 5;
  static constexpr uint32_t kM = 1u << 6;
  static constexpr uint32_t kS = 1u << 7;
  static constexpr uint32_t kU = 1u << 8;
  static constexpr uint32_t kV = 1u << 9;
  static constexpr uint32_t kZcb = 1u << 10;
  static constexpr uint32_t kZcmp = 1u << 11;
  static constexpr uint32_t kZcmt = 1u << 12;
  static constexpr uint32_t kZicond = 1u << 13;
  static constexpr uint32_t kZicntr = 1u << 14;
  static constexpr uint32_t kZihpm = 1u << 15;
  static constexpr uint32_t kZkn = 1u << 16;
  static constexpr uint32_t kZicbom = 1u << 17;
  static constexpr uint32_t kSvnapot = 1u << 18;
  // Features the reference has no model for. They are carried so that enabling
  // one is stated at startup: see rv_unmodelled_features.
  static constexpr uint32_t kClic = 1u << 24;
  static constexpr uint32_t kCvxif = 1u << 25;
  static constexpr uint32_t kCustomFp = 1u << 26;
};

// Names the enabled features the reference cannot model, empty when there are
// none. Each needs rules to cover its effects, or the comparison is unsound in
// a way no field count would show.
std::string rv_unmodelled_features(uint32_t bits);

// Builds the spike ISA string and privilege string for an extension set. An
// extension the DUT does not implement makes the reference execute what the DUT
// traps on, and a missing one does the reverse, so this has to be exact.
std::string rv_isa_string(int xlen, uint32_t bits);
std::string rv_priv_string(uint32_t bits);

class rv_cosim : public rv_mmio_supplier {
 public:
  rv_cosim(int xlen, int vlen, int plen, uint32_t isa_bits);
  ~rv_cosim() override;

  // Options, by name, from the plusargs the RTL forwards. Returns false for an
  // unknown key or a bad value, which stops the simulation, so every option
  // given takes effect.
  bool configure(const std::string &key, const std::string &value);

  // Loads the images and builds the reference. Called on the first retirement,
  // once every option has arrived.
  bool start(std::string *error);

  void retire(const rv_retire &r);
  // A trap the DUT took. The reference is stepped so that it takes the same one
  // from its own state; comparing cause and tval arrives with the trap work.
  void trap(uint64_t cycle, uint8_t priv, uint64_t pc, uint32_t insn, uint64_t cause,
            uint64_t tval, bool is_interrupt);

  // An interrupt the DUT took, reported on the first instruction of its handler:
  // RVFI's trap bit covers synchronous exceptions only.
  void interrupt(uint64_t cycle, uint64_t cause);

  // A CSR the DUT wrote this cycle. Buffered until the trap or retirement it
  // belongs to has been stepped, then compared against the reference's own.
  void csr_write(uint32_t addr, uint64_t rdata, uint64_t wdata, uint64_t wmask);

  uint64_t mismatches() const { return n_mismatches_; }
  uint64_t compared() const { return n_compared_; }
  // True once the retirement budget set by the "stop_after" option is spent.
  // A generated program has no notion of being finished -- the comparison is
  // the result, not a verdict the program reports -- so a fuzzing run asks for
  // a number of instructions and stops there. Zero, the default, means run to
  // the end.
  bool budget_spent() const { return stop_after_ != 0 && n_compared_ >= stop_after_; }
  void report(FILE *f) const;

  rv_policy &policy() { return policy_; }

  // rv_mmio_supplier: the window during which the DUT's observed value stands in
  // for a device the reference does not model.
  bool load(uint64_t paddr, size_t len, uint8_t *bytes) override;
  bool store(uint64_t paddr, size_t len, const uint8_t *bytes) override;

 private:
  // Every comparison goes through here, so that adding a rule cannot miss a
  // call site. Returns true when the field agreed or a rule accounted for it.
  // `matched` receives the rule that accounted for the field, if any, so a
  // caller can act on an action that changes state.
  bool check(rv_field f, const rv_retire &r, const rv_context &ctx, uint64_t ref, uint64_t dut,
             const char *what, rv_rule **matched = nullptr);
  void mismatch(const char *field, const rv_retire &r, uint64_t expected, uint64_t got,
                const char *what);
  // Takes the DUT's value for a CSR the reference cannot match into the
  // reference before it steps, so that rd then compares naturally.
  void copy_csr_before_step(const rv_retire &r);
  // Restores a CSR the DUT hardwires, undoing a write the reference honoured and
  // the DUT ignored. Paired with copy_csr_before_step around the step.
  void hold_csr_after_step(const rv_retire &r);

  void compare_rd(const rv_retire &r, const rv_ref_effects &eff);
  void compare_mem(const rv_retire &r, const rv_ref_effects &eff);
  // The architectural effect of a trap: what the DUT left in the trap CSRs
  // against what the reference decided for itself.
  void compare_trap_csrs(const rv_retire &r);

  int xlen_ = 64;
  int vlen_ = 64;
  int plen_ = 56;
  uint32_t isa_bits_ = 0;

  std::string image_;
  std::string bootrom_;
  std::string spike_log_ = "/dev/null";
  uint64_t max_reports_ = 10;
  // 0 means "derive from the configuration"; see start().
  unsigned vaddr_bits_ = 0;
  // The DUT's own dimensions, forwarded from its configuration. At zero the
  // reference keeps its defaults, which describe a different machine: sixteen
  // PMP entries, a four-byte PMP granule, four triggers and a sixty-four byte
  // cache block.
  unsigned pmp_entries_ = 0;
  unsigned pmp_granularity_ = 0;
  unsigned trigger_count_ = 0;
  unsigned cache_block_ = 0;
  bool dims_set_ = false;
  // Set across a step when a `hold` rule applies to the CSR being written.
  bool hold_valid_ = false;
  uint32_t hold_addr_ = 0;
  uint64_t hold_value_ = 0;
  // Self test: corrupt one retirement on the way in, so that a run which
  // reports nothing can be told from a checker that reports nothing.
  bool fault_armed_ = false;
  uint64_t fault_order_ = 0;

  rv_memif mem_;
  rv_policy policy_;
  std::unique_ptr<rv_ref_spike> ref_;
  bool started_ = false;
  bool broken_ = false;

  // Set for the duration of one reference step, so an MMIO load can answer with
  // what the DUT saw for this very instruction.
  const rv_retire *cur_ = nullptr;

  // Interrupts the DUT took that the reference has not been told about yet. The
  // arrival time comes from the DUT; everything the interrupt then does --
  // delegation, vectoring, mstatus -- is the reference's own decision.
  // Set when an injected interrupt has left the reference at a PC the DUT is
  // not at, and cleared by the single retirement that follows it.
  bool pc_inject_pending_ = false;
  // Likewise for the trap CSRs of an injected entry: the reference never took
  // the trap, so mstatus.MPP/MPIE, mepc and mcause hold nothing to compare and
  // are brought over from the DUT.
  bool csr_inject_pending_ = false;
  uint64_t n_csr_absent_ = 0;
  uint64_t n_csr_seeded_ = 0;
  // Retirements per mode, indexed as rv_mode_name: U, S, debug, M, VU, VS.
  uint64_t n_mode_[6] = {0, 0, 0, 0, 0, 0};
  uint64_t n_irq_injected_ = 0;
  uint64_t n_csr_vs_aliased_ = 0;
  bool csr_seeded_ = false;

  // The trap CSRs the DUT wrote this cycle, by address. Cleared once consumed.
  struct dut_csr {
    uint32_t addr = 0;
    uint64_t wdata = 0;
  };
  static constexpr unsigned kMaxCsrs = 48;
  dut_csr csrs_[kMaxCsrs];
  unsigned n_csrs_ = 0;

  // One counter per rv_field, so the report can say what was compared rather
  // than only what disagreed.
  static constexpr unsigned kNumFields = 11;
  uint64_t n_checks_[kNumFields] = {0};

  uint64_t n_compared_ = 0;
  uint64_t stop_after_ = 0;
  uint64_t n_traps_ = 0;
  uint64_t n_interrupts_ = 0;
  uint64_t n_trap_csrs_ = 0;
  uint64_t n_mismatches_ = 0;
  uint64_t n_mmio_supplied_ = 0;
};

}  // namespace cva6tb_cosim

#endif  // CVA6TB_COSIM_RV_COSIM_H
