// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// The divergence policy: the named, scoped, counted exceptions to strict
// comparison.
//
// A rule is a decision not to verify something, so every rule carries a
// reason, every firing is counted, and the end of run report states how much
// was covered by rules and how much was checked.
//
// The default is strict: a field with no matching rule is compared exactly.

#ifndef CVA6TB_COSIM_RV_POLICY_H
#define CVA6TB_COSIM_RV_POLICY_H

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace cva6tb_cosim {

// What may legitimately differ, one entry per thing a rule can be about.
enum class rv_field {
  pc,
  insn,
  rd,
  mem_addr,
  mem_rdata,
  mem_wdata,
  priv,
  trap_cause,
  trap_tval,
  csr,
  // Entry into an interrupt handler, matched only on the interrupt path. Its
  // own field so that a rule about taking an interrupt cannot also relax the
  // comparison of a synchronous trap's cause, which shares trap_cause.
  irq_entry,
};

enum class rv_action {
  compare,  // the default; listed so a rule can restore strictness in a subset
  masked,   // compare only the bits in `mask`
  copy,     // take the DUT's value into the reference before it steps
  supply,   // hand the DUT's value to the reference as an input, do not compare
  skip,     // do not execute on the reference at all; apply the DUT's effects
  inject,   // force reference state the DUT reached by means the reference lacks
  hold,     // keep the reference's value: the DUT hardwires this and ignores writes
};

const char *rv_field_name(rv_field f);
const char *rv_action_name(rv_action a);
// Returns false for an unknown name, so a misspelled rule is a configuration
// error.
bool rv_field_from_name(const std::string &s, rv_field *out);
bool rv_action_from_name(const std::string &s, rv_action *out);

// Whether a rule applies to addresses the reference has memory for. "Not RAM"
// is the honest way to name the peripherals: the address map decides what is
// backed, and a rule that had to list every device would drift from it.
enum class rv_scope { any, ram, mmio };

struct rv_rule {
  std::string name;
  // Mandatory. A rule without a reason is an ignore list entry, and the point
  // of this engine is that there is no such thing here.
  std::string why;

  rv_field field = rv_field::pc;
  rv_action action = rv_action::compare;

  // Scope. A rule applies where all of these match.
  rv_scope scope = rv_scope::any;
  uint64_t addr_lo = 0;
  uint64_t addr_hi = UINT64_MAX;
  uint32_t csr_addr = 0;        // 0 means any CSR
  uint8_t priv_mask = 0xff;     // bit per privilege level
  // Instruction encoding scope, for a rule about one class of instruction:
  // (insn & insn_mask) == insn_match. A zero mask matches everything.
  uint32_t insn_mask = 0;
  uint32_t insn_match = 0;

  uint64_t mask = UINT64_MAX;   // rv_action::masked
  uint64_t budget = 0;          // 0 means unlimited
  uint64_t hits = 0;
};

// What a lookup is scoped by.
struct rv_context {
  uint64_t addr = 0;
  uint32_t csr_addr = 0;
  uint32_t insn = 0;
  uint8_t priv = 0;
  // Whether `addr` is backed by memory in the reference. Carried here rather
  // than resolved in the engine, so the policy stays independent of the map.
  bool is_ram = true;
};

class rv_policy {
 public:
  // Returns false if the rule has no name or no reason.
  bool add(const rv_rule &r, std::string *error);

  // Parses one or more rules from a configuration string: comma separated
  // key=value pairs, several rules separated by ';'. Every key is rejected if
  // unknown and every rule needs a name, a field, an action and a reason, so a
  // misspelling is a configuration error.
  //
  //   name=misa-ro,field=rd,csr=0x301,action=masked,mask=0x...,why=hardwired
  //
  bool parse(const std::string &spec, std::string *error);

  // Rules that did not fire. A rule that never fires is either unnecessary or
  // the run never reached what it was written for, and both are worth saying.
  void report_unused(FILE *f) const;

  // The first rule that matches, or nullptr for strict comparison. Rules are
  // tried in the order they were added, so a later, narrower rule cannot be
  // shadowed by accident -- add the specific ones first.
  rv_rule *match(rv_field f, const rv_context &c);

  // A rule that has fired more often than its budget allows. That is a failure,
  // not an exception: the budget is the statement of how much was expected.
  bool over_budget(std::string *which) const;

  void report(FILE *f) const;
  bool empty() const { return rules_.empty(); }

 private:
  std::vector<rv_rule> rules_;
};

// The rules that are true of this testbench regardless of the program: the
// peripherals the reference does not model, and the counters it cannot match.
void rv_policy_add_builtins(rv_policy &p);

// A named set of rules for one known reason the two models differ. None are on
// by default: a clean run has to mean something, so every hole is opted into.
//
//   misa-ro     CVA6 hardwires misa; spike implements misa.C as writable
//   lrsc        a failing store-conditional writes no memory in the reference
//   clic        CLIC entry is injected: spike 1.1 has no CLIC model
//   cache-alias a cached store is visible through the uncached alias only once
//               the line is written back, which a cacheless model cannot show
//
// Returns false for an unknown set name.
bool rv_policy_add_set(rv_policy &p, const std::string &name, std::string *error);
const char *rv_policy_set_names();

}  // namespace cva6tb_cosim

#endif  // CVA6TB_COSIM_RV_POLICY_H
