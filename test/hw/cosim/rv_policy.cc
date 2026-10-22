// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>

#include "rv_policy.h"

#include <cinttypes>
#include <cstdlib>

#include "rv_memif.h"

namespace cva6tb_cosim {

namespace {

struct name_entry {
  const char *name;
  rv_field field;
};

const name_entry kFields[] = {
    {"pc", rv_field::pc},
    {"insn", rv_field::insn},
    {"rd", rv_field::rd},
    {"mem_addr", rv_field::mem_addr},
    {"mem_rdata", rv_field::mem_rdata},
    {"mem_wdata", rv_field::mem_wdata},
    {"priv", rv_field::priv},
    {"trap_cause", rv_field::trap_cause},
    {"trap_tval", rv_field::trap_tval},
    {"csr", rv_field::csr},
    {"irq_entry", rv_field::irq_entry},
};

struct action_entry {
  const char *name;
  rv_action action;
};

const action_entry kActions[] = {
    {"compare", rv_action::compare}, {"masked", rv_action::masked},
    {"copy", rv_action::copy},       {"supply", rv_action::supply},
    {"skip", rv_action::skip},       {"inject", rv_action::inject},
    {"hold", rv_action::hold},
};

}  // namespace

const char *rv_field_name(rv_field f) {
  for (const auto &e : kFields) {
    if (e.field == f) return e.name;
  }
  return "?";
}

const char *rv_action_name(rv_action a) {
  for (const auto &e : kActions) {
    if (e.action == a) return e.name;
  }
  return "?";
}

bool rv_field_from_name(const std::string &s, rv_field *out) {
  for (const auto &e : kFields) {
    if (s == e.name) {
      *out = e.field;
      return true;
    }
  }
  return false;
}

bool rv_action_from_name(const std::string &s, rv_action *out) {
  for (const auto &e : kActions) {
    if (s == e.name) {
      *out = e.action;
      return true;
    }
  }
  return false;
}

bool rv_policy::add(const rv_rule &r, std::string *error) {
  if (r.name.empty()) {
    if (error != nullptr) *error = "a rule needs a name";
    return false;
  }
  if (r.why.empty()) {
    if (error != nullptr) {
      *error = "rule '" + r.name + "' has no reason; a rule without one is an ignore list entry";
    }
    return false;
  }
  // A rule that selects the same things as one already added never fires:
  // match() returns the first rule that fits, so only the first ever applies.
  // Two such rules are therefore an error naming both. tvec and clic, which both
  // claim mtvec, for the vectored base alignment and for CLIC mode, are the pair
  // this keeps apart.
  for (const auto &e : rules_) {
    if (e.field != r.field) continue;
    if (e.csr_addr != r.csr_addr) continue;
    if (e.scope != r.scope) continue;
    if (e.insn_mask != r.insn_mask || e.insn_match != r.insn_match) continue;
    if (e.addr_lo != r.addr_lo || e.addr_hi != r.addr_hi) continue;
    if ((e.priv_mask & r.priv_mask) == 0) continue;
    if (error != nullptr) {
      *error = "rule '" + r.name + "' selects what '" + e.name +
               "' already selects; the first would decide both, so one of them is not doing what "
               "it says. Enable one or the other, or narrow them so they do not overlap";
    }
    return false;
  }
  rules_.push_back(r);
  return true;
}

rv_rule *rv_policy::match(rv_field f, const rv_context &c) {
  for (auto &r : rules_) {
    if (r.field != f) continue;
    if (r.scope == rv_scope::ram && !c.is_ram) continue;
    if (r.scope == rv_scope::mmio && c.is_ram) continue;
    if (c.addr < r.addr_lo || c.addr > r.addr_hi) continue;
    if (r.csr_addr != 0 && r.csr_addr != c.csr_addr) continue;
    if (r.insn_mask != 0 && (c.insn & r.insn_mask) != r.insn_match) continue;
    if ((r.priv_mask & (uint8_t)(1u << (c.priv & 3))) == 0) continue;
    return &r;
  }
  return nullptr;
}

namespace {

// Splits on a separator without dragging in a tokenizer. Empty pieces are kept
// out, so a trailing separator is not an error.
std::vector<std::string> split(const std::string &s, char sep) {
  std::vector<std::string> out;
  size_t start = 0;
  while (start <= s.size()) {
    const size_t at = s.find(sep, start);
    const std::string piece = s.substr(start, at == std::string::npos ? std::string::npos
                                                                     : at - start);
    if (!piece.empty()) out.push_back(piece);
    if (at == std::string::npos) break;
    start = at + 1;
  }
  return out;
}

bool parse_u64(const std::string &v, uint64_t *out) {
  if (v.empty()) return false;
  char *end = nullptr;
  const uint64_t n = strtoull(v.c_str(), &end, 0);
  if (end == nullptr || *end != '\0') return false;
  *out = n;
  return true;
}

}  // namespace

bool rv_policy::parse(const std::string &spec, std::string *error) {
  for (const auto &one : split(spec, ';')) {
    rv_rule r;
    bool have_field = false;
    bool have_action = false;

    for (const auto &kv : split(one, ',')) {
      const size_t eq = kv.find('=');
      if (eq == std::string::npos) {
        if (error != nullptr) *error = "'" + kv + "' is not key=value";
        return false;
      }
      const std::string key = kv.substr(0, eq);
      const std::string val = kv.substr(eq + 1);
      uint64_t n = 0;

      if (key == "name") {
        r.name = val;
      } else if (key == "why") {
        r.why = val;
      } else if (key == "field") {
        if (!rv_field_from_name(val, &r.field)) {
          if (error != nullptr) *error = "unknown field '" + val + "'";
          return false;
        }
        have_field = true;
      } else if (key == "action") {
        if (!rv_action_from_name(val, &r.action)) {
          if (error != nullptr) *error = "unknown action '" + val + "'";
          return false;
        }
        have_action = true;
      } else if (key == "scope") {
        if (val == "any") r.scope = rv_scope::any;
        else if (val == "ram") r.scope = rv_scope::ram;
        else if (val == "mmio") r.scope = rv_scope::mmio;
        else {
          if (error != nullptr) *error = "unknown scope '" + val + "' (any, ram, mmio)";
          return false;
        }
      } else if (key == "csr" && parse_u64(val, &n)) {
        r.csr_addr = (uint32_t)n;
      } else if (key == "addr_lo" && parse_u64(val, &n)) {
        r.addr_lo = n;
      } else if (key == "addr_hi" && parse_u64(val, &n)) {
        r.addr_hi = n;
      } else if (key == "mask" && parse_u64(val, &n)) {
        r.mask = n;
      } else if (key == "budget" && parse_u64(val, &n)) {
        r.budget = n;
      } else if (key == "priv_mask" && parse_u64(val, &n)) {
        r.priv_mask = (uint8_t)n;
      } else if (key == "insn_mask" && parse_u64(val, &n)) {
        r.insn_mask = (uint32_t)n;
      } else if (key == "insn_match" && parse_u64(val, &n)) {
        r.insn_match = (uint32_t)n;
      } else {
        if (error != nullptr) *error = "unknown or malformed option '" + kv + "'";
        return false;
      }
    }

    if (!have_field || !have_action) {
      if (error != nullptr) {
        *error = "rule '" + (r.name.empty() ? one : r.name) + "' needs both a field and an action";
      }
      return false;
    }
    if (!add(r, error)) return false;
  }
  return true;
}

void rv_policy::report_unused(FILE *f) const {
  bool any = false;
  for (const auto &r : rules_) {
    if (r.hits != 0) continue;
    if (!any) {
      fprintf(f, "*** [cva6tb_cosim] INFO: rules that did not fire:\n");
      any = true;
    }
    fprintf(f, "***   %-22s %-10s %s\n", r.name.c_str(), rv_field_name(r.field), r.why.c_str());
  }
}

bool rv_policy::over_budget(std::string *which) const {
  for (const auto &r : rules_) {
    if (r.budget != 0 && r.hits > r.budget) {
      if (which != nullptr) *which = r.name;
      return true;
    }
  }
  return false;
}

void rv_policy::report(FILE *f) const {
  uint64_t fired = 0;
  for (const auto &r : rules_) fired += r.hits;

  if (fired == 0) {
    fprintf(f, "*** [cva6tb_cosim] INFO: no divergence rule fired: everything was compared\n");
    return;
  }

  fprintf(f, "*** [cva6tb_cosim] INFO: coverage holes, by rule:\n");
  for (const auto &r : rules_) {
    if (r.hits == 0) continue;
    fprintf(f, "***   %-22s %-10s %-9s %8" PRIu64 "  %s\n", r.name.c_str(),
            rv_field_name(r.field), rv_action_name(r.action), r.hits, r.why.c_str());
    if (r.budget != 0 && r.hits > r.budget) {
      fprintf(f, "***   %-22s OVER BUDGET: %" PRIu64 " firings allowed, %" PRIu64 " seen\n",
              r.name.c_str(), r.budget, r.hits);
    }
  }
}

void rv_policy_add_builtins(rv_policy &p) {
  std::string error;

  // Anything that is not RAM. The reference executes the instruction either
  // way, so the effective address, the access size and the sign extension stay
  // checked; only the data byte comes from the DUT.
  {
    rv_rule r;
    r.name = "mmio";
    r.why = "the testbench peripherals are not modelled by the reference";
    r.field = rv_field::mem_rdata;
    r.scope = rv_scope::mmio;
    r.action = rv_action::supply;
    p.add(r, &error);
  }

  // fence.t is a PULP custom instruction under OpcodeCustom0 that the reference
  // does not implement, so there it raises an illegal instruction. It redirects
  // no control flow and writes no register -- the decoder zeroes rd -- which is
  // exactly what skip requires.
  {
    rv_rule r;
    r.name = "fence-t";
    r.why = "fence.t is a CVA6 custom instruction the reference does not implement";
    r.field = rv_field::insn;
    r.insn_mask = 0x7f;
    r.insn_match = 0x0b;
    r.action = rv_action::skip;
    p.add(r, &error);
  }

  // Interrupt arrival is a property of the platform, not of the core, and the
  // reference is attached to no platform: its mip holds only what this
  // co-simulation injects, at the retirement where RVFI reports the interrupt,
  // while the DUT's tracks the testbench's timer and external lines
  // continuously. The two therefore disagree about mip at almost any instant,
  // and a read of it is taken from the DUT -- the same reasoning as mmio above,
  // and built in for the same reason: no rule set makes this comparable.
  {
    rv_rule r;
    r.name = "mip-read";
    r.why = "interrupt arrival comes from the platform, which the reference is not attached to";
    r.field = rv_field::rd;
    r.csr_addr = 0x344;
    r.action = rv_action::copy;
    p.add(r, &error);
  }
  {
    rv_rule r;
    r.name = "sip-read";
    r.why = "interrupt arrival comes from the platform, which the reference is not attached to";
    r.field = rv_field::rd;
    r.csr_addr = 0x144;
    r.action = rv_action::copy;
    p.add(r, &error);
  }

  // A load's value cannot be compared against the reference at all: spike
  // records reads in log_mem_read with the address and the length but a zero
  // value (mmu.cc), so there is nothing to compare to. The register the load
  // writes is compared, which covers the same ground for anything that reaches
  // architectural state.
  {
    rv_rule r;
    r.name = "load-data-via-rd";
    r.why = "spike logs no value for a read; the loaded value is checked through rd";
    r.field = rv_field::mem_rdata;
    r.scope = rv_scope::ram;
    r.action = rv_action::supply;
    p.add(r, &error);
  }

  // A cache-block operation is cache maintenance, not an architectural access:
  // it moves no data to or from a register and the reference logs nothing for
  // it. CVA6 issues one through the load/store unit, so RVFI reports it with an
  // effective address that has no counterpart in the reference. Structural, and
  // so built in: this is true of any conformant reference, not of spike in
  // particular. The mask
  // takes funct3=010 under MISC-MEM, which is cbo.inval, cbo.clean, cbo.flush
  // and Zicboz's cbo.zero; Zicbop's prefetch hints are funct3=110 and are not
  // matched.
  {
    rv_rule r;
    r.name = "cbo-no-access";
    r.why = "a cache-block operation is not an architectural memory access";
    r.field = rv_field::mem_addr;
    r.insn_mask = 0x0000707f;
    r.insn_match = 0x0000200f;
    r.action = rv_action::supply;
    p.add(r, &error);
  }

  // Counters run at different rates in the two models by definition. Copying
  // beats ignoring: an ignored counter that feeds a branch diverges control
  // flow and buries the real cause thousands of instructions upstream.
  static const struct {
    const char *name;
    uint32_t addr;
  } kCounters[] = {
      {"mcycle", 0xb00}, {"minstret", 0xb02}, {"cycle", 0xc00},
      {"time", 0xc01},   {"instret", 0xc02},
  };
  for (const auto &c : kCounters) {
    rv_rule r;
    r.name = std::string("counter-") + c.name;
    r.why = "a performance counter cannot run at the same rate in both models";
    r.field = rv_field::csr;
    r.csr_addr = c.addr;
    r.action = rv_action::copy;
    p.add(r, &error);
  }
}

const char *rv_policy_set_names() {
  return "misa-ro, lrsc, ids, deleg, endianness, tvec, tinst, irq, clic, cache-alias, hpm";
}

bool rv_policy_add_set(rv_policy &p, const std::string &name, std::string *error) {
  // Each of these is one known reason the two models differ, written down once
  // with its reason, so a run opts into it by name.
  if (name == "misa-ro") {
    // CVA6 ignores writes to misa (csr_regfile.sv: `CSR_MISA: ;`) and reads back
    // a constant; spike implements misa.C as writable. Any test that disables an
    // extension through misa then diverges -- and so does what counts as an
    // illegal instruction afterwards.
    return p.parse("name=misa-ro-read,field=rd,csr=0x301,action=copy,"
                   "why=CVA6-hardwires-misa-so-the-read-is-taken-from-the-DUT;"
                   "name=misa-ro-write,field=csr,csr=0x301,action=hold,"
                   "why=CVA6-ignores-misa-writes-while-spike-honours-them",
                   error);
  }
  if (name == "deleg") {
    // medeleg bit 0 is instruction address misaligned, which cannot occur at
    // all once C is implemented: every two-byte aligned target is legal. The
    // reference hardwires the bit to zero on that reasoning; CVA6 leaves it
    // writable. Both are allowed -- the specification says such a bit *may* be
    // read-only zero -- and delegating an exception that can never be raised
    // misreports nothing, so the bit is excluded and the rest of the register
    // still compared exactly.
    // The read is copied for the same reason the endianness read is: a masked
    // comparison relaxes the check but leaves the two models holding different
    // values, and a read then carries the difference into a general register
    // where nothing can name it. Masking a CSR and copying its read go
    // together.
    return p.parse("name=deleg-warl,field=csr,csr=0x302,action=masked,"
                   "mask=0xfffffffffff7fdfe,"
                   "why=which-causes-medeleg-permits-delegating-is-implementation-defined;"
                   "name=deleg-warl-read,field=rd,csr=0x302,action=copy,"
                   "why=the-models-hold-different-medeleg-values-so-the-read-must-not-cascade",
                   error);
  }
  if (name == "lrsc") {
    // A store-conditional that fails writes no memory, so the reference records
    // no access where RVFI reports one. SC is opcode 0x2f with funct5 0b00011.
    // funct5 and opcode only: SC.W and SC.D differ in funct3 and both apply.
    return p.parse("name=lrsc-sc,field=mem_addr,action=supply,"
                   "insn_mask=0xf800007f,insn_match=0x1800002f,"
                   "why=a-failing-store-conditional-writes-no-memory-in-the-reference;"
                   "name=lrsc-sc-wdata,field=mem_wdata,action=supply,"
                   "insn_mask=0xf800007f,insn_match=0x1800002f,"
                   "why=a-failing-store-conditional-writes-no-memory-in-the-reference",
                   error);
  }
  if (name == "ids") {
    // Identification registers name the implementation, so they differ by
    // definition: spike reports marchid 5, CVA6 reports 3. Nothing about the
    // architecture depends on them, and a test that prints them is not wrong.
    return p.parse("name=id-mvendorid,field=rd,csr=0xf11,action=copy,"
                   "why=identification-registers-name-the-implementation;"
                   "name=id-marchid,field=rd,csr=0xf12,action=copy,"
                   "why=identification-registers-name-the-implementation;"
                   "name=id-mimpid,field=rd,csr=0xf13,action=copy,"
                   "why=identification-registers-name-the-implementation",
                   error);
  }
  if (name == "irq") {
    // For running software that takes interrupts. RVFI reports an interrupt
    // only as rvfi.intr on the handler's first instruction, so the reference
    // cannot be told when the DUT took it and takes it at a different boundary.
    // This hands the entry over -- the handler PC and the trap CSRs come from
    // the DUT -- so a long workload that ticks a timer runs on past each tick.
    //
    // What that gives up is everything about *taking* the interrupt:
    // delegation, vectoring, mstatus.MPP and MPIE are no longer derived by the
    // reference and so no longer compared. What it keeps is every instruction
    // either side of the entry, which for a runtime is almost all of them.
    // Leave it off to test interrupt delivery itself, which is what the
    // architectural suite's Interrupts tests do: there the entry is the subject
    // and injecting it would check nothing.
    //
    // Scoped to irq_entry, so a synchronous trap's cause stays compared exactly.
    return p.parse("name=irq-entry,field=irq_entry,action=inject,"
                   "why=RVFI-reports-an-interrupt-only-after-the-fact-so-the-entry-comes-from-the-DUT",
                   error);
  }
  if (name == "tvec") {
    // How strictly a trap vector base must be aligned in vectored mode is
    // implementation defined: the architecture asks for four bytes and permits
    // more. CVA6 asks for 256 -- csr_regfile.sv clears mtvec[7:1] on a vectored
    // write -- where a model at the architectural minimum clears only bit 1.
    // Both are conformant and the difference is visible the moment software
    // writes a base that is not already aligned that far.
    //
    // The register comparison is masked: the mode bits and everything above the
    // 256-byte granule are still compared; only bits 7 to 2, where the two
    // disagree, are excluded. The read is copied for the usual reason: it keeps
    // the masked difference out of the general registers.
    return p.parse("name=tvec-mtvec,field=csr,csr=0x305,action=masked,"
                   "mask=0xffffffffffffff03,"
                   "why=vectored-trap-vector-alignment-above-four-bytes-is-implementation-defined;"
                   "name=tvec-stvec,field=csr,csr=0x105,action=masked,"
                   "mask=0xffffffffffffff03,"
                   "why=vectored-trap-vector-alignment-above-four-bytes-is-implementation-defined;"
                   "name=tvec-mtvec-read,field=rd,csr=0x305,action=copy,"
                   "why=the-models-align-the-vectored-base-differently-so-the-read-must-not-cascade;"
                   "name=tvec-stvec-read,field=rd,csr=0x105,action=copy,"
                   "why=the-models-align-the-vectored-base-differently-so-the-read-must-not-cascade",
                   error);
  }
  if (name == "endianness") {
    // mstatus.MBE, SBE and UBE are WARL: an implementation may provide one
    // endianness or both, and the bits are read-only zero in the first case and
    // writable in the second. Two conformant implementations therefore disagree
    // about whether a write to them sticks, which is what this is.
    //
    // The register comparison masks the three bits and keeps the other sixty
    // one exact. The read is copied: a model with only one endianness cannot
    // hold the bit at all -- a write of it goes through that model's own mstatus
    // mask, which clears it again -- so copying is what keeps the difference out
    // of the general registers and everything computed from them.
    //
    // What this gives up: neither the field nor the byte swapping it selects is
    // compared, since the other model has no second endianness to be compared
    // against.
    return p.parse("name=endianness-mstatus,field=csr,csr=0x300,action=masked,"
                   "mask=0xffffffcfffffffbf,"
                   "why=mstatus-MBE-is-WARL-and-an-implementation-may-provide-one-endianness-or-both;"
                   "name=endianness-sstatus,field=csr,csr=0x100,action=masked,"
                   "mask=0xffffffffffffffbf,"
                   "why=mstatus-UBE-is-WARL-and-an-implementation-may-provide-one-endianness-or-both;"
                   "name=endianness-mstatus-read,field=rd,csr=0x300,action=copy,"
                   "why=a-model-with-one-endianness-cannot-hold-MBE-so-the-read-must-not-cascade;"
                   "name=endianness-sstatus-read,field=rd,csr=0x100,action=copy,"
                   "why=a-model-with-one-endianness-cannot-hold-UBE-so-the-read-must-not-cascade",
                   error);
  }
  if (name == "tinst") {
    // The reference does not compute either of the two things a memory fault
    // reports under H: mmu.cc throws its page faults with tinst and tval2 both
    // zero, so mtinst and htinst read zero there, and mtval2 and htval carry no
    // guest physical address. CVA6 implements all four, and riscv-hyp-tests
    // checks them instruction by instruction -- those tests pass on the DUT
    // alone. The DUT's value is taken so a handler that reads one behaves the
    // same in both, and counted, because nothing verifies it here.
    return p.parse("name=tinst-mtinst,field=rd,csr=0x34a,action=copy,"
                   "why=the-architecture-permits-writing-zero-instead-of-a-transformed-instruction;"
                   "name=tinst-mtval2,field=rd,csr=0x34b,action=copy,"
                   "why=the-architecture-permits-writing-zero-instead-of-a-guest-physical-address;"
                   "name=tinst-htinst,field=rd,csr=0x64a,action=copy,"
                   "why=the-architecture-permits-writing-zero-instead-of-a-transformed-instruction;"
                   "name=tinst-htval,field=rd,csr=0x643,action=copy,"
                   "why=the-architecture-permits-writing-zero-instead-of-a-guest-physical-address",
                   error);
  }
  if (name == "clic") {
    // Spike has no CLIC, so the registers it adds do not exist there at all: a
    // csr access to one is an illegal instruction on the reference and an
    // ordinary instruction on the DUT. Those are stepped over, with the DUT's
    // result put into the reference, so the two stay in step through a CLIC
    // driver.
    //
    // CLIC also changes what mtvec and stvec read back -- the low six bits
    // become 3f in CLIC mode (csr_regfile.sv) -- and it vectors trap entry
    // through mtvt, which the reference cannot derive, so the entry PC is taken
    // from the DUT and counted.
    //
    // This is the largest hole the co-simulation can have, which is why it is
    // opt-in and named. Everything the CLIC driver does *between* those
    // instructions is still compared.
    static const struct { const char *name; unsigned addr; } kClicCsrs[] = {
        {"mtvt", 0x307},   {"mintstatus", 0x346}, {"mintthresh", 0x347},
        {"stvt", 0x107},   {"sintthresh", 0x147}, {"sintstatus", 0xdb1},
        {"vstvt", 0x207},  {"vsintthresh", 0x247},
    };
    for (const auto &c : kClicCsrs) {
      // A csr instruction is opcode 0x73 with the register in bits 31:20, so
      // masking those two and leaving funct3 free covers csrrw/s/c and the
      // immediate forms in one rule.
      char spec[256];
      snprintf(spec, sizeof(spec),
               "name=clic-%s,field=insn,action=skip,insn_mask=0xfff0007f,"
               "insn_match=0x%x,why=%s-does-not-exist-in-the-reference",
               c.name, (c.addr << 20) | 0x73u, c.name);
      if (!p.parse(spec, error)) return false;
    }
    // The entry PC is injected through the trap_cause rule, which is scoped to
    // the retirement after a CLIC interrupt, so the program counter check stays
    // on for every other retirement.
    return p.parse("name=clic-mtvec-write,field=csr,csr=0x305,action=masked,"
                   "mask=0xffffffffffffffc0,"
                   "why=CLIC-mode-puts-3f-in-the-low-six-bits-of-mtvec-which-the-reference-cannot-encode;"
                   "name=clic-stvec-write,field=csr,csr=0x105,action=masked,"
                   "mask=0xffffffffffffffc0,"
                   "why=CLIC-mode-puts-3f-in-the-low-six-bits-of-stvec-which-the-reference-cannot-encode;"
                   "name=clic-mtvec-read,field=rd,csr=0x305,action=copy,"
                   "why=CLIC-mode-changes-what-mtvec-reads-back;"
                   "name=clic-stvec-read,field=rd,csr=0x105,action=copy,"
                   "why=CLIC-mode-changes-what-stvec-reads-back;"
                   "name=clic-entry,field=trap_cause,action=inject,"
                   "why=CLIC-vectors-trap-entry-through-mtvt-which-the-reference-cannot-derive;"
                   // In CLIC mode xcause carries the previous privilege,
                   // interrupt-enable and interrupt level alongside the
                   // exception code, none of which the reference's xcause has.
                   "name=clic-mcause-read,field=rd,csr=0x342,action=copy,"
                   "why=CLIC-mode-packs-mpp-mpie-and-mpil-into-mcause;"
                   "name=clic-scause-read,field=rd,csr=0x142,action=copy,"
                   "why=CLIC-mode-packs-spp-spie-and-spil-into-scause;"
                   "name=clic-mcause-write,field=csr,csr=0x342,action=supply,"
                   "why=CLIC-mode-packs-mpp-mpie-and-mpil-into-mcause;"
                   "name=clic-scause-write,field=csr,csr=0x142,action=supply,"
                   "why=CLIC-mode-packs-spp-spie-and-spil-into-scause",
                   error);
  }
  if (name == "hpm") {
    // How many hpmcounters a hart implements is implementation defined -- the
    // architecture lets hpmcounter3 through hpmcounter31 be hardwired to zero,
    // and a hart that hardwires one also hardwires its enable and inhibit bits.
    // CVA6 implements six; a reference built with Zihpm implements all
    // twenty-nine and has no knob to say otherwise, so the bits above the
    // implemented count read back set on one side and clear on the other.
    //
    // Only the hpm bits are given up: CY, TM and IR stay compared, and the
    // counter-enable permission those grant -- the part that has actually been
    // wrong -- is not a value comparison at all but a gated read that traps on
    // one side and retires on the other, which no rule here touches.
    //
    // Each masked register needs a copy on its read, the pairing tvec and deleg
    // above already use. Masking alone leaves the two models holding different
    // values and the difference escapes through the GPR the readback lands in:
    // with masked-only, the first symptom here was an add.uw with different
    // operands five thousand instructions after the write.
    const char *const kRegs[] = {"0x320", "0x306", "0x106"};  // mcountinhibit, mcounteren, scounteren
    for (const char *reg : kRegs) {
      const std::string masked = std::string("name=hpm-") + reg +
                                 ",field=csr,csr=" + reg +
                                 ",action=masked,mask=0x7,"
                                 "why=the-number-of-hpmcounters-is-implementation-defined";
      if (!p.parse(masked, error)) return false;
      const std::string read = std::string("name=hpm-") + reg +
                               "-read,field=rd,csr=" + reg +
                               ",action=copy,"
                               "why=the-models-hold-different-hpm-enable-bits-so-the-read-must-not-cascade";
      if (!p.parse(read, error)) return false;
    }
    return true;
  }
  if (name == "cache-alias") {
    // A store through the cached region reaches memory only when the line is
    // written back, so the uncached alias can legitimately read something older
    // than the reference's own store. A cacheless model cannot show this.
    return p.parse("name=cache-alias,field=rd,action=supply,"
                   "addr_lo=0x40000000,addr_hi=0x4fffffff,"
                   "why=a-cacheless-reference-cannot-model-a-dirty-line-behind-the-uncached-alias",
                   error);
  }
  if (error != nullptr) {
    *error = "unknown rule set '" + name + "' (known: " + rv_policy_set_names() + ")";
  }
  return false;
}

}  // namespace cva6tb_cosim
