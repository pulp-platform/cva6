// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>

#include "rv_cosim.h"

#include <cinttypes>
#include <cstring>

#include "../common/rv_disasm.h"

namespace cva6tb_cosim {

namespace {

// RVFI carries a compressed instruction in the low half of a 32-bit field, and
// spike reports the encoding it fetched. Comparing all 32 bits would trip over
// whatever sits in the upper half, which is not architectural state.
uint32_t insn_bits(uint32_t insn) {
  return ((insn & 0x3u) != 0x3u) ? (insn & 0xffffu) : insn;
}

std::string disasm(uint32_t insn, int xlen, uint32_t isa_bits) {
  const unsigned ext = (isa_bits & rv_isa_bits::kZcmt) ? cva6tb_rv::RV_EXT_ZCMT : 0u;
  return cva6tb_rv::rv_decode(insn, xlen, ext).text;
}

}  // namespace

// A privilege level and the virtualisation bit together name one mode, and a
// mismatch reports it by that name.
const char *rv_mode_name(uint64_t mode) {
  switch (mode) {
    case 0: return "U";
    case 1: return "HS";
    case 3: return "M";
    case 4: return "VU";
    case 5: return "VS";
    default: return "?";
  }
}

std::string rv_isa_string(int xlen, uint32_t bits) {
  // The single-letter extensions go in canonical order, then the Z extensions,
  // each prefixed with '_' as spike's parser expects.
  std::string s = (xlen == 32) ? "rv32i" : "rv64i";
  if (bits & rv_isa_bits::kM) s += "m";
  if (bits & rv_isa_bits::kA) s += "a";
  if (bits & rv_isa_bits::kF) s += "f";
  if (bits & rv_isa_bits::kD) s += "d";
  if (bits & rv_isa_bits::kC) s += "c";
  if (bits & rv_isa_bits::kV) s += "v";
  if (bits & rv_isa_bits::kH) s += "h";

  // Zicsr and Zifencei are implied by CVA6's base and are not in misa.
  s += "_zicsr_zifencei";
  // CVA6's B is Zba, Zbb, Zbs and Zbc: decoder.sv decodes clmul, clmulh and
  // clmulr alongside the rest under RVB, and the architectural tests for this
  // configuration are generated with Zbc and Zbkc declared. Zbkc is the crypto
  // subset of the same carry-less multiply, so it comes with it.
  if (bits & rv_isa_bits::kB) s += "_zba_zbb_zbs_zbc_zbkc";
  if (bits & rv_isa_bits::kZcb) s += "_zcb";
  if (bits & rv_isa_bits::kZcmp) s += "_zcmp";
  if (bits & rv_isa_bits::kZcmt) s += "_zcmt";
  if (bits & rv_isa_bits::kZicond) s += "_zicond";
  // Without these the reference has no cycle, instret or hpmcounterN CSR at
  // all and raises an illegal instruction on reads the DUT serves.
  if (bits & rv_isa_bits::kZicntr) s += "_zicntr";
  if (bits & rv_isa_bits::kZihpm) s += "_zihpm";
  // CVA6's ZKN gates AES64*/AES32*, SHA256*, SHA512*, pack, packh, brev8,
  // xperm4 and xperm8 in decoder.sv, which is Zbkb, Zbkx, Zkne, Zknd and Zknh
  // -- spike's zkn exactly, Zbkc included and already named under B above.
  if (bits & rv_isa_bits::kZkn) s += "_zkn";
  // The cache block size travels with this, through the dimensions: a cbo
  // instruction's address is checked against the block the reference was built
  // with, not the one the DUT has.
  if (bits & rv_isa_bits::kZicbom) s += "_zicbom";
  // Without this the reference reads a PTE's N bit as reserved and faults
  // where the DUT translates.
  if (bits & rv_isa_bits::kSvnapot) s += "_svnapot";
  return s;
}

std::string rv_unmodelled_features(uint32_t bits) {
  static const struct {
    uint32_t bit;
    const char *what;
  } kUnmodelled[] = {
      {rv_isa_bits::kClic, "CLIC (the clic rule set covers its entry and registers)"},
      {rv_isa_bits::kCvxif, "the CV-X-IF accelerator interface (no rules exist for it)"},
      {rv_isa_bits::kCustomFp, "the custom FP formats XF16/XF16ALT/XF8/XFVec (no rules exist)"},
  };
  std::string s;
  for (const auto &u : kUnmodelled) {
    if ((bits & u.bit) == 0) continue;
    if (!s.empty()) s += ", ";
    s += u.what;
  }
  return s;
}

std::string rv_priv_string(uint32_t bits) {
  // Spike accepts M, MU or MSU and nothing else. The hypervisor extension is
  // named by the 'h' in the ISA string, not here, and spike rejects any other
  // letter.
  std::string s = "M";
  if (bits & rv_isa_bits::kS) s += "S";
  if (bits & rv_isa_bits::kU) s += "U";
  return s;
}

rv_cosim::rv_cosim(int xlen, int vlen, int plen, uint32_t isa_bits)
    : xlen_(xlen), vlen_(vlen), plen_(plen), isa_bits_(isa_bits) {
  // First, so that a rule given on the command line is tried before them and can
  // narrow one: match() returns the first rule that fits.
  rv_policy_add_builtins(policy_);
}

rv_cosim::~rv_cosim() = default;

bool rv_cosim::configure(const std::string &key, const std::string &value) {
  if (key == "image") {
    image_ = value;
    return true;
  }
  if (key == "bootrom") {
    bootrom_ = value;
    return true;
  }
  if (key == "spike_log") {
    spike_log_ = value;
    return true;
  }
  if (key == "max_reports") {
    max_reports_ = strtoull(value.c_str(), nullptr, 0);
    return true;
  }
  if (key == "rule") {
    std::string error;
    if (!policy_.parse(value, &error)) {
      fprintf(stderr, "*** [cva6tb_cosim] ERROR: bad rule: %s\n", error.c_str());
      return false;
    }
    return true;
  }
  if (key == "rules") {
    std::string error;
    size_t start = 0;
    while (start <= value.size()) {
      const size_t at = value.find(',', start);
      const std::string one =
          value.substr(start, at == std::string::npos ? std::string::npos : at - start);
      if (!one.empty() && !rv_policy_add_set(policy_, one, &error)) {
        fprintf(stderr, "*** [cva6tb_cosim] ERROR: %s\n", error.c_str());
        return false;
      }
      if (at == std::string::npos) break;
      start = at + 1;
    }
    return true;
  }
  if (key == "vaddr_bits") {
    vaddr_bits_ = (unsigned)strtoul(value.c_str(), nullptr, 0);
    return true;
  }
  // The DUT's dimensions, as four numbers in one option so a configuration
  // cannot forward some and forget others: PMP entries, PMP granule in bytes,
  // trigger count, cache block in bytes.
  if (key == "dims") {
    unsigned a = 0, b = 0, c = 0, d = 0;
    if (sscanf(value.c_str(), "%u,%u,%u,%u", &a, &b, &c, &d) != 4) {
      fprintf(stderr,
              "*** [cva6tb_cosim] ERROR: dims wants four numbers: "
              "pmp,granule,triggers,cacheblock\n");
      return false;
    }
    pmp_entries_ = a;
    pmp_granularity_ = b;
    trigger_count_ = c;
    cache_block_ = d;
    dims_set_ = true;
    return true;
  }
  if (key == "stop_after") {
    stop_after_ = strtoull(value.c_str(), nullptr, 0);
    return true;
  }
  if (key == "inject_fault") {
    fault_order_ = strtoull(value.c_str(), nullptr, 0);
    fault_armed_ = true;
    return true;
  }
  fprintf(stderr, "*** [cva6tb_cosim] ERROR: unknown option '%s'\n", key.c_str());
  return false;
}

bool rv_cosim::start(std::string *error) {
  if (image_.empty()) {
    *error = "no image: +cosim_image= was not given";
    return false;
  }
  if (!bootrom_.empty() && !mem_.load_bin(bootrom_, rv_addr_map::kBootromBase, error)) return false;
  if (!mem_.load_hex(image_, error)) return false;

  // The DUT supplies every MMIO value, so no device model is installed here:
  // this object is the supplier, answering from the retirement being checked.
  mem_.set_mmio_supplier(this);

  const std::string isa = rv_isa_string(xlen_, isa_bits_);
  const std::string priv = rv_priv_string(isa_bits_);
  // CVA6 implements Sv39 on RV64 and Sv32 on RV32, and nothing but bare without
  // S-mode. A future configuration with a wider mode sets it through
  // +cosim_vaddr_bits.
  unsigned vaddr_bits = vaddr_bits_;
  if (vaddr_bits == 0) {
    if ((isa_bits_ & rv_isa_bits::kS) != 0) vaddr_bits = (xlen_ == 32) ? 32 : 39;
  }

  // Before the reference is built, because processor_t reads the configuration
  // in its constructor. Without these it keeps its own defaults -- sixteen PMP
  // entries where CVA6 has eight, a four-byte PMP granule where CVA6 has eight,
  // four triggers where Sdtrig is off, a sixty-four byte cache block where the
  // line is sixteen -- and the reference models a different machine.
  if (dims_set_) {
    mem_.cfg().pmpregions = pmp_entries_;
    mem_.cfg().pmpgranularity = pmp_granularity_;
    mem_.cfg().trigger_count = trigger_count_;
    mem_.cfg().cache_blocksz = cache_block_;
  } else {
    printf(
        "*** [cva6tb_cosim] WARNING: the DUT's dimensions were not forwarded; the reference "
        "keeps its own PMP, trigger and cache-block defaults\n");
  }

  ref_ = std::make_unique<rv_ref_spike>(mem_, isa, priv, rv_addr_map::kBootromBase, vaddr_bits,
                                        spike_log_);

  // Reset state RVFI does not report, seeded from the configuration. With
  // Zicbom, csr_regfile.sv leaves reset with the cache-block operation enables
  // set -- CBIE invalidating, CBCFE on -- in menvcfg, in senvcfg with S-mode and
  // in henvcfg with H, while the reference resets them to zero; the
  // architecture leaves their reset value unspecified. Counted with the
  // registers seeded from the snapshot.
  if ((isa_bits_ & rv_isa_bits::kZicbom) != 0) {
    const uint64_t cbo_enables = (0x3ull << 4) | (0x1ull << 6);
    const struct {
      uint32_t addr;
      bool present;
    } envcfgs[] = {{0x30a, true},
                   {0x10a, (isa_bits_ & rv_isa_bits::kS) != 0},
                   {0x60a, (isa_bits_ & rv_isa_bits::kH) != 0}};
    for (const auto &e : envcfgs) {
      if (!e.present) continue;
      ref_->put_csr(e.addr, ref_->get_csr(e.addr) | cbo_enables);
      n_csr_seeded_++;
    }
  }

  printf("*** [cva6tb_cosim] INFO: reference is spike, %s / %s\n", isa.c_str(), priv.c_str());
  // Printed for the same reason as the ISA string: so the machine the reference
  // models can be read against the configuration.
  if (dims_set_) {
    printf(
        "*** [cva6tb_cosim] INFO: reference configured with %u PMP entries, %u-byte PMP "
        "granule, %u triggers, %u-byte cache block\n",
        pmp_entries_, pmp_granularity_, trigger_count_, cache_block_);
  }
  printf("*** [cva6tb_cosim] INFO: image '%s' at 0x%016" PRIx64 " (%s)\n", image_.c_str(),
         mem_.image_base(), mem_.image_in_spm() ? "SPM" : "DRAM");
  // The reference's own view of what it implements, to read against the ISA
  // string above.
  printf("*** [cva6tb_cosim] INFO: reference misa 0x%016" PRIx64 ", vaddr bits %u\n",
         ref_->get_csr(0x301), vaddr_bits);
  // Every extension of cva6_cfg_t is either in the ISA string above or named
  // here, so the startup output describes the whole DUT.
  const std::string unmodelled = rv_unmodelled_features(isa_bits_);
  if (!unmodelled.empty()) {
    printf("*** [cva6tb_cosim] INFO: the DUT has features the reference cannot model: %s\n",
           unmodelled.c_str());
  }
  fflush(stdout);

  started_ = true;
  return true;
}

void rv_cosim::mismatch(const char *field, const rv_retire &r, uint64_t expected, uint64_t got,
                        const char *what) {
  n_mismatches_++;
  if (n_mismatches_ > max_reports_) return;

  fprintf(stderr,
          "\n*** [cva6tb_cosim] MISMATCH on %s%s%s at retirement %" PRIu64 " (cycle %" PRIu64
          ", port %u)\n",
          field, (what != nullptr && what[0] != '\0') ? " " : "",
          (what != nullptr) ? what : "", r.order, r.cycle, (unsigned)r.commit_port);
  fprintf(stderr, "      dut  pc 0x%016" PRIx64 "  insn 0x%08x  %s\n", r.pc, insn_bits(r.insn),
          disasm(r.insn, xlen_, isa_bits_).c_str());
  fprintf(stderr, "      ref  expected 0x%016" PRIx64 ", dut had 0x%016" PRIx64 "\n", expected,
          got);
  if (n_mismatches_ == max_reports_) {
    fprintf(stderr, "      (further mismatches will be counted, not printed)\n");
  }
  fflush(stderr);
}

bool rv_cosim::check(rv_field f, const rv_retire &r, const rv_context &ctx, uint64_t ref,
                     uint64_t dut, const char *what, rv_rule **matched) {
  n_checks_[(unsigned)f]++;
  rv_rule *rule = policy_.match(f, ctx);
  if (matched != nullptr) *matched = rule;
  if (rule == nullptr) {
    // Strict: no rule, exact comparison.
    if (ref == dut) return true;
    mismatch(rv_field_name(f), r, ref, dut, what);
    return false;
  }

  rule->hits++;

  // A budget is the statement of how often an exception was expected. Exceeding
  // it is a failure, reported at the instruction that exceeded it.
  if (rule->budget != 0 && rule->hits > rule->budget) {
    char what[160];
    snprintf(what, sizeof(what), "(rule '%s' over budget: %llu allowed, %llu seen)",
             rule->name.c_str(), (unsigned long long)rule->budget,
             (unsigned long long)rule->hits);
    mismatch(rv_field_name(f), r, ref, dut, what);
    return false;
  }

  switch (rule->action) {
    case rv_action::compare:
      if (ref == dut) return true;
      mismatch(rv_field_name(f), r, ref, dut, what);
      return false;
    case rv_action::masked:
      if ((ref & rule->mask) == (dut & rule->mask)) return true;
      mismatch(rv_field_name(f), r, ref & rule->mask, dut & rule->mask, what);
      return false;
    case rv_action::copy:
    case rv_action::supply:
    case rv_action::skip:
    case rv_action::inject:
    case rv_action::hold:
      // The value was taken from the DUT, unchecked. The hit count above reports
      // it as a coverage hole.
      return true;
  }
  return true;
}

void rv_cosim::copy_csr_before_step(const rv_retire &r) {
  const cva6tb_rv::decoded_t d = cva6tb_rv::rv_decode(r.insn, xlen_, 0);
  if (!d.is_csr) return;

  rv_context ctx;
  ctx.csr_addr = d.csr_addr;
  ctx.priv = r.priv;
  rv_rule *rule = policy_.match(rv_field::csr, ctx);
  if (rule == nullptr) return;

  if (rule->action == rv_action::hold) {
    // Remember what the reference has now; the write is undone after the step.
    hold_valid_ = true;
    hold_addr_ = d.csr_addr;
    hold_value_ = ref_->get_csr(d.csr_addr);
    rule->hits++;
    return;
  }
  if (rule->action != rv_action::copy) return;

  // For a CSR read the DUT's value *is* rd_wdata, so putting it into the
  // reference before it steps makes rd compare naturally and leaves the
  // reference's own copy of the CSR aligned for whatever reads it next.
  if (d.dst.kind != cva6tb_rv::REG_GPR || d.dst.idx == 0) return;
  rule->hits++;
  ref_->put_csr(d.csr_addr, r.rd_wdata);
}

void rv_cosim::hold_csr_after_step(const rv_retire &r) {
  if (!hold_valid_) return;
  hold_valid_ = false;
  // The DUT ignores writes here and reads back a constant, so the reference must
  // not have moved either. Restoring beats comparing: a reference that disabled
  // an extension the DUT keeps goes on to trap on instructions the DUT executes.
  ref_->put_csr(hold_addr_, hold_value_);
}

void rv_cosim::compare_rd(const rv_retire &r, const rv_ref_effects &eff) {
  const cva6tb_rv::decoded_t d = cva6tb_rv::rv_decode(r.insn, xlen_, 0);

  // x0 is hardwired: RVFI reports the write, and it reaches no state.
  if (d.dst.kind == cva6tb_rv::REG_NONE) return;
  if (d.dst.kind == cva6tb_rv::REG_GPR && d.dst.idx == 0) return;

  // RVFI's rd_addr is five bits with no register-file tag, and CVA6 muxes the FP
  // result onto rd_wdata, so which file was written comes from the decoder.
  const rv_reg_kind want =
      (d.dst.kind == cva6tb_rv::REG_FPR) ? rv_reg_kind::fpr : rv_reg_kind::gpr;

  bool found = false;
  uint64_t ref_value = 0;
  for (unsigned i = 0; i < eff.n_writes; i++) {
    if (eff.writes[i].kind != want) continue;
    if (eff.writes[i].index != d.dst.idx) continue;
    ref_value = eff.writes[i].value;
    found = true;
    break;
  }

  rv_context ctx;
  ctx.priv = r.priv;
  ctx.insn = r.insn;
  // A csrr of a register the two models disagree about arrives here as an rd
  // comparison, so the address has to be in scope or no rule could name it.
  if (d.is_csr) ctx.csr_addr = d.csr_addr;
  // A load's destination register carries the address it came from, so a rule
  // about a region covers the value it produced.
  if (r.mem_rmask != 0) ctx.addr = r.mem_paddr;
  if (!found) {
    // The DUT says it wrote a register the reference did not. That is a real
    // divergence, and it is reported as one.
    check(rv_field::rd, r, ctx, 0, r.rd_wdata,
          (want == rv_reg_kind::fpr) ? "f (reference wrote nothing)"
                                     : "x (reference wrote nothing)");
    return;
  }

  char what[16];
  snprintf(what, sizeof(what), "%c%u", want == rv_reg_kind::fpr ? 'f' : 'x',
           (unsigned)d.dst.idx);
  rv_rule *rule = nullptr;
  check(rv_field::rd, r, ctx, ref_value, r.rd_wdata, what, &rule);

  // Supplying a value only declines to compare it; the reference still holds its
  // own, and everything computed from it diverges next. Copying takes the DUT's
  // value, which is what stops a known difference from becoming a cascade.
  if (rule != nullptr && rule->action == rv_action::copy) {
    if (want == rv_reg_kind::fpr) {
      ref_->put_fpr(d.dst.idx, r.rd_wdata);
    } else {
      ref_->put_xpr(d.dst.idx, r.rd_wdata);
    }
  }
}

void rv_cosim::compare_mem(const rv_retire &r, const rv_ref_effects &eff) {
  if (r.mem_rmask == 0 && r.mem_wmask == 0) return;

  rv_context ctx;
  ctx.priv = r.priv;
  ctx.addr = r.mem_paddr;
  ctx.is_ram = mem_.is_ram(r.mem_paddr);
  ctx.insn = r.insn;

  // The address first: a load or store to the wrong place is the interesting
  // failure, and it is the one the physical address probe exists to catch.
  if (eff.n_accesses == 0) {
    check(rv_field::mem_addr, r, ctx, 0, r.mem_vaddr, "(reference accessed nothing)");
    return;
  }
  check(rv_field::mem_addr, r, ctx, eff.accesses[0].addr, r.mem_vaddr, nullptr);

  if (r.mem_wmask == 0) {
    // A loaded value cannot be compared here: see the load-data-via-rd rule.
    check(rv_field::mem_rdata, r, ctx, 0, r.mem_rdata, nullptr);
    return;
  }

  for (unsigned i = 0; i < eff.n_accesses; i++) {
    if (!eff.accesses[i].is_write) continue;
    const unsigned bytes = eff.accesses[i].bytes ? eff.accesses[i].bytes : 8;
    const uint64_t mask = (bytes >= 8) ? UINT64_MAX : ((1ull << (bytes * 8)) - 1);
    check(rv_field::mem_wdata, r, ctx, eff.accesses[i].value & mask, r.mem_wdata & mask, nullptr);
    return;
  }
}

void rv_cosim::retire(const rv_retire &in) {
  if (broken_) return;

  // The self test corrupts what the DUT is said to have done, at one nominated
  // retirement, so a run shows that the checker fires.
  rv_retire r = in;
  if (fault_armed_ && r.order == fault_order_) {
    fprintf(stderr, "*** [cva6tb_cosim] INFO: injecting a fault at retirement %" PRIu64 "\n",
            r.order);
    r.pc ^= 0x10;
  }
  if (!started_) {
    std::string error;
    if (!start(&error)) {
      fprintf(stderr, "*** [cva6tb_cosim] ERROR: %s\n", error.c_str());
      broken_ = true;
      return;
    }
  }

  // An instruction the reference cannot execute at all -- a custom encoding it
  // does not implement -- is stepped over, with the DUT's effects applied in
  // its place. Only sensible for instructions that do not
  // redirect control flow: the reference is advanced by the encoding's length.
  {
    rv_context sctx;
    sctx.priv = r.priv;
    sctx.insn = r.insn;
    rv_rule *srule = policy_.match(rv_field::insn, sctx);
    if (srule != nullptr && srule->action == rv_action::skip) {
      srule->hits++;
      const cva6tb_rv::decoded_t d = cva6tb_rv::rv_decode(r.insn, xlen_, 0);
      if (d.dst.kind == cva6tb_rv::REG_GPR && d.dst.idx != 0) {
        ref_->put_xpr(d.dst.idx, r.rd_wdata);
      } else if (d.dst.kind == cva6tb_rv::REG_FPR) {
        ref_->put_fpr(d.dst.idx, r.rd_wdata);
      }
      ref_->skip(r.pc + (((r.insn & 0x3u) != 0x3u) ? 2 : 4));
      n_compared_++;
      return;
    }
  }

  rv_ref_effects eff;

  // Before stepping, so that a counter the reference cannot match is taken from
  // the DUT and both models continue from the same value.
  copy_csr_before_step(r);

  // An interrupt whose entry the reference cannot derive leaves it at a PC the
  // DUT is not at. Bring it over *before* the step, so the reference executes
  // this retirement's instruction. Doing it here scopes the injection to this
  // one retirement and keeps the program counter check strict everywhere else.
  if (pc_inject_pending_) {
    pc_inject_pending_ = false;
    ref_->set_pc(r.pc);
  }

  // The window during which an MMIO load is answered from this retirement.
  cur_ = &r;
  const bool ok = ref_->step(eff);
  cur_ = nullptr;

  hold_csr_after_step(r);

  if (!ok) {
    fprintf(stderr, "*** [cva6tb_cosim] ERROR: reference could not step at 0x%016" PRIx64 "\n",
            r.pc);
    broken_ = true;
    return;
  }

  n_compared_++;

  rv_context ctx;
  ctx.priv = r.priv;

  // The program counter first: once control flow diverges every later field is
  // noise, and the instruction that actually went wrong is the one here.
  if (!check(rv_field::pc, r, ctx, eff.pc, r.pc, nullptr)) {
    // Realign, so that later reports describe later divergences. Strictly this
    // can hide a second, independent divergence; it keeps the first one
    // readable, and the count still reflects every one of them.
    ref_->set_pc(r.pc);
    return;
  }

  if (!check(rv_field::insn, r, ctx, insn_bits(eff.insn), insn_bits(r.insn), nullptr)) return;

  // Privilege before the rest: a mode mismatch changes what everything else
  // means. The virtualisation bit is part of it -- S alone does not say whether
  // the instruction ran in HS or VS, and an S-mode CSR access means a different
  // register in each -- so the two are compared as the one mode they describe.
  {
    n_mode_[((r.virt ? 1u : 0u) << 2) | (r.priv & 3)]++;
    const uint64_t ref_mode = ((uint64_t)eff.virt << 2) | eff.priv;
    const uint64_t dut_mode = ((uint64_t)r.virt << 2) | r.priv;
    char what[32];
    snprintf(what, sizeof(what), "%s vs %s", rv_mode_name(ref_mode), rv_mode_name(dut_mode));
    check(rv_field::priv, r, ctx, ref_mode, dut_mode, what);
  }

  compare_rd(r, eff);
  compare_mem(r, eff);

  // An mret or an explicit csrw reports CSR writes on a retirement. Compare
  // them the same way, so they cannot leak into the next trap's comparison.
  compare_trap_csrs(r);
}

void rv_cosim::trap(uint64_t cycle, uint8_t priv, uint64_t pc, uint32_t insn, uint64_t cause,
                    uint64_t tval, bool is_interrupt) {
  if (broken_ || !started_) return;

  // rvfi.trap is synchronous only, so is_interrupt should never be set here;
  // an interrupt arrives through interrupt(). The warning asserts that reading
  // of cva6_rvfi.
  if (is_interrupt) {
    fprintf(stderr, "*** [cva6tb_cosim] WARNING: rvfi.trap reported an interrupt "
                    "(cause 0x%llx); interrupt handling expects rvfi.intr\n",
            (unsigned long long)cause);
  }

  // The DUT reports a trap instead of a retirement, so the reference has to be
  // stepped here too or the two fall out of step by one instruction.
  n_traps_++;
  rv_retire r;
  r.cycle = cycle;
  r.pc = pc;
  r.insn = insn;
  r.priv = priv;

  rv_ref_effects eff;
  cur_ = &r;
  const bool ok = ref_->step(eff);
  cur_ = nullptr;
  if (!ok) {
    broken_ = true;
    return;
  }

  rv_context ctx;
  ctx.priv = priv;

  if (!eff.trapped) {
    // The DUT trapped and the reference did not. That is the interesting
    // direction: an exception CVA6 raises and the architecture does not.
    check(rv_field::trap_cause, r, ctx, 0, cause, "(reference did not trap)");
  } else {
    check(rv_field::trap_cause, r, ctx, eff.trap_cause, cause, nullptr);
    // tval is out of band on rvfi_to_iti and only meaningful when the
    // configuration produces it, which is why it is its own field and can be
    // ruled out on a configuration that ties it off.
    check(rv_field::trap_tval, r, ctx, eff.trap_tval, tval, nullptr);
  }

  compare_trap_csrs(r);
}

void rv_cosim::interrupt(uint64_t cycle, uint64_t cause) {
  if (broken_ || !started_) return;
  n_interrupts_++;

  rv_retire r;
  r.cycle = cycle;
  rv_context ctx;
  // irq_entry first, so a run can give up on interrupt entry alone. That is the
  // case for a long workload -- a runtime that takes a timer tick now and then
  // -- where the interest is in the hundreds of thousands of instructions
  // between the interrupts, not in how the handler was entered. Without it the
  // run stops at the first tick, because the reference learns of an interrupt
  // only from rvfi.intr on the handler's first instruction and so takes it at a
  // different boundary than the DUT.
  //
  // trap_cause is still consulted after it, which is what the clic set matches.
  rv_rule *rule = policy_.match(rv_field::irq_entry, ctx);
  if (rule == nullptr) rule = policy_.match(rv_field::trap_cause, ctx);

  if (rule != nullptr && rule->action == rv_action::inject) {
    // The reference cannot work out where this interrupt goes -- CLIC reads a
    // vector table it knows nothing about -- so the entry comes from the DUT.
    // The instruction the interrupt aborted did not retire on either side, so
    // nothing is lost by not stepping: only the PC has to be brought over, and
    // only for the one retirement that follows.
    rule->hits++;
    n_irq_injected_++;
    pc_inject_pending_ = true;
    csr_inject_pending_ = true;
    return;
  }

  // Otherwise the reference takes the interrupt itself. Raising it pending and
  // stepping once consumes the trap without executing an instruction, which
  // leaves the reference at the handler entry *it* decided on -- so delegation,
  // vectoring and mstatus are all still compared against the DUT's.
  ref_->set_pending_irq(1ull << (cause & 63));
  rv_ref_effects eff;
  cur_ = &r;
  ref_->step(eff);
  cur_ = nullptr;
}


void rv_cosim::csr_write(uint32_t addr, uint64_t rdata, uint64_t wdata, uint64_t wmask) {
  (void)rdata;
  // A zero write mask means the register was not written this cycle. Only what
  // was actually written is compared, so an untouched register cannot fail.
  if (wmask == 0 || n_csrs_ >= kMaxCsrs) return;
  csrs_[n_csrs_].addr = addr;
  csrs_[n_csrs_].wdata = wdata;
  n_csrs_++;
}

void rv_cosim::compare_trap_csrs(const rv_retire &r) {
  rv_context ctx;
  ctx.priv = r.priv;

  // Reset values legitimately differ: CVA6 loads mtvec with boot_addr + 0x40 on
  // leaving reset (csr_regfile.sv) while the reference resets it to zero. The
  // snapshot detects that load as a change, so the first batch describes the
  // DUT's reset state. It seeds the reference, so both start from the same
  // architectural state -- the same reason the memory image is loaded into
  // both -- and each seeded register is counted as unchecked.
  if (!csr_seeded_) {
    csr_seeded_ = true;
    for (unsigned i = 0; i < n_csrs_; i++) {
      if (!ref_->has_csr(csrs_[i].addr)) {
        n_csr_absent_++;
        continue;
      }
      ref_->put_csr(csrs_[i].addr, csrs_[i].wdata);
      n_csr_seeded_++;
    }
    n_csrs_ = 0;
    return;
  }

  if (csr_inject_pending_) {
    // The reference did not take this trap, so its trap CSRs hold the state from
    // before it: there is nothing to compare them against. Write the DUT's values
    // in, so the handler's own reads and its eventual return see what the
    // DUT saw. This is the widest part of the injected-entry hole and it is
    // counted as such.
    csr_inject_pending_ = false;
    for (unsigned i = 0; i < n_csrs_; i++) {
      if (!ref_->has_csr(csrs_[i].addr)) {
        n_csr_absent_++;
        continue;
      }
      ref_->put_csr(csrs_[i].addr, csrs_[i].wdata);
      n_trap_csrs_++;
    }
    n_csrs_ = 0;
    return;
  }

  for (unsigned i = 0; i < n_csrs_; i++) {
    ctx.csr_addr = csrs_[i].addr;
    // A register the reference does not implement cannot be compared: reading
    // it raises an illegal instruction inside the reference. It is counted, so
    // the report shows the hole.
    if (!ref_->has_csr(csrs_[i].addr)) {
      n_csr_absent_++;
      continue;
    }
    // In VS mode an S-mode address names the VS register, and the reference
    // redirects the read accordingly -- but the DUT's snapshot reports the
    // HS register, which is a view of mstatus and does not move with it. The
    // two are different registers, so comparing them says nothing.
    //
    // The mode that decides this is the reference's *now*, not the retirement's:
    // the read happens after the step, so an sret executed in HS that returns
    // to VS redirects the read even though the instruction itself ran with
    // virt clear. That is precisely the case this gets wrong if it tests the
    // retirement's own bit.
    if (ref_->virt() && csrs_[i].addr >= 0x100 && csrs_[i].addr <= 0x1ff) {
      n_csr_vs_aliased_++;
      continue;
    }
    // The reference has already taken its own trap, so its CSRs hold what it
    // decided. Comparing them checks the trap *semantics* beyond the cause:
    // mepc, mcause, mtval and mstatus are the whole effect.
    const uint64_t ref_value = ref_->get_csr(csrs_[i].addr);
    char what[24];
    snprintf(what, sizeof(what), "0x%03x", csrs_[i].addr);
    rv_rule *rule = nullptr;
    check(rv_field::csr, r, ctx, ref_value, csrs_[i].wdata, what, &rule);
    // A rule that takes the DUT's value has to leave the reference holding it
    // too. Relaxing the comparison alone leaves the two models disagreeing
    // about the register from here on, and the difference then propagates --
    // into whatever reads it, and out of there into every register it reaches,
    // where no rule can name it any more because there is no CSR in sight.
    if (rule != nullptr &&
        (rule->action == rv_action::copy || rule->action == rv_action::supply)) {
      ref_->put_csr(csrs_[i].addr, csrs_[i].wdata);
    }
    n_trap_csrs_++;
  }
  n_csrs_ = 0;
}

bool rv_cosim::load(uint64_t paddr, size_t len, uint8_t *bytes) {
  // Not skipped: the reference still executes the instruction, so the effective
  // address, the access size and the sign extension all stay checked. Only the
  // data comes from the DUT, because nothing here models the device that
  // produced it.
  memset(bytes, 0, len);
  if (cur_ != nullptr && cur_->mem_rmask != 0) {
    const uint64_t v = cur_->mem_rdata;
    const size_t n = len < sizeof(v) ? len : sizeof(v);
    memcpy(bytes, &v, n);
    n_mmio_supplied_++;
  }
  (void)paddr;
  return true;
}

bool rv_cosim::store(uint64_t paddr, size_t len, const uint8_t *bytes) {
  // A store to a device leaves no architectural state the reference can check,
  // so it is accepted and dropped.
  (void)paddr;
  (void)len;
  (void)bytes;
  return true;
}

void rv_cosim::report(FILE *f) const {
  fprintf(f,
          "*** [cva6tb_cosim] INFO: %" PRIu64 " instructions compared, %" PRIu64 " traps (%" PRIu64
          " interrupts), %" PRIu64 " CSR writes\n",
          n_compared_, n_traps_, n_interrupts_, n_trap_csrs_);
  fprintf(f, "*** [cva6tb_cosim] INFO: %" PRIu64 " MMIO loads supplied by the DUT\n",
          n_mmio_supplied_);
  // Which modes were reached at all. A mode comparison that never left M proves
  // nothing about the privilege checks, the same way "no mismatches" proves
  // nothing without the per-field counts above. Only the modes that occurred
  // are listed, so a plain M-mode run says so by saying nothing else.
  if (n_irq_injected_ != 0) {
    fprintf(f,
            "*** [cva6tb_cosim] INFO: %" PRIu64
            " interrupt entries taken from the DUT, not derived\n",
            n_irq_injected_);
  }
  {
    std::string split;
    for (unsigned m = 0; m < 6; m++) {
      if (n_mode_[m] == 0) continue;
      char one[64];
      snprintf(one, sizeof(one), "%s%s %" PRIu64, split.empty() ? "" : ", ",
               rv_mode_name(m), n_mode_[m]);
      split += one;
    }
    if (!split.empty()) {
      fprintf(f, "*** [cva6tb_cosim] INFO: retirements by mode: %s\n", split.c_str());
    }
  }
  if (n_csr_vs_aliased_ != 0) {
    fprintf(f,
            "*** [cva6tb_cosim] INFO: %" PRIu64
            " S-mode CSR writes not compared: VS mode aliases them to the VS register\n",
            n_csr_vs_aliased_);
  }
  if (n_csr_seeded_ != 0) {
    fprintf(f,
            "*** [cva6tb_cosim] INFO: %" PRIu64
            " CSRs seeded from the DUT's reset state, not compared\n",
            n_csr_seeded_);
  }
  if (n_csr_absent_ != 0) {
    fprintf(f,
            "*** [cva6tb_cosim] INFO: %" PRIu64
            " CSR writes to registers the reference does not implement\n",
            n_csr_absent_);
  }
  if (mem_.unmapped() != 0) {
    fprintf(f, "*** [cva6tb_cosim] INFO: %" PRIu64 " accesses outside every region\n",
            mem_.unmapped());
  }
  // What was actually compared, field by field. A clean result means nothing
  // without it: this is the difference between "no mismatches" and "nothing
  // was checked", which look identical from the outside.
  fprintf(f, "*** [cva6tb_cosim] INFO: comparisons made, by field:\n");
  for (unsigned i = 0; i < kNumFields; i++) {
    if (n_checks_[i] == 0) continue;
    fprintf(f, "***   %-12s %10" PRIu64 "\n", rv_field_name((rv_field)i), n_checks_[i]);
  }
  policy_.report(f);
  policy_.report_unused(f);
  if (n_mismatches_ == 0) {
    fprintf(f, "*** [cva6tb_cosim] INFO: no mismatches\n");
  } else {
    fprintf(f, "*** [cva6tb_cosim] ERROR: %" PRIu64 " mismatches\n", n_mismatches_);
  }
}

}  // namespace cva6tb_cosim
