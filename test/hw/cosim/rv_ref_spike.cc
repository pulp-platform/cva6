// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>

#include "rv_ref_spike.h"

#include <iostream>

#include <riscv/decode.h>
#include <riscv/mmu.h>

namespace cva6tb_cosim {

namespace {

// log_reg_write is keyed by (number << 4) | type, with the type nibble defined
// in spike's decode_macros.h. CSR entries are keyed by CSR *address*, not by a
// register number -- see csrs.cc, which writes ((address) << 4) | 4.
constexpr uint64_t kTagInt = 0;
constexpr uint64_t kTagFp = 1;
constexpr uint64_t kTagVector = 2;
constexpr uint64_t kTagVectorHint = 3;
constexpr uint64_t kTagCsr = 4;

rv_reg_kind kind_of(uint64_t tag) {
  switch (tag) {
    case kTagInt: return rv_reg_kind::gpr;
    case kTagFp: return rv_reg_kind::fpr;
    case kTagVector: return rv_reg_kind::vector;
    case kTagCsr: return rv_reg_kind::csr;
    case kTagVectorHint: return rv_reg_kind::none;  // fixed key 3, carries nothing
    default: return rv_reg_kind::none;
  }
}

}  // namespace

rv_ref_spike::rv_ref_spike(rv_memif &mem, const std::string &isa, const std::string &priv,
                           uint64_t start_pc, unsigned vaddr_bits,
                           const std::string &log_path)
    : mem_(mem) {
  mem_.cfg().isa = isa.c_str();
  mem_.cfg().priv = priv.c_str();

  // commit_log_print_insn() writes to this unconditionally once commit logging
  // is on (execute.cc), so it needs a real FILE*.
  log_ = fopen(log_path.c_str(), "w");
  if (log_ == nullptr) {
    fprintf(stderr, "*** [cva6tb_cosim] ERROR: cannot open spike log '%s'\n",
            log_path.c_str());
  }

  proc_ = std::make_unique<processor_t>(isa.c_str(), priv.c_str(), &mem_.cfg(), &mem_,
                                        /*id=*/0, /*halt_on_reset=*/false, log_, std::cerr);
  mem_.add_hart(0, proc_.get());

  // Paging, and then a reset. Both are needed and the order matters: satp masks
  // its MODE field to zero unless the MMU width is set, and the writable masks
  // of mstatus and sstatus capture `has_page` once, when the CSR objects are
  // built (csrs.cc). The constructor built them with no MMU, so without the
  // reset that rebuilds them SUM, MXR and TVM stay permanently unwritable --
  // which shows up much later as a supervisor program that cannot reach user
  // memory. This is what sim_t does after its own device tree parse, and
  // replicating it is the price of driving processor_t directly.
  proc_->set_max_vaddr_bits(vaddr_bits);
  proc_->reset();

  // After the reset, which clears the state the CSRs live in. Commit logging
  // selects the logged variant of each instruction in insn_template.cc, which is
  // what fills log_reg_write: the reference reports register writes only with it
  // on. It costs one formatted line per instruction, so the log goes to
  // /dev/null unless one is asked for.
  proc_->enable_log_commits();

  proc_->get_state()->pc = start_pc;
}

rv_ref_spike::~rv_ref_spike() {
  // The processor holds this FILE*, so it must outlive it.
  proc_.reset();
  if (log_ != nullptr) fclose(log_);
}

bool rv_ref_spike::step(rv_ref_effects &out) {
  state_t *s = proc_->get_state();

  // Spike has no last_inst_pc, so the PC of the instruction that is about to
  // execute has to be taken before stepping.
  out = rv_ref_effects();
  out.pc = s->pc;
  // The virtualisation mode the instruction runs *in*. There is no equivalent of
  // last_inst_priv for it, so it is taken before the step.
  const bool virt_before = s->v;

  // Fetching separately from executing: the encoding is wanted for reporting,
  // and spike does not keep the one it just ran.
  try {
    insn_fetch_t fetch = proc_->get_mmu()->load_insn(s->pc);
    out.insn = (uint32_t)fetch.insn.bits();
  } catch (const trap_t &) {
    // An instruction that cannot even be fetched is still a valid outcome: the
    // step below will take the same fault and report it.
    out.insn = 0;
  }

  proc_->step(1);

  out.next_pc = s->pc;
  // The privilege the instruction *executed in*, not the one left behind: RVFI's
  // mode is the retiring instruction's, and the two differ on exactly the
  // instructions that change privilege, which is where it matters most. Spike
  // stashes it for its own commit log (commit_log_stash_privilege).
  out.priv = (uint8_t)s->last_inst_priv;
  out.virt = virt_before;

  // log_reg_write is cleared at the start of every step (execute.cc), so what
  // is in it now belongs to the instruction just executed.
  for (const auto &w : s->log_reg_write) {
    if (out.n_writes >= rv_ref_effects::kMaxWrites) break;
    const rv_reg_kind k = kind_of(w.first & 0xf);
    if (k == rv_reg_kind::none) continue;
    rv_reg_write &rec = out.writes[out.n_writes++];
    rec.kind = k;
    rec.index = (uint32_t)(w.first >> 4);
    rec.value = w.second.v[0];
  }

  // Reads first, then writes, so that an AMO reports them in program order.
  for (const auto &a : s->log_mem_read) {
    if (out.n_accesses >= rv_ref_effects::kMaxAccesses) break;
    rv_mem_access &rec = out.accesses[out.n_accesses++];
    rec.addr = std::get<0>(a);
    rec.bytes = std::get<2>(a);
    rec.is_write = false;
  }
  for (const auto &a : s->log_mem_write) {
    if (out.n_accesses >= rv_ref_effects::kMaxAccesses) break;
    rv_mem_access &rec = out.accesses[out.n_accesses++];
    rec.addr = std::get<0>(a);
    rec.value = std::get<1>(a);
    rec.bytes = std::get<2>(a);
    rec.is_write = true;
  }

  // A trap is visible in the same place: take_trap() writes the epc, the cause
  // and tval through the CSR objects, and those writes are logged like any other
  // (csrs.cc). Requiring *both* an epc and a cause in one instruction is what
  // separates a trap from a program writing mcause by hand, which writes only
  // the one register.
  bool have_epc = false;
  bool have_cause = false;
  for (unsigned i = 0; i < out.n_writes; i++) {
    if (out.writes[i].kind != rv_reg_kind::csr) continue;
    switch (out.writes[i].index) {
      case 0x341:  // mepc
      case 0x141:  // sepc
      case 0x241:  // vsepc
        have_epc = true;
        break;
      case 0x342:  // mcause
      case 0x142:  // scause
      case 0x242:  // vscause
        have_cause = true;
        out.trap_cause = out.writes[i].value;
        break;
      case 0x343:  // mtval
      case 0x143:  // stval
      case 0x243:  // vstval
        out.trap_tval = out.writes[i].value;
        break;
      default:
        break;
    }
  }
  out.trapped = have_epc && have_cause;

  return true;
}

void rv_ref_spike::skip(uint64_t next_pc) { proc_->get_state()->pc = next_pc; }

void rv_ref_spike::set_pending_irq(uint64_t mask) {
  // Raises the bits as pending and leaves everything else to spike: delegation,
  // vectoring, mstatus.MPP/MPIE and the handler PC are all decided by its own
  // logic, so only the *arrival* of the interrupt comes from the DUT.
  proc_->get_state()->mip->backdoor_write_with_mask(mask, mask);
}

// processor_t::get_csr throws trap_illegal_instruction for a register it does
// not implement, peek or not, so existence is tested against the map, which
// keeps exceptions off the DPI boundary.
bool rv_ref_spike::has_csr(uint32_t addr) {
  const auto &m = proc_->get_state()->csrmap;
  return m.find((reg_t)addr) != m.end();
}

uint64_t rv_ref_spike::get_csr(uint32_t addr) { return proc_->get_csr((int)addr); }

void rv_ref_spike::put_csr(uint32_t addr, uint64_t value) {
  proc_->put_csr((int)addr, (reg_t)value);
}

uint64_t rv_ref_spike::get_xpr(unsigned i) { return proc_->get_state()->XPR[i]; }

void rv_ref_spike::put_xpr(unsigned i, uint64_t value) {
  proc_->get_state()->XPR.write(i, (reg_t)value);
}

uint64_t rv_ref_spike::get_fpr(unsigned i) { return proc_->get_state()->FPR[i].v[0]; }

void rv_ref_spike::put_fpr(unsigned i, uint64_t value) {
  freg_t f;
  f.v[0] = value;
  // NaN boxing: the upper half is all ones for anything narrower than the
  // widest supported float, which is what spike's own freg() helpers produce.
  f.v[1] = (uint64_t)-1;
  proc_->get_state()->FPR.write(i, f);
}

uint64_t rv_ref_spike::pc() { return proc_->get_state()->pc; }

void rv_ref_spike::set_pc(uint64_t pc) { proc_->get_state()->pc = pc; }

uint8_t rv_ref_spike::priv() { return (uint8_t)proc_->get_state()->prv; }

bool rv_ref_spike::virt() { return proc_->get_state()->v; }

}  // namespace cva6tb_cosim
