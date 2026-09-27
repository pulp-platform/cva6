// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// The output formats and the factory that picks between them.

#include "rv_format.h"

#include <cstdarg>
#include <cstring>
#include <string>

#include "cva6tb_tracer_dpi.h"

namespace cva6tb_tracer {

namespace {

char priv_char(unsigned char priv) {
  switch (priv & 3) {
    case 0: return 'U';
    case 1: return 'S';
    case 2: return 'D';  // cva6 reports debug mode as 2'b10 on rvfi.mode
    default: return 'M';
  }
}

// Human readable cause, in the wording the legacy ex_trace_item used.
// The exception code lives in the low 12 bits; in CLIC mode the bits above it
// carry the saved interrupt level and privilege, so they must be masked off.
const char *cause_text(uint64_t cause, int xlen) {
  const uint64_t irq_bit = 1ull << (xlen - 1);
  const uint64_t code = cause & 0xFFFull;
  if (cause & irq_bit) {
    switch (code) {
      case 1:  return "Supervisor Software Interrupt";
      case 3:  return "Machine Software Interrupt";
      case 5:  return "Supervisor Timer Interrupt";
      case 7:  return "Machine Timer Interrupt";
      case 9:  return "Supervisor External Interrupt";
      case 11: return "Machine External Interrupt";
      case 2:  return "Virtual Supervisor Software Interrupt";
      case 6:  return "Virtual Supervisor Timer Interrupt";
      case 10: return "Virtual Supervisor External Interrupt";
      case 12: return "Supervisor Guest External Interrupt";
      default: return "Interrupt";
    }
  }
  switch (code) {
    case 0:  return "Instruction Address Misaligned";
    case 1:  return "Instruction Access Fault";
    case 2:  return "Illegal Instruction";
    case 3:  return "Breakpoint";
    case 4:  return "Load Address Misaligned";
    case 5:  return "Load Access Fault";
    case 6:  return "Store Address Misaligned";
    case 7:  return "Store Access Fault";
    case 8:  return "Environment Call User Mode";
    case 9:  return "Environment Call Supervisor Mode";
    case 10: return "Environment Call Virtual Supervisor Mode";
    case 11: return "Environment Call Machine Mode";
    case 12: return "Instruction Page Fault";
    case 13: return "Load Page Fault";
    case 15: return "Store Page Fault";
    case 20: return "Instruction Guest Page Fault";
    case 21: return "Load Guest Page Fault";
    case 22: return "Virtual Instruction";
    case 23: return "Store Guest Page Fault";
    default: return "Unknown Exception";
  }
}

// tval carries no information for environment calls and interrupts
bool cause_has_tval(uint64_t cause, int xlen) {
  if (cause & (1ull << (xlen - 1))) return false;
  const uint64_t code = cause & 0xFFFull;
  return !(code >= 8 && code <= 11);  // environment calls
}

// Appending helper: keeps track of how much of `buf` is already used.
struct line_t {
  char   buf[1024];
  size_t n = 0;

  void add(const char *f, ...) __attribute__((format(printf, 2, 3))) {
    if (n >= sizeof(buf) - 1) return;
    va_list ap;
    va_start(ap, f);
    int w = vsnprintf(buf + n, sizeof(buf) - n, f, ap);
    va_end(ap);
    if (w > 0) n += ((size_t)w < sizeof(buf) - n) ? (size_t)w : (sizeof(buf) - n - 1);
  }
};

// Match a decoded source operand against the two operand slots RVFI reports.
// Integer code matches by register number; for the few cva6 instructions whose
// scoreboard operand slots do not line up with the architectural rs1/rs2 (most
// notably the FP ops) we fall back to consuming the slots in order.
struct src_values_t {
  uint8_t  addr[2];
  uint64_t val[2];
  bool     used[2];

  bool lookup(const reg_ref_t &r, uint64_t *out) {
    for (int i = 0; i < 2; i++)
      if (!used[i] && addr[i] == r.idx) { used[i] = true; *out = val[i]; return true; }
    for (int i = 0; i < 2; i++)
      if (!used[i]) { used[i] = true; *out = val[i]; return true; }
    return false;
  }
};

}  // namespace

// ---------------------------------------------------------------------------
// legacy: the format of common/local/util/instr_tracer.sv
// ---------------------------------------------------------------------------

namespace {

class legacy_formatter_t : public formatter_t {
 public:
  explicit legacy_formatter_t(const format_config_t &cfg) : cfg_(cfg) {}

  void header(FILE *f) override {
    if (cfg_.verbosity <= 0) return;
    fprintf(f, "# cva6 tracer: RV%d, verbosity=%d\n", cfg_.xlen, cfg_.verbosity);
    fprintf(f, "# time cycle priv pc mispredict insn%s disassembly  <written reg>  <read regs>\n",
            cfg_.verbosity >= 2 ? " [port #order]" : "");
  }

  void retire(FILE *f, const retire_info_t &r) override;
  void trap(FILE *f, const trap_info_t &t) override;

 private:
  format_config_t cfg_;
};

void legacy_formatter_t::retire(FILE *f, const retire_info_t &r) {
  const decoded_t &d = *r.dec;
  const uint32_t insn_print = ((r.insn & 0x3u) != 0x3u) ? (r.insn & 0xFFFFu) : r.insn;

  line_t l;
  l.add("%8lluns %8llu %c %0*llx %llu %08x ", r.sim_time_ns, r.cycle, priv_char(r.priv),
        cfg_.pc_digits, r.pc, (unsigned long long)(r.flags & CVA6_TRACE_FLAG_MISPREDICT),
        insn_print);
  if (cfg_.verbosity >= 2) l.add("[p%u #%llu] ", (unsigned)r.commit_port, r.order);
  l.add("%-36s", d.text.c_str());

  // destination register, with the value RVFI reports for it
  if (d.dst.kind == REG_FPR) {
    l.add(" %-4s:%016llx", rv_fpr_name(d.dst.idx), r.rd_wdata);
  } else if (d.dst.kind == REG_GPR && d.dst.idx != 0) {
    l.add(" %-4s:%016llx", rv_gpr_name(d.dst.idx), r.rd_wdata);
  }

  // source registers
  src_values_t srcs = {{r.rs1_addr, r.rs2_addr}, {r.rs1_rdata, r.rs2_rdata}, {false, false}};
  for (unsigned i = 0; i < d.n_src; i++) {
    const reg_ref_t &reg = d.src[i];
    uint64_t v = 0;
    const bool known = srcs.lookup(reg, &v);
    if (reg.kind == REG_FPR) {
      if (known) l.add(" %-4s:%016llx", rv_fpr_name(reg.idx), (unsigned long long)v);
      else l.add(" %-4s:%16s", rv_fpr_name(reg.idx), "----------------");
    } else if (reg.kind == REG_GPR && reg.idx != 0) {
      if (known) l.add(" %-4s:%016llx", rv_gpr_name(reg.idx), (unsigned long long)v);
      else l.add(" %-4s:%16s", rv_gpr_name(reg.idx), "----------------");
    }
  }

  // addresses: the legacy tracer printed these for plain loads and stores only
  const bool show_addr = d.is_load || d.is_store || (cfg_.verbosity >= 1 && d.is_amo);
  if (show_addr && (r.mem_rmask || r.mem_wmask)) {
    l.add(" VA: %0*llx PA: %0*llx", cfg_.pc_digits, r.mem_vaddr, cfg_.pa_digits, r.mem_paddr);
  }

  if (cfg_.verbosity >= 1) {
    const unsigned bytes = d.mem_bytes ? d.mem_bytes : 8;
    const uint64_t mask = (bytes >= 8) ? ~0ull : ((1ull << (bytes * 8)) - 1);
    if (r.mem_wmask) l.add(" ST:0x%0*llx", (int)(bytes * 2), r.mem_wdata & mask);
    if (r.mem_rmask) l.add(" LD:0x%0*llx", (int)(bytes * 2), r.mem_rdata & mask);
    if (d.is_csr) {
      l.add(" csr:%s", rv_csr_name(d.csr_addr).c_str());
      // reconstruct the written value where RVFI gives us enough information
      const unsigned f3 = (r.insn >> 12) & 7;
      const unsigned rs1_field = (r.insn >> 15) & 0x1F;
      const uint64_t operand = ((f3 & 4) != 0) ? rs1_field : r.rs1_rdata;
      const bool     writes = (f3 & 3) == 1 || rs1_field != 0;
      const bool     old_known = r.rd_addr != 0;
      if (writes) {
        switch (f3 & 3) {
          case 1: l.add(" <-%016llx", (unsigned long long)operand); break;
          case 2:
            if (old_known) l.add(" <-%016llx", (unsigned long long)(r.rd_wdata | operand));
            break;
          case 3:
            if (old_known) l.add(" <-%016llx", (unsigned long long)(r.rd_wdata & ~operand));
            break;
          default: break;
        }
      }
    }
  }

  // branch outcome, which only a resolved control flow instruction carries
  if (cfg_.verbosity >= 1 && (r.flags & CVA6_TRACE_FLAG_BRANCH)) {
    l.add(" br:%s%s", (r.flags & CVA6_TRACE_FLAG_TAKEN) ? "taken" : "not-taken",
          (r.flags & CVA6_TRACE_FLAG_MISPREDICT) ? ",mispredicted" : "");
  }

  if (cfg_.verbosity >= 1 && (r.flags & CVA6_TRACE_FLAG_TRAP_ENTRY))
    l.add((r.flags & CVA6_TRACE_FLAG_INTERRUPT) ? "  <- interrupt entry" : "  <- trap entry");

  if (r.sym) {
    const uint64_t off = r.pc - r.sym->addr;
    if (off) l.add(" <%s+0x%llx>", r.sym->name.c_str(), (unsigned long long)off);
    else l.add(" <%s>", r.sym->name.c_str());
  }

  l.add("\n");
  fwrite(l.buf, 1, l.n, f);
}

void legacy_formatter_t::trap(FILE *f, const trap_info_t &t) {
  line_t l;
  l.add("Exception @%10lluns, PC: %0*llx, Cause: %s", t.sim_time_ns, cfg_.pc_digits, t.pc,
        cause_text(t.cause, cfg_.xlen));
  if (cause_has_tval(t.cause, cfg_.xlen)) l.add(", \n\t\t\t\ttval: %016llx", t.tval);
  if (cfg_.verbosity >= 1) {
    const uint32_t insn_print = ((t.insn & 0x3u) != 0x3u) ? (t.insn & 0xFFFFu) : t.insn;
    l.add("\n#%8lluns %8llu %c %0*llx - %08x %-36s [%s cause:%016llx]", t.sim_time_ns, t.cycle,
          priv_char(t.priv), cfg_.pc_digits, t.pc, insn_print, t.dec->text.c_str(),
          rv_cause_name(t.cause, cfg_.xlen).c_str(), t.cause);
  }
  l.add("\n");
  fwrite(l.buf, 1, l.n, f);
}

}  // namespace

// ---------------------------------------------------------------------------
// The commit log, in two dialects.
//
// Two lines per instruction: the first names it, the second says what it
// changed. `spike` reproduces what `spike -l --log-commits` writes, down to the
// per line `core N:` prefix and the register column, so the two logs diff
// against each other directly. `rvfi` is the same information in the spelling
// corev_apu/tb/rvfi_tracer.sv uses, which is what some of the converters under
// verif/sim/ were written against.
//
// Neither writes a DASM() placeholder: the disassembly is produced here, so
// there is no spike-dasm pass to run.
// ---------------------------------------------------------------------------

namespace {

// spike's wording for a trap, which is what the converters' ILLE_RE looks for
const char *spike_cause_name(uint64_t cause, int xlen) {
  if (cause & (1ull << (xlen - 1))) return "interrupt";
  switch (cause & 0xFFFull) {
    case 0:  return "trap_instruction_address_misaligned";
    case 1:  return "trap_instruction_access_fault";
    case 2:  return "trap_illegal_instruction";
    case 3:  return "trap_breakpoint";
    case 4:  return "trap_load_address_misaligned";
    case 5:  return "trap_load_access_fault";
    case 6:  return "trap_store_address_misaligned";
    case 7:  return "trap_store_access_fault";
    case 8:  return "trap_user_ecall";
    case 9:  return "trap_supervisor_ecall";
    case 10: return "trap_virtual_supervisor_ecall";
    case 11: return "trap_machine_ecall";
    case 12: return "trap_instruction_page_fault";
    case 13: return "trap_load_page_fault";
    case 15: return "trap_store_page_fault";
    case 20: return "trap_instruction_guest_page_fault";
    case 21: return "trap_load_guest_page_fault";
    case 22: return "trap_virtual_instruction";
    case 23: return "trap_store_guest_page_fault";
    default: return "trap_unknown";
  }
}

// Our disassembly is laid out in the legacy tracer's columns; spike uses its
// own. Re-space it rather than teach the disassembler a second layout: take the
// mnemonic, collapse the runs of padding in the operands, lay it out again.
std::string respace(const std::string &text) {
  const size_t split = text.find(' ');
  if (split == std::string::npos) return text;  // no operands
  std::string ops;
  bool        gap = false;
  for (size_t i = split; i < text.size(); i++) {
    if (text[i] == ' ') { gap = true; continue; }
    if (!ops.empty() && gap) ops += ' ';
    gap = false;
    ops += text[i];
  }
  if (ops.empty()) return text.substr(0, split);
  char buf[256];
  snprintf(buf, sizeof(buf), "%-7s %s", text.substr(0, split).c_str(), ops.c_str());
  return buf;
}

class commit_log_formatter_t : public formatter_t {
 public:
  enum dialect_t { SPIKE, RVFI };

  commit_log_formatter_t(const format_config_t &cfg, dialect_t dialect)
      : cfg_(cfg), dialect_(dialect) {}

  bool machine_readable() const override { return true; }

  void retire(FILE *f, const retire_info_t &r) override {
    const decoded_t &d = *r.dec;
    const bool       compressed = (r.insn & 0x3u) != 0x3u;
    const uint32_t   insn_short = compressed ? (r.insn & 0xFFFFu) : r.insn;
    const bool       exact = dialect_ == SPIKE;

    line_t l;
    // the instruction itself
    if (exact) {
      l.add("core %3u: 0x%0*llx (0x%08x) %s\n", cfg_.hart_id, cfg_.pc_digits, r.pc, r.insn,
            respace(d.text).c_str());
    } else {
      l.add("core %s%u: 0x%0*llx (0x%08x) %s\n",
            (r.flags & CVA6_TRACE_FLAG_INTERRUPT) ? "  INTERRUPT " : "  ", cfg_.hart_id,
            cfg_.pc_digits, r.pc, r.insn, d.text.c_str());
    }

    // and what it changed
    if (exact) l.add("core %3u: ", cfg_.hart_id);
    l.add("%x 0x%0*llx (0x%0*x)", r.priv & 3, cfg_.pc_digits, r.pc, compressed ? 4 : 8,
          insn_short);
    // spike left aligns the register number, rvfi_tracer right aligns it
    const char *reg_fmt = exact ? " %c%-2u 0x%016llx" : " %c%2u 0x%016llx";
    if (d.dst.kind == REG_FPR) {
      l.add(reg_fmt, 'f', (unsigned)d.dst.idx, r.rd_wdata);
    } else if (d.dst.kind == REG_GPR && d.dst.idx != 0) {
      l.add(reg_fmt, 'x', (unsigned)d.dst.idx, r.rd_wdata);
      if (r.mem_rmask) l.add(" mem 0x%0*llx", cfg_.pc_digits, r.mem_vaddr);
    }
    if (r.mem_wmask) l.add(" mem 0x%0*llx 0x%016llx", cfg_.pc_digits, r.mem_vaddr, r.mem_wdata);
    l.add("\n");

    fwrite(l.buf, 1, l.n, f);
  }

  void trap(FILE *f, const trap_info_t &t) override {
    if (dialect_ == SPIKE) {
      fprintf(f, "core %3u: exception %s, epc 0x%0*llx\n", cfg_.hart_id,
              spike_cause_name(t.cause, cfg_.xlen), cfg_.pc_digits, t.pc);
      return;
    }
    const uint32_t insn_short = ((t.insn & 0x3u) != 0x3u) ? (t.insn & 0xFFFFu) : t.insn;
    fprintf(f, "%s exception @ 0x%0*llx (0x%x)\n", rv_cause_name(t.cause, cfg_.xlen).c_str(),
            cfg_.pc_digits, t.pc, insn_short);
  }

 private:
  format_config_t cfg_;
  dialect_t       dialect_;
};

}  // namespace

// ---------------------------------------------------------------------------
// binary: fixed size little endian records
//
// Meant for tools, not for people. A short header names the geometry, then one
// record per event, all the same size and all naturally aligned, so a reader
// maps the file and indexes into it instead of parsing anything:
//
//   import numpy as np
//   rec = np.dtype([('cycle','<u8'), ('time_ns','<u8'), ...])
//   data = np.fromfile(path, dtype=rec, offset=64)
//
// The layout is documented in util/read_trace.py, which is both the reference
// reader and a converter. Fields keep their meaning across record kinds except
// where the table there says otherwise, and `reserved` leaves room to add
// fields without moving the ones already there.
// ---------------------------------------------------------------------------

namespace {

constexpr uint16_t BINARY_VERSION = 2;  // 2 added the class byte, taken from reserved
constexpr uint16_t BINARY_RECORD_SIZE = 96;
constexpr uint8_t  BINARY_KIND_RETIRE = 0;
constexpr uint8_t  BINARY_KIND_TRAP = 1;

// What the instruction is, so that a reader does not need a RISC-V decoder to
// answer the questions a decoder would otherwise be needed for.
constexpr uint8_t BINARY_CLASS_LOAD = 0x01;
constexpr uint8_t BINARY_CLASS_STORE = 0x02;
constexpr uint8_t BINARY_CLASS_AMO = 0x04;
constexpr uint8_t BINARY_CLASS_CSR = 0x08;
constexpr uint8_t BINARY_CLASS_BRANCH = 0x10;
constexpr uint8_t BINARY_CLASS_JUMP = 0x20;
constexpr uint8_t BINARY_CLASS_COMPRESSED = 0x40;

uint8_t binary_class_of(const decoded_t &d, uint32_t insn) {
  uint8_t c = 0;
  if (d.is_load) c |= BINARY_CLASS_LOAD;
  if (d.is_store) c |= BINARY_CLASS_STORE;
  if (d.is_amo) c |= BINARY_CLASS_AMO;
  if (d.is_csr) c |= BINARY_CLASS_CSR;
  if (d.is_branch) c |= BINARY_CLASS_BRANCH;
  if (d.is_jump) c |= BINARY_CLASS_JUMP;
  if ((insn & 0x3u) != 0x3u) c |= BINARY_CLASS_COMPRESSED;
  return c;
}

#pragma pack(push, 1)
struct binary_header_t {
  char     magic[8];  // "CVA6TRC"
  uint16_t version;
  uint16_t record_size;
  uint8_t  xlen;
  uint8_t  vlen;
  uint8_t  plen;
  uint8_t  hart_id;
  uint8_t  reserved[48];
};

struct binary_record_t {
  uint64_t cycle;
  uint64_t time_ns;
  uint64_t pc;
  uint64_t rd_wdata;   // trap: cause
  uint64_t rs1_rdata;  // trap: tval
  uint64_t rs2_rdata;  // trap: unused
  uint64_t mem_vaddr;
  uint64_t mem_paddr;
  uint64_t mem_wdata;
  uint64_t mem_rdata;
  uint32_t insn;
  uint8_t  kind;
  uint8_t  priv;
  uint8_t  flags;
  uint8_t  rd_addr;
  uint8_t  rs1_addr;
  uint8_t  rs2_addr;
  uint8_t  commit_port;
  uint8_t  mem_rmask;
  uint8_t  mem_wmask;
  uint8_t  klass;  // BINARY_CLASS_*, so no decoder is needed downstream
  uint8_t  reserved[2];
};
#pragma pack(pop)

static_assert(sizeof(binary_header_t) == 64, "binary header must stay 64 bytes");
static_assert(sizeof(binary_record_t) == BINARY_RECORD_SIZE, "record size must match the header");

class binary_formatter_t : public formatter_t {
 public:
  explicit binary_formatter_t(const format_config_t &cfg) : cfg_(cfg) {}

  bool machine_readable() const override { return true; }

  void header(FILE *f) override {
    binary_header_t h;
    memset(&h, 0, sizeof(h));
    memcpy(h.magic, "CVA6TRC", 7);
    h.version = BINARY_VERSION;
    h.record_size = BINARY_RECORD_SIZE;
    h.xlen = (uint8_t)cfg_.xlen;
    h.vlen = (uint8_t)(cfg_.pc_digits * 4);
    h.plen = (uint8_t)(cfg_.pa_digits * 4);
    h.hart_id = (uint8_t)cfg_.hart_id;
    fwrite(&h, sizeof(h), 1, f);
  }

  void retire(FILE *f, const retire_info_t &r) override {
    binary_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.kind = BINARY_KIND_RETIRE;
    rec.cycle = r.cycle;
    rec.time_ns = r.sim_time_ns;
    rec.pc = r.pc;
    rec.rd_wdata = r.rd_wdata;
    rec.rs1_rdata = r.rs1_rdata;
    rec.rs2_rdata = r.rs2_rdata;
    rec.mem_vaddr = r.mem_vaddr;
    rec.mem_paddr = r.mem_paddr;
    rec.mem_wdata = r.mem_wdata;
    rec.mem_rdata = r.mem_rdata;
    rec.insn = r.insn;
    rec.priv = r.priv;
    rec.flags = r.flags;
    rec.rd_addr = r.rd_addr;
    rec.rs1_addr = r.rs1_addr;
    rec.rs2_addr = r.rs2_addr;
    rec.commit_port = r.commit_port;
    rec.mem_rmask = (uint8_t)r.mem_rmask;
    rec.mem_wmask = (uint8_t)r.mem_wmask;
    rec.klass = binary_class_of(*r.dec, r.insn);
    fwrite(&rec, sizeof(rec), 1, f);
  }

  void trap(FILE *f, const trap_info_t &t) override {
    binary_record_t rec;
    memset(&rec, 0, sizeof(rec));
    rec.kind = BINARY_KIND_TRAP;
    rec.cycle = t.cycle;
    rec.time_ns = t.sim_time_ns;
    rec.pc = t.pc;
    rec.rd_wdata = t.cause;
    rec.rs1_rdata = t.tval;
    rec.insn = t.insn;
    rec.priv = t.priv;
    rec.flags = t.is_interrupt ? CVA6_TRACE_FLAG_INTERRUPT : 0;
    rec.klass = binary_class_of(*t.dec, t.insn);
    fwrite(&rec, sizeof(rec), 1, f);
  }

 private:
  format_config_t cfg_;
};

}  // namespace

// ---------------------------------------------------------------------------

formatter_t *make_formatter(const char *name, const format_config_t &cfg) {
  const std::string n(name ? name : "");
  if (n.empty() || n == "legacy") return new legacy_formatter_t(cfg);
  if (n == "spike") return new commit_log_formatter_t(cfg, commit_log_formatter_t::SPIKE);
  if (n == "rvfi") return new commit_log_formatter_t(cfg, commit_log_formatter_t::RVFI);
  if (n == "binary") return new binary_formatter_t(cfg);
  return nullptr;
}

const char *formatter_names() { return "legacy, spike, rvfi, binary"; }

}  // namespace cva6tb_tracer
