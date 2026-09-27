// Copyright 2026 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Description: Backend of the CVA6 instruction tracer.
//
//              This file decides *what* gets reported: it owns the output file,
//              the trace and annotation windows, the symbol table and the
//              decoder. How a line actually looks is left to a formatter in
//              rv_format.h, so a new output format does not touch anything
//              here.

#include "cva6tb_tracer_dpi.h"

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "rv_disasm.h"
#include "rv_format.h"
#include "rv_stats.h"
#include "rv_symbols.h"
#include "rv_trigger.h"

using namespace cva6tb_tracer;

namespace {

// How symbol names are attached to the trace.
enum symbol_mode_t { SYM_OFF = 0, SYM_HEADER = 1, SYM_INLINE = 2, SYM_BOTH = 3 };

struct tracer_t {
  FILE       *f = nullptr;
  int         xlen = 64;
  unsigned    extensions = 0;
  uint64_t    n_seen = 0;     // retirements handed to the tracer
  uint64_t    n_retired = 0;  // of those, the ones inside the trace window
  uint64_t    n_traps = 0;

  // how a line looks, and everything it needs to know
  format_config_t fcfg;
  std::string     format_name = "legacy";
  formatter_t    *fmt = nullptr;
  bool            header_written = false;

  // optional symbol annotation, off unless a source is configured
  symbol_table_t symbols;
  int            symbol_mode = SYM_HEADER;
  bool           have_last_symbol = false;
  uint64_t       last_symbol_addr = 0;

  // windows: one gating the trace, one gating annotation within it
  window_t trace_window;
  window_t symbol_window;

  // optional end of run summary, written to its own file
  stats_t     stats;
  std::string stats_path;
};

std::vector<tracer_t *> g_tracers;

// Builds the formatter named by t->format_name from the current configuration,
// replacing any previous one. Returns false for an unknown name.
bool rebuild_formatter(tracer_t *t) {
  formatter_t *next = make_formatter(t->format_name.c_str(), t->fcfg);
  if (!next) return false;
  delete t->fmt;
  t->fmt = next;
  return true;
}

// The header belongs to whichever formatter is finally in force, so it is
// written at the first event rather than at open: configuration all happens
// before the first clock edge, and writing it early would emit the default
// format's header and then the chosen format's records.
void ensure_header(tracer_t *t) {
  if (t->header_written) return;
  t->header_written = true;
  t->fmt->header(t->f);
}

tracer_t *get(int handle) {
  if (handle < 0 || (size_t)handle >= g_tracers.size()) return nullptr;
  tracer_t *t = g_tracers[handle];
  return (t && t->f) ? t : nullptr;
}

}  // namespace

// ---------------------------------------------------------------------------

extern "C" int cva6tb_tracer_open(const char *filename, int xlen, int vlen, int plen,
                                  int verbosity, int extensions) {
  tracer_t *t = new tracer_t();
  t->f = fopen(filename, "w");
  if (!t->f) {
    fprintf(stderr, "*** [cva6tb_tracer] ERROR: cannot open '%s' for writing\n", filename);
    delete t;
    return -1;
  }
  // a generous stdio buffer keeps the tracer off the critical path
  setvbuf(t->f, nullptr, _IOFBF, 1 << 20);
  t->xlen = (xlen == 32) ? 32 : 64;
  t->extensions = (unsigned)extensions;
  t->fcfg.xlen = t->xlen;
  t->fcfg.verbosity = verbosity;
  t->fcfg.pc_digits = (vlen + 3) / 4;
  t->fcfg.pa_digits = (plen + 3) / 4;

  g_tracers.push_back(t);
  const int handle = (int)g_tracers.size() - 1;
  rebuild_formatter(t);

  printf("*** [cva6tb_tracer] INFO: writing '%s' (RV%d, verbosity %d)\n", filename, t->xlen,
         verbosity);
  return handle;
}

extern "C" int cva6tb_tracer_config(int handle, const char *key, const char *value) {
  tracer_t *t = get(handle);
  if (!t || !key) return -1;
  const std::string k(key);
  const char *v = value ? value : "";

  if (k == "symbols") {
    const int n = t->symbols.add_sources(v);
    if (n < 0) return -1;
    if (t->symbol_mode == SYM_OFF) t->symbol_mode = SYM_HEADER;
    printf("*** [cva6tb_tracer] INFO: %d symbols loaded from '%s'\n", n, v);
    return 0;
  }
  if (k == "symbol_mode") {
    const std::string m(v);
    if (m == "off") t->symbol_mode = SYM_OFF;
    else if (m == "header") t->symbol_mode = SYM_HEADER;
    else if (m == "inline") t->symbol_mode = SYM_INLINE;
    else if (m == "both") t->symbol_mode = SYM_BOTH;
    else return -1;
    return 0;
  }
  if (k == "format") {
    if (t->header_written) {
      fprintf(stderr, "*** [cva6tb_tracer] ERROR: the format cannot change once the trace has "
                      "started\n");
      return -1;
    }
    const std::string previous = t->format_name;
    t->format_name = v;
    if (!rebuild_formatter(t)) {
      t->format_name = previous;
      rebuild_formatter(t);
      fprintf(stderr, "*** [cva6tb_tracer] ERROR: unknown format '%s', known formats are %s\n", v,
              formatter_names());
      return -1;
    }
    return 0;
  }
  if (k == "stats") {
    t->stats_path = v;
    return 0;
  }
  if (k == "hart_id") {
    t->fcfg.hart_id = (unsigned)strtoul(v, nullptr, 0);
    return rebuild_formatter(t) ? 0 : -1;
  }
  if (k == "start") return t->trace_window.set_start(v, t->symbols) ? 0 : -1;
  if (k == "stop") return t->trace_window.set_stop(v, t->symbols) ? 0 : -1;
  if (k == "symbol_start") return t->symbol_window.set_start(v, t->symbols) ? 0 : -1;
  if (k == "symbol_stop") return t->symbol_window.set_stop(v, t->symbols) ? 0 : -1;

  fprintf(stderr, "*** [cva6tb_tracer] ERROR: unknown option '%s'\n", key);
  return -1;
}

namespace {

// Annotation is on only when a symbol source was configured and the annotation
// window is open. Emits the header line as a side effect when the symbol
// changes, and returns the symbol for the inline form.
const symbol_t *annotate(tracer_t *t, uint64_t cycle, uint64_t pc, const symbol_t *sym) {
  if (t->symbol_mode == SYM_OFF || t->symbols.empty()) return nullptr;
  if (!t->symbol_window.active(cycle, pc)) return nullptr;

  // a header line between two instruction lines would corrupt a format that is
  // meant to be parsed, so those get the inline form only
  const bool want_header = (t->symbol_mode == SYM_HEADER || t->symbol_mode == SYM_BOTH) &&
                           !t->fmt->machine_readable();
  if (want_header) {
    const bool changed = sym ? (!t->have_last_symbol || t->last_symbol_addr != sym->addr)
                             : t->have_last_symbol;
    if (changed && sym) fprintf(t->f, "%0*llx <%s>:\n", t->fcfg.pc_digits,
                                (unsigned long long)sym->addr, sym->name.c_str());
    t->have_last_symbol = sym != nullptr;
    t->last_symbol_addr = sym ? sym->addr : 0;
  }
  return (t->symbol_mode == SYM_INLINE || t->symbol_mode == SYM_BOTH) ? sym : nullptr;
}

}  // namespace

extern "C" void cva6tb_tracer_retire(int handle, unsigned long long cycle,
                                     unsigned long long sim_time_ns, unsigned char commit_port,
                                     unsigned long long order, unsigned char priv,
                                     unsigned char flags, unsigned long long pc,
                                     unsigned int insn, unsigned char rd_addr,
                                     unsigned long long rd_wdata, unsigned char rs1_addr,
                                     unsigned long long rs1_rdata, unsigned char rs2_addr,
                                     unsigned long long rs2_rdata, unsigned long long mem_vaddr,
                                     unsigned long long mem_paddr, unsigned int mem_rmask,
                                     unsigned int mem_wmask, unsigned long long mem_wdata,
                                     unsigned long long mem_rdata) {
  tracer_t *t = get(handle);
  if (!t) return;
  t->n_seen++;
  // the window is evaluated for every retirement, whether or not it is traced,
  // so that a pc or symbol trigger cannot be missed
  if (!t->trace_window.active(cycle, pc)) return;
  t->n_retired++;
  ensure_header(t);

  // one lookup feeds both the annotation and the profile in the summary
  const symbol_t *sym = t->symbols.empty() ? nullptr : t->symbols.lookup(pc);
  const symbol_t *inline_sym = annotate(t, cycle, pc, sym);
  const decoded_t d = rv_decode(insn, t->xlen, t->extensions);

  retire_info_t r;
  r.cycle = cycle;
  r.sim_time_ns = sim_time_ns;
  r.order = order;
  r.pc = pc;
  r.insn = insn;
  r.commit_port = commit_port;
  r.priv = priv;
  r.flags = flags;
  r.rd_addr = rd_addr;
  r.rs1_addr = rs1_addr;
  r.rs2_addr = rs2_addr;
  r.rd_wdata = rd_wdata;
  r.rs1_rdata = rs1_rdata;
  r.rs2_rdata = rs2_rdata;
  r.mem_vaddr = mem_vaddr;
  r.mem_paddr = mem_paddr;
  r.mem_wdata = mem_wdata;
  r.mem_rdata = mem_rdata;
  r.mem_rmask = mem_rmask;
  r.mem_wmask = mem_wmask;
  r.dec = &d;
  r.sym = inline_sym;

  t->fmt->retire(t->f, r);
  if (!t->stats_path.empty()) t->stats.record(r, sym);
}

extern "C" void cva6tb_tracer_trap(int handle, unsigned long long cycle,
                                   unsigned long long sim_time_ns, unsigned char priv,
                                   unsigned long long pc, unsigned int insn,
                                   unsigned long long cause, unsigned long long tval,
                                   unsigned char is_interrupt) {
  tracer_t *t = get(handle);
  if (!t) return;
  if (!t->trace_window.active(cycle, pc)) return;
  t->n_traps++;
  ensure_header(t);

  const decoded_t d = rv_decode(insn, t->xlen, t->extensions);

  trap_info_t info;
  info.cycle = cycle;
  info.sim_time_ns = sim_time_ns;
  info.pc = pc;
  info.cause = cause;
  info.tval = tval;
  info.insn = insn;
  info.priv = priv;
  info.is_interrupt = is_interrupt;
  info.dec = &d;

  t->fmt->trap(t->f, info);
  if (!t->stats_path.empty()) t->stats.record(info, rv_cause_name(cause, t->xlen));
}

extern "C" unsigned long long cva6tb_tracer_count(int handle) {
  tracer_t *t = get(handle);
  return t ? t->n_retired : 0;
}

extern "C" void cva6tb_tracer_close(int handle) {
  if (handle < 0 || (size_t)handle >= g_tracers.size()) return;
  tracer_t *t = g_tracers[handle];
  if (!t || !t->f) return;
  ensure_header(t);  // a run that traced nothing still gets a well formed file
  fflush(t->f);
  fclose(t->f);
  t->f = nullptr;
  delete t->fmt;
  t->fmt = nullptr;

  if (!t->stats_path.empty() && !t->stats.empty()) {
    FILE *sf = fopen(t->stats_path.c_str(), "w");
    if (sf) {
      t->stats.report(sf, t->fcfg);
      fclose(sf);
      printf("*** [cva6tb_tracer] INFO: statistics written to '%s'\n", t->stats_path.c_str());
    } else {
      fprintf(stderr, "*** [cva6tb_tracer] ERROR: cannot open '%s' for writing\n",
              t->stats_path.c_str());
    }
  }
  if (t->n_seen != t->n_retired)
    printf("*** [cva6tb_tracer] INFO: %llu instructions traced of %llu retired, %llu traps\n",
           (unsigned long long)t->n_retired, (unsigned long long)t->n_seen,
           (unsigned long long)t->n_traps);
  else
    printf("*** [cva6tb_tracer] INFO: %llu instructions, %llu traps traced\n",
           (unsigned long long)t->n_retired, (unsigned long long)t->n_traps);
}
