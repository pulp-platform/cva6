// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// Runs a program on the reference model alone, with no RTL and no simulator.
//
// This exists because every co-simulation bug looks the same from inside a
// simulation: the reference and the DUT disagree. Here there is no DUT, so an
// ISA string that is missing an extension, an address map that has drifted from
// cva6tb_pkg.sv, or an image loaded at the wrong address fails on its own, in a
// second, with a stack trace if it comes to that. It stays useful long after
// bring-up for exactly that reason: it is what tells a co-simulation bug from an
// RTL bug.
//
//   rv_cosim_standalone <image.hex> --bootrom <bootrom.bin> [options]

#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "rv_memif.h"
#include "rv_ref_spike.h"

using namespace cva6tb_cosim;

namespace {

constexpr const char *kDefaultIsa =
    "rv64gc_zba_zbb_zbs_zbc_zbkc_zcb_zicond_zicntr_zihpm";

struct options {
  std::string image;
  std::string bootrom;
  // Matches cv64a6_imafdc_sv39_wb: rv64gc is imafdc plus Zicsr and Zifencei,
  // and that config adds B -- Zba, Zbb, Zbs, Zbc and Zbkc -- plus Zcb, Zicond,
  // Zicntr and Zihpm. It is what rv_isa_string() derives for that
  // configuration, and the two have to agree.
  // This has to agree with the DUT too: naming an extension the hardware does
  // not implement makes the reference execute what the DUT traps on, and leaving
  // one out does the reverse. Use --isa for any other configuration -- deriving
  // it from the configuration is the co-simulation's job, where the config is
  // known.
  std::string isa = kDefaultIsa;
  std::string priv = "MSU";
  uint64_t max_insns = 10'000'000;
  std::string spike_log = "/dev/null";
  // Sv39 by default, matching every 64-bit CVA6 configuration with S-mode.
  unsigned vaddr_bits = 39;
  bool verbose = false;
};

void usage(const char *argv0) {
  fprintf(stderr,
          "usage: %s <image.hex> [options]\n"
          "\n"
          "  --bootrom <file>   raw boot ROM image, loaded at 0x%08llx\n"
          "  --isa <string>     spike ISA string      (default %s)\n"
          "  --priv <string>    privilege modes       (default MSU)\n"
          "  --max-insns <n>    give up after n instructions (default 10000000)\n"
          "  --spike-log <file> write spike's own commit log here\n"
          "  --vaddr-bits <n>   39 for Sv39, 32 for Sv32, 0 for bare (default 39)\n"
          "  --verbose          print each instruction's PC\n",
          argv0, (unsigned long long)rv_addr_map::kBootromBase, kDefaultIsa);
}

bool parse(int argc, char **argv, options *o) {
  for (int i = 1; i < argc; i++) {
    const std::string a = argv[i];
    const bool has_next = (i + 1) < argc;
    if (a == "--bootrom" && has_next) {
      o->bootrom = argv[++i];
    } else if (a == "--isa" && has_next) {
      o->isa = argv[++i];
    } else if (a == "--priv" && has_next) {
      o->priv = argv[++i];
    } else if (a == "--max-insns" && has_next) {
      o->max_insns = strtoull(argv[++i], nullptr, 0);
    } else if (a == "--spike-log" && has_next) {
      o->spike_log = argv[++i];
    } else if (a == "--vaddr-bits" && has_next) {
      o->vaddr_bits = (unsigned)strtoul(argv[++i], nullptr, 0);
    } else if (a == "--verbose") {
      o->verbose = true;
    } else if (a == "-h" || a == "--help") {
      return false;
    } else if (!a.empty() && a[0] == '-') {
      fprintf(stderr, "unknown option '%s'\n", a.c_str());
      return false;
    } else if (o->image.empty()) {
      o->image = a;
    } else {
      fprintf(stderr, "unexpected argument '%s'\n", a.c_str());
      return false;
    }
  }
  return !o->image.empty();
}

}  // namespace

int main(int argc, char **argv) {
  options o;
  if (!parse(argc, argv, &o)) {
    usage(argv[0]);
    return 2;
  }

  rv_memif mem;
  std::string err;

  if (!o.bootrom.empty() && !mem.load_bin(o.bootrom, rv_addr_map::kBootromBase, &err)) {
    fprintf(stderr, "error: boot ROM: %s\n", err.c_str());
    return 1;
  }
  if (!mem.load_hex(o.image, &err)) {
    fprintf(stderr, "error: image: %s\n", err.c_str());
    return 1;
  }

  // The testbench decides where the image went from the first address in the
  // hex file, and reports that decision through the boot-mode register, which
  // the boot ROM reads to pick its jump target. Model the same decision so the
  // reference boots the way the DUT does.
  rv_sim_devices devices(mem.image_in_spm());
  mem.set_mmio_supplier(&devices);

  printf("image  : %s (loaded at 0x%016" PRIx64 ", %s)\n", o.image.c_str(), mem.image_base(),
         mem.image_in_spm() ? "SPM, boot mode 1" : "DRAM, boot mode 0");
  printf("isa    : %s / %s\n", o.isa.c_str(), o.priv.c_str());

  rv_ref_spike ref(mem, o.isa, o.priv, rv_addr_map::kBootromBase, o.vaddr_bits, o.spike_log);

  rv_ref_effects eff;
  uint64_t n = 0;
  for (; n < o.max_insns && !devices.finished(); n++) {
    const uint64_t pc = ref.pc();
    if (!ref.step(eff)) {
      fprintf(stderr, "error: reference could not step at pc 0x%016" PRIx64 "\n", pc);
      return 1;
    }
    if (o.verbose) {
      printf("%8" PRIu64 "  0x%016" PRIx64 "  (0x%08x)\n", n, eff.pc, eff.insn);
    }
  }

  printf("\n%" PRIu64 " instructions, final pc 0x%016" PRIx64 "\n", n, ref.pc());
  if (mem.unmapped() != 0) {
    printf("%" PRIu64 " accesses fell outside every region\n", mem.unmapped());
  }

  if (!devices.finished()) {
    printf("FAILED: did not reach end of computation within %" PRIu64 " instructions\n",
           o.max_insns);
    return 1;
  }

  printf("end of computation, return code %u\n", devices.return_code());
  return devices.return_code() == 0 ? 0 : 1;
}
