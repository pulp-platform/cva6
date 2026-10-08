// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// LOAD-BEARING ASSUMPTION: CVA6 is the only master that can write RAM.
// cva6tb_pkg.sv:34 sets AxiXbarMasters = 1, i_cva6 is the crossbar's only slave
// port, debug_req_i is tied off and there is no DMA. The reference's copy of
// memory is therefore kept correct by replaying retired stores and nothing else.
// If a second master is ever added to the SoC, this copy will silently drift out
// of step with the DUT's, and every load after that point is suspect.

#include "rv_memif.h"

#include <cctype>
#include <cstdio>
#include <cstring>

namespace cva6tb_cosim {

namespace {

// Spike caches a host pointer per 4 KiB page, so a region is either all RAM or
// all MMIO at page granularity. Every base and size in the testbench map is 4 KiB
// aligned, which this asserts.
bool page_aligned(uint64_t v) { return (v & 0xfffull) == 0; }

int hex_digit(int c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

}  // namespace

// ---------------------------------------------------------------- devices ---

bool rv_sim_devices::load(uint64_t paddr, size_t len, uint8_t *bytes) {
  memset(bytes, 0, len);

  // Platform control registers are word indexed: 0 boot mode, 1 EOC.
  if (paddr >= rv_addr_map::kPcrBase && paddr < rv_addr_map::kPcrBase + 0x1000) {
    const uint64_t word = (paddr - rv_addr_map::kPcrBase) >> 2;
    const uint32_t v = (word == 0) ? boot_mode_ : (word == 1 ? eoc_code_ : 0);
    const size_t n = len < sizeof(v) ? len : sizeof(v);
    memcpy(bytes, &v, n);
    return true;
  }

  // Everything else reads as zero. A real peripheral read would be a gap in the
  // standalone model, not in the co-simulation, where the DUT supplies it.
  return true;
}

bool rv_sim_devices::store(uint64_t paddr, size_t len, const uint8_t *bytes) {
  if (paddr >= rv_addr_map::kPcrBase && paddr < rv_addr_map::kPcrBase + 0x1000) {
    const uint64_t word = (paddr - rv_addr_map::kPcrBase) >> 2;
    if (word == 1) {
      uint32_t v = 0;
      memcpy(&v, bytes, len < sizeof(v) ? len : sizeof(v));
      // bit 0 signals end of computation, bits 31:1 carry the return code
      if (v & 1u) {
        eoc_valid_ = true;
        eoc_code_ = v >> 1;
      }
    }
    return true;
  }

  // Console: byte writes to offset 0, flushed on newline, same as
  // cva6tb_sim_console.sv.
  if (paddr >= rv_addr_map::kConsoleBase && paddr < rv_addr_map::kConsoleBase + 0x1000) {
    if (((paddr - rv_addr_map::kConsoleBase) & 0x1f) == 0 && len > 0) {
      const char c = (char)bytes[0];
      if (c == '\n') {
        printf("[CON] %s\n", console_.c_str());
        fflush(stdout);
        console_.clear();
      } else {
        console_.push_back(c);
      }
    }
    return true;
  }

  return true;
}

// ----------------------------------------------------------------- memif ----

rv_memif::rv_memif() {
  bootrom_ = std::make_unique<mem_t>(rv_addr_map::kBootromSize);
  spm_ = std::make_unique<mem_t>(rv_addr_map::kSpmSize);
  dram_ = std::make_unique<mem_t>(rv_addr_map::kDramSize);

  regions_ = {
      {rv_addr_map::kBootromBase, rv_addr_map::kBootromSize, 0, bootrom_.get(), "bootrom"},
      {rv_addr_map::kSpmBase, rv_addr_map::kSpmSize, 0, spm_.get(), "spm"},
      {rv_addr_map::kDramBase, rv_addr_map::kDramSize, 0, dram_.get(), "dram"},
      {rv_addr_map::kDramAliasBase, rv_addr_map::kDramAliasSize, rv_addr_map::kDramAliasMask,
       dram_.get(), "dram (uncached alias)"},
  };

  for (const auto &r : regions_) {
    if (!page_aligned(r.base) || !page_aligned(r.size)) {
      fprintf(stderr, "*** [cva6tb_cosim] ERROR: region '%s' is not 4 KiB aligned\n", r.name);
    }
  }

  // The reference is one hart, machine through user, with spike's own defaults
  // for everything the testbench does not constrain. The ISA string is set by
  // the caller, which derives it from the CVA6 configuration.
  cfg_.hartids = {0};
  cfg_.explicit_hartids = true;
  cfg_.start_pc.set_global(rv_addr_map::kBootromBase);
  cfg_.priv = "MSU";
  // CVA6ConfigDcacheLineWidth is 128 bits on the bring-up configuration, and
  // Zicbom's cbo.* operate on that granularity.
  cfg_.cache_blocksz = 16;
}

const rv_memif::region *rv_memif::find(uint64_t paddr, uint64_t *offset) const {
  for (const auto &r : regions_) {
    if (paddr < r.base || paddr >= r.base + r.size) continue;
    // An alias folds onto the region it shadows; everything else is a plain
    // offset from its own base.
    *offset = r.mask != 0 ? (paddr & r.mask) : (paddr - r.base);
    return &r;
  }
  return nullptr;
}

bool rv_memif::is_ram(uint64_t paddr) const {
  uint64_t off = 0;
  return find(paddr, &off) != nullptr;
}

char *rv_memif::addr_to_mem(reg_t paddr) {
  uint64_t off = 0;
  const region *r = find(paddr, &off);
  // Returning nullptr is what routes the access to mmio_load/mmio_store, so
  // every address outside RAM lands there by construction.
  if (r == nullptr) return nullptr;
  return r->mem->contents(off);
}

bool rv_memif::mmio_load(reg_t paddr, size_t len, uint8_t *bytes) {
  if (mmio_ != nullptr) return mmio_->load(paddr, len, bytes);
  n_unmapped_++;
  memset(bytes, 0, len);
  return true;
}

bool rv_memif::mmio_store(reg_t paddr, size_t len, const uint8_t *bytes) {
  if (mmio_ != nullptr) return mmio_->store(paddr, len, bytes);
  n_unmapped_++;
  return true;
}

const char *rv_memif::get_symbol(uint64_t paddr) {
  // Symbol lookup is the tracer's job (rv_symbols), and spike only uses this for
  // its own log messages, which this backend does not produce.
  (void)paddr;
  return nullptr;
}

bool rv_memif::write(uint64_t paddr, size_t len, const uint8_t *bytes) {
  uint64_t off = 0;
  const region *r = find(paddr, &off);
  if (r == nullptr) {
    n_unmapped_++;
    return false;
  }
  return r->mem->store(off, len, bytes);
}

// ---------------------------------------------------------------- loading ---

bool rv_memif::load_bin(const std::string &path, uint64_t base, std::string *error) {
  FILE *f = fopen(path.c_str(), "rb");
  if (f == nullptr) {
    if (error != nullptr) *error = "cannot open '" + path + "'";
    return false;
  }

  uint8_t buf[4096];
  uint64_t addr = base;
  size_t n = 0;
  bool ok = true;
  while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
    if (!write(addr, n, buf)) {
      if (error != nullptr) *error = "'" + path + "' does not fit the address map";
      ok = false;
      break;
    }
    addr += n;
  }
  fclose(f);
  return ok;
}

bool rv_memif::load_hex(const std::string &path, std::string *error) {
  FILE *f = fopen(path.c_str(), "r");
  if (f == nullptr) {
    if (error != nullptr) *error = "cannot open '" + path + "'";
    return false;
  }

  uint64_t addr = 0;
  bool have_base = false;
  // Bytes are written one at a time: the images are small and this runs once
  // per simulation.
  uint8_t byte = 0;
  int hi = -1;
  int c = 0;
  bool ok = true;

  while ((c = fgetc(f)) != EOF) {
    if (c == '@') {
      // Address line. A half-assembled byte before it means a malformed file,
      // which is reported as one.
      if (hi >= 0) {
        if (error != nullptr) *error = "'" + path + "': odd hex digit before an address";
        ok = false;
        break;
      }
      uint64_t a = 0;
      bool any = false;
      while ((c = fgetc(f)) != EOF && hex_digit(c) >= 0) {
        a = (a << 4) | (uint64_t)hex_digit(c);
        any = true;
      }
      if (!any) {
        if (error != nullptr) *error = "'" + path + "': '@' without an address";
        ok = false;
        break;
      }
      addr = a;
      if (!have_base) {
        image_base_ = a;
        have_base = true;
      }
      continue;
    }

    const int d = hex_digit(c);
    if (d < 0) {
      if (isspace(c)) continue;
      if (error != nullptr) {
        *error = "'" + path + "': unexpected character '" + std::string(1, (char)c) + "'";
      }
      ok = false;
      break;
    }

    if (hi < 0) {
      hi = d;
    } else {
      byte = (uint8_t)((hi << 4) | d);
      hi = -1;
      if (!write(addr, 1, &byte)) {
        if (error != nullptr) {
          char msg[128];
          snprintf(msg, sizeof(msg), "'%s': address 0x%llx is not in any RAM region",
                   path.c_str(), (unsigned long long)addr);
          *error = msg;
        }
        ok = false;
        break;
      }
      addr++;
    }
  }

  if (ok && hi >= 0) {
    if (error != nullptr) *error = "'" + path + "': trailing odd hex digit";
    ok = false;
  }
  if (ok && !have_base) {
    if (error != nullptr) *error = "'" + path + "': no '@address' line";
    ok = false;
  }

  fclose(f);
  return ok;
}

}  // namespace cva6tb_cosim
