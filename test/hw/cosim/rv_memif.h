// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// The memory the reference model sees: spike's simif_t, backed by one mem_t per
// RAM region of the testbench, with everything else routed to MMIO.
//
// Spike owns this copy, and it is kept in step by replaying every retired
// store. A load whose value spike computes from its own memory catches a stale
// cache line or a bad store-buffer forward.

#ifndef CVA6TB_COSIM_RV_MEMIF_H
#define CVA6TB_COSIM_RV_MEMIF_H

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <riscv/cfg.h>
#include <riscv/devices.h>
#include <riscv/simif.h>

namespace cva6tb_cosim {

// The testbench address map, from test/hw/cva6tb_pkg.sv, mirrored by hand: it
// is a dozen constants, and a mismatch shows up immediately as a fetch from
// unmapped memory.
struct rv_addr_map {
  static constexpr uint64_t kBootromBase = 0x0000'0000'0001'0000ull;
  static constexpr uint64_t kBootromSize = 0x0000'1000ull;
  static constexpr uint64_t kSpmBase = 0x0000'0000'2000'0000ull;
  static constexpr uint64_t kSpmSize = 0x0010'0000ull;
  static constexpr uint64_t kDramBase = 0x0000'0000'8000'0000ull;
  static constexpr uint64_t kDramSize = 0x1000'0000ull;
  // The uncached alias is remapped onto the cached region before it reaches the
  // memory (cva6tb_soc.sv: addr = DramBase | (addr & DramUncachedMask)), so both
  // views must hit the same bytes here too.
  static constexpr uint64_t kDramAliasBase = 0x0000'0000'4000'0000ull;
  static constexpr uint64_t kDramAliasSize = 0x1000'0000ull;
  static constexpr uint64_t kDramAliasMask = 0x3fff'ffffull;

  // MMIO, listed for the standalone device model and for reporting.
  static constexpr uint64_t kPcrBase = 0x1000'0000ull;      // boot mode, EOC
  static constexpr uint64_t kConsoleBase = 0x1000'1000ull;  // character sink
  static constexpr uint64_t kClintBase = 0x0204'0000ull;
  static constexpr uint64_t kPlicBase = 0x0c00'0000ull;
  static constexpr uint64_t kRtcTimerBase = 0x1000'2000ull;
  static constexpr uint64_t kClicBase = 0x1004'0000ull;
};

// Supplies the data for an access that is not backed by RAM. In lockstep this
// is the value RVFI observed, so the instruction is still *executed* by the
// reference -- address, size and sign extension all stay checked, and only the
// data byte comes from the DUT. Standalone, it is the small device model below.
class rv_mmio_supplier {
 public:
  virtual ~rv_mmio_supplier() = default;
  virtual bool load(uint64_t paddr, size_t len, uint8_t *bytes) = 0;
  virtual bool store(uint64_t paddr, size_t len, const uint8_t *bytes) = 0;
};

// Just enough of the testbench peripherals to let a program boot and finish
// without any RTL: the boot-mode register the boot ROM reads to decide where to
// jump, the console it prints through, and the EOC register it parks on. This
// is for the standalone tool only -- in lockstep the DUT supplies these values,
// and this model is not installed.
class rv_sim_devices : public rv_mmio_supplier {
 public:
  explicit rv_sim_devices(bool boot_from_spm) : boot_mode_(boot_from_spm ? 1 : 0) {}

  bool load(uint64_t paddr, size_t len, uint8_t *bytes) override;
  bool store(uint64_t paddr, size_t len, const uint8_t *bytes) override;

  bool finished() const { return eoc_valid_; }
  // Return code the program reported, valid once finished().
  uint32_t return_code() const { return eoc_code_; }

 private:
  uint32_t boot_mode_ = 0;
  bool eoc_valid_ = false;
  uint32_t eoc_code_ = 0;
  std::string console_;
};

class rv_memif : public simif_t {
 public:
  rv_memif();
  ~rv_memif() override = default;

  // --- construction -------------------------------------------------------

  // Loads a Verilog hex image as written by `objcopy -O verilog`: `@<byte
  // address>` lines followed by whitespace separated hex bytes. That is exactly
  // what the testbench feeds $readmemh into axi_sim_mem's `logic [7:0]
  // mem[addr_t]`, so both sides see the same bytes at the same addresses.
  // Returns false and fills `error` if the file cannot be read or parsed.
  bool load_hex(const std::string &path, std::string *error);

  // Loads a raw binary at `base`, for the boot ROM image.
  bool load_bin(const std::string &path, uint64_t base, std::string *error);

  // First address seen in the last hex image, which is how the testbench picks
  // between SPM and DRAM, and therefore what the boot-mode register must report.
  uint64_t image_base() const { return image_base_; }
  bool image_in_spm() const { return image_base_ == rv_addr_map::kSpmBase; }

  // Whether an address is backed by memory here. The policy needs this to tell
  // a device from RAM without keeping a second copy of the address map.
  bool is_ram(uint64_t paddr) const;

  void set_mmio_supplier(rv_mmio_supplier *s) { mmio_ = s; }
  void add_hart(size_t id, processor_t *p) { harts_[id] = p; }

  // --- simif_t ------------------------------------------------------------

  char *addr_to_mem(reg_t paddr) override;
  bool mmio_load(reg_t paddr, size_t len, uint8_t *bytes) override;
  bool mmio_store(reg_t paddr, size_t len, const uint8_t *bytes) override;
  void proc_reset(unsigned id) override { (void)id; }
  const cfg_t &get_cfg() const override { return cfg_; }
  const std::map<size_t, processor_t *> &get_harts() const override { return harts_; }
  const char *get_symbol(uint64_t paddr) override;

  cfg_t &cfg() { return cfg_; }

  // --- store replay -------------------------------------------------------

  // Applies a retired store to the reference's copy. Used when a store has to
  // be reproduced without executing it, which is what a `skip` rule does.
  bool write(uint64_t paddr, size_t len, const uint8_t *bytes);

  // Number of accesses that fell outside every region. Non-zero means the map
  // here and the map in cva6tb_pkg.sv have drifted apart.
  uint64_t unmapped() const { return n_unmapped_; }

 private:
  struct region {
    uint64_t base;
    uint64_t size;
    uint64_t mask;   // alias folding; 0 means none
    mem_t *mem;      // where the bytes actually live
    const char *name;
  };

  // Resolves an address to its region and the offset within the backing store,
  // folding the uncached alias onto the cached region on the way.
  const region *find(uint64_t paddr, uint64_t *offset) const;

  std::unique_ptr<mem_t> bootrom_;
  std::unique_ptr<mem_t> spm_;
  std::unique_ptr<mem_t> dram_;
  std::vector<region> regions_;

  cfg_t cfg_;
  std::map<size_t, processor_t *> harts_;
  rv_mmio_supplier *mmio_ = nullptr;

  uint64_t image_base_ = 0;
  uint64_t n_unmapped_ = 0;
};

}  // namespace cva6tb_cosim

#endif  // CVA6TB_COSIM_RV_MEMIF_H
