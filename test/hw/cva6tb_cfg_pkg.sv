// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// The configuration of CVA6 in the testbench SoC: a configuration package's
// user configuration with the memory attributes of the SoC's address map. The
// testbench (cva6tb_pkg) and the ASIC implementation wrapper (asic/,
// cva6_asic_pkg) both build it, so that the implemented core is the one the
// testbench runs.

package cva6tb_cfg_pkg;

  // The memory attributes describe this SoC's address map, not the core, so they
  // are set here for every configuration. They go into the user configuration,
  // before build_config, because it derives NonIdemPotenceEn from them.
  function automatic config_pkg::cva6_cfg_t gen_cva6_cfg(config_pkg::cva6_user_cfg_t user_cfg);
    config_pkg::cva6_user_cfg_t cfg = user_cfg;
    // Everything below the SPM -- the error slave and the regbus peripherals -- is
    // device space: stores there must not be merged, nor loads issued speculatively.
    cfg.NrNonIdempotentRules  = unsigned'(1);
    cfg.NonIdempotentAddrBase = 1024'({64'h0000_0000});
    cfg.NonIdempotentLength   = 1024'({64'h2000_0000});
    cfg.NrExecuteRegionRules  = unsigned'(4);
    cfg.ExecuteRegionAddrBase = 1024'({64'h8000_0000, 64'h2000_0000, 64'(56'h01A0_0000), 64'h0001_0000});
    cfg.ExecuteRegionLength   = 1024'({64'h4000_0000, 64'h0010_0000, 64'(56'h0000_4000), 64'h0001_0000});
    cfg.NrCachedRegionRules   = unsigned'(2);
    cfg.CachedRegionAddrBase  = 1024'({64'h8000_0000, 64'h2000_0000});
    cfg.CachedRegionLength    = 1024'({64'h4000_0000, 64'h0010_0000});
    return build_config_pkg::build_config(cfg);
  endfunction

endpackage
