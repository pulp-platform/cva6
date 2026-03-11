// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>

package cva6tb_pkg;

  `include "axi/typedef.svh"
  `include "register_interface/typedef.svh"
  `include "rvfi_types.svh"

  import ariane_pkg::*;

  // CVA6 configuration struct
  localparam config_pkg::cva6_user_cfg_t CVA6UserCfg = cva6_config_pkg::cva6_cfg;
  localparam config_pkg::cva6_cfg_t          CVA6Cfg = build_config_pkg::build_config(CVA6UserCfg);

  ////////////////////////////
  // Test SoC configuration //
  ////////////////////////////

  localparam int unsigned AxiXbarMasters   = 1;
  localparam int unsigned AxiXbarSlaves    = 3;
  localparam int unsigned AxiSlvIdWidth    = CVA6UserCfg.AxiIdWidth + $clog2(AxiXbarMasters);
  localparam int unsigned AxiXbarAddrRules = 4;
  localparam int unsigned AxiSlvIdErrSlv   = 0;
  localparam int unsigned AxiSlvIdReg      = 1;
  localparam int unsigned AxiSlvIdDram     = 2;

  localparam int unsigned RegbusOutNum     = 4;
  localparam int unsigned RegbusRulesNum   = 4;
  localparam int unsigned RegbusErrId      = 0;
  localparam int unsigned RegbusPCRsId     = 1;
  localparam int unsigned RegbusConsoleId  = 2;
  localparam int unsigned RegbusClicId     = 3;

  localparam int unsigned NumClicIrqs      = 256;

  // AXI xbar configuration
  localparam axi_pkg::xbar_cfg_t AxiXbarCfg = '{
    NoSlvPorts:         AxiXbarMasters,
    NoMstPorts:         AxiXbarSlaves,
    MaxMstTrans:        24,
    MaxSlvTrans:        24,
    FallThrough:        0,
    LatencyMode:        axi_pkg::CUT_ALL_PORTS,
    PipelineStages:     0,
    AxiIdWidthSlvPorts: CVA6UserCfg.AxiIdWidth,
    AxiIdUsedSlvPorts:  CVA6UserCfg.AxiIdWidth,
    UniqueIds:          0,
    AxiAddrWidth:       CVA6UserCfg.AxiAddrWidth,
    AxiDataWidth:       CVA6UserCfg.AxiDataWidth,
    NoAddrRules:        AxiXbarAddrRules,
    default:            '0
  };

  //////////////////////
  // Type definitions //
  //////////////////////

  localparam type rvfi_instr_t        = `RVFI_INSTR_T(CVA6Cfg);
  localparam type rvfi_csr_elmt_t     = `RVFI_CSR_ELMT_T(CVA6Cfg);
  localparam type rvfi_csr_t          = `RVFI_CSR_T(CVA6Cfg, rvfi_csr_elmt_t);
  localparam type rvfi_probes_instr_t = `RVFI_PROBES_INSTR_T(CVA6Cfg);
  localparam type rvfi_probes_csr_t   = `RVFI_PROBES_CSR_T(CVA6Cfg);
  localparam type rvfi_probes_t       = struct packed {
    rvfi_probes_csr_t   csr;
    rvfi_probes_instr_t instr;
  };

  localparam type axi_addr_t   = logic [CVA6UserCfg.AxiAddrWidth-1:0];
  localparam type axi_data_t   = logic [CVA6UserCfg.AxiDataWidth-1:0];
  localparam type axi_strb_t   = logic [CVA6UserCfg.AxiDataWidth/8-1:0];
  localparam type axi_mst_id_t = logic [CVA6UserCfg.AxiIdWidth-1:0];
  localparam type axi_slv_id_t = logic [AxiSlvIdWidth-1:0];
  localparam type axi_user_t   = logic [CVA6UserCfg.AxiUserWidth-1:0];

  `AXI_TYPEDEF_ALL(axi_mst, axi_addr_t, axi_mst_id_t, axi_data_t, axi_strb_t, axi_user_t);
  `AXI_TYPEDEF_ALL(axi_slv, axi_addr_t, axi_slv_id_t, axi_data_t, axi_strb_t, axi_user_t);
  typedef axi_mst_resp_t axi_mst_rsp_t;
  typedef axi_slv_resp_t axi_slv_rsp_t;

  localparam type reg_addr_t = logic [31:0];
  localparam type reg_idx_t  = logic [cf_math_pkg::idx_width(RegbusOutNum)-1:0];

  `REG_BUS_TYPEDEF_ALL(reg, reg_addr_t, logic [31:0], logic [3:0])

  // Type for address map entries
  typedef struct packed {
    logic [5:0] idx;
    axi_addr_t  start_addr;
    axi_addr_t  end_addr;
  } addr_rule_t;

  // Type for regbus address map entries
  typedef struct packed {
    logic [5:0] idx;
    reg_addr_t  start_addr;
    reg_addr_t  end_addr;
  } reg_addr_rule_t;

  //////////////////////////
  // Test SoC address map //
  //////////////////////////

  // AXI error slave region
  localparam axi_addr_t AxiErrBaseAddr       = CVA6UserCfg.AxiAddrWidth'(64'h0000_0000);
  localparam axi_addr_t AxiErrSize           = CVA6UserCfg.AxiAddrWidth'(64'h1000_0000);
  // Regbus region
  localparam axi_addr_t RegbusBaseAddr       = CVA6UserCfg.AxiAddrWidth'(64'h1000_0000);
  localparam axi_addr_t RegbusSize           = CVA6UserCfg.AxiAddrWidth'(64'h0010_0000);
  // DRAM region (cached range)
  localparam axi_addr_t DramBaseAddr         = CVA6UserCfg.AxiAddrWidth'(64'h8000_0000);
  localparam axi_addr_t DramSize             = CVA6UserCfg.AxiAddrWidth'(64'h1000_0000);
  // Uncached alias of the DRAM region
  localparam axi_addr_t DramUncachedBaseAddr = CVA6UserCfg.AxiAddrWidth'(64'h4000_0000);
  localparam axi_addr_t DramUncachedSize     = CVA6UserCfg.AxiAddrWidth'(64'h1000_0000);
  localparam axi_addr_t DramUncachedMask     = CVA6UserCfg.AxiAddrWidth'(64'h3FFF_FFFF);

  // Regbus error slave region
  localparam reg_addr_t RegErrBaseAddr  = 32'h0000_0000;
  localparam reg_addr_t RegErrSize      = 32'h1000_0000;
  // Platform Control Registers region
  localparam reg_addr_t PCRsBaseAddr    = 32'h1000_0000;
  localparam reg_addr_t PCRsSize        = 32'h0000_1000;
  // Sim console region
  localparam reg_addr_t ConsoleBaseAddr = 32'h1000_1000;
  localparam reg_addr_t ConsoleSize     = 32'h0000_1000;
  // CLIC region
  localparam reg_addr_t ClicBaseAddr    = 32'h1004_0000;
  localparam reg_addr_t ClicSize        = 32'h0003_0000;

  localparam addr_rule_t [AxiXbarAddrRules-1:0] AxiMap = {
    addr_rule_t'{ idx: AxiSlvIdErrSlv, start_addr: AxiErrBaseAddr,       end_addr: (AxiErrBaseAddr + AxiErrSize)             }, // AXI error slave
    addr_rule_t'{ idx: AxiSlvIdReg,    start_addr: RegbusBaseAddr,       end_addr: (RegbusBaseAddr + RegbusSize)             }, // Regbus
    addr_rule_t'{ idx: AxiSlvIdDram,   start_addr: DramBaseAddr,         end_addr: (DramBaseAddr + DramSize)                 }, // AXI Sim Mem
    addr_rule_t'{ idx: AxiSlvIdDram,   start_addr: DramUncachedBaseAddr, end_addr: (DramUncachedBaseAddr + DramUncachedSize) }  // AXI Sim Mem Alias
  };

  localparam reg_addr_rule_t [RegbusRulesNum-1:0] RegbusMap = {
    reg_addr_rule_t'{ idx: RegbusErrId,     start_addr: RegErrBaseAddr,  end_addr: (RegErrBaseAddr + RegErrSize)   }, // Regbus error slave
    reg_addr_rule_t'{ idx: RegbusPCRsId,    start_addr: PCRsBaseAddr,    end_addr: (PCRsBaseAddr + PCRsSize)       }, // Platform Control Registers
    reg_addr_rule_t'{ idx: RegbusConsoleId, start_addr: ConsoleBaseAddr, end_addr: (ConsoleBaseAddr + ConsoleSize) }, // Sim console
    reg_addr_rule_t'{ idx: RegbusClicId,    start_addr: ClicBaseAddr,    end_addr: (ClicBaseAddr + ClicSize)       }  // CLIC
  };

  //////////////////////
  // Helper functions //
  //////////////////////

  function automatic void log(string msg);
    $display("@%t | [TB ] %s", $realtime, msg);
  endfunction

  function automatic void setup();
    $display("===========================================");
    $display("======== CVA6 standalone testbench ========");
    $display("===========================================");
    // Time format
    $timeformat(-9, 0, "ns", 9); // 1: scale (ns=-9), 2: decimals, 3: suffix, 4: print-field width
    // Waveform logging
    `ifdef VERILATOR
    $dumpfile($sformatf("cva6tb.fst"));
    $dumpvars;
    `endif
  endfunction

  function automatic void cleanup();
    $display("===========================================");
    $display("============ End of simulation ============");
    $display("===========================================");
    `ifdef VERILATOR
    $dumpflush;
    `endif
    $finish;
  endfunction

  function automatic void pass();
    log("**SIMULATION PASSED**");
    cleanup();
  endfunction

  function automatic void fail();
    log("**SIMULATION FAILED**");
    cleanup();
  endfunction

endpackage
