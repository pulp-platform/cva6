// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// Basic testbench pre-loading a binary and polling the EOC register for success.
// Times out after a specified number of cycles.

module test #(
  // Tuning of the core, by default the configuration's
  parameter int unsigned          LoadPipeRegs = cva6_config_pkg::cva6_cfg.NrLoadPipeRegs,
  parameter config_pkg::bp_type_t BpType       = cva6_config_pkg::cva6_cfg.BPType,
  parameter int unsigned          BhtEntries   = cva6_config_pkg::cva6_cfg.BHTEntries,
  parameter int unsigned          BhtHist      = cva6_config_pkg::cva6_cfg.BHTHist,
  parameter int unsigned          RasDepth     = cva6_config_pkg::cva6_cfg.RASDepth,
  // Memory that was never written reads as zero, else as random data
  parameter bit                   ZeroSimMem   = 1'b0
);

  import ariane_pkg::*;
  import cva6tb_pkg::*;

  // The configuration with this SoC's platform parameters and the tuning above
  function automatic config_pkg::cva6_user_cfg_t tuned_cva6_cfg();
    config_pkg::cva6_user_cfg_t cfg = CVA6UserCfg;
    cfg.NrLoadPipeRegs = LoadPipeRegs;
    cfg.BPType         = BpType;
    cfg.BHTEntries     = BhtEntries;
    cfg.BHTHist        = BhtHist;
    cfg.RASDepth       = RasDepth;
    return cfg;
  endfunction

  localparam config_pkg::cva6_cfg_t CVA6Cfg = build_config_pkg::build_config(tuned_cva6_cfg());

  // localparam time         ClkPeriod = 1ns;
  // localparam time         RtcPeriod = 1us;
  localparam int unsigned ClkPeriodPs = 1000;
  localparam int unsigned RtcPeriodPs = 1000000;
  localparam int unsigned RstCycles = 10;

  int unsigned RetCodeSuccess;
  int unsigned MaxCycles;
  int unsigned MemDelay;

  logic bootmode;

  // empty -> no waveform dump
  string fstfile;

  function automatic void parse_args();
    if (!$value$plusargs("RetCodeSuccess=%d", RetCodeSuccess)) RetCodeSuccess = 0;
    if (!$value$plusargs("MaxCycles=%d",      MaxCycles))           MaxCycles = 10_000;
    if (!$value$plusargs("MemDelay=%d",       MemDelay))            MemDelay = 10;
    if (!$value$plusargs("fst=%s", fstfile)) begin
      if ($test$plusargs("fst")) fstfile = "cva6tb.fst";
    end
  endfunction

  logic        clk;
  logic        rst_n;
  logic        rtc;
  logic        eoc;
  logic [30:0] retcode;

  assign eoc     = i_dut.i_regs.eoc_o;
  assign retcode = i_dut.i_regs.retcode_o;

  cva6tb_clk_rst_gen #(
    .ClkPeriodPs  ( ClkPeriodPs ),
    .RstClkCycles ( RstCycles   )
  ) i_clk_rst_gen (
    .clk_o        ( clk         ),
    .rst_no       ( rst_n       )
  );

  cva6tb_clk_rst_gen #(
    .ClkPeriodPs  ( RtcPeriodPs ),
    .RstClkCycles ( 1           )
  ) i_rtc_gen (
    .clk_o        ( rtc         ),
    .rst_no       (             )
  );

  cva6tb_soc #(
    .CVA6Cfg    ( CVA6Cfg    ),
    .ZeroSimMem ( ZeroSimMem )
  ) i_dut (
    .clk_i           ( clk       ),
    .rst_ni          ( rst_n     ),
    .rtc_i           ( rtc       ),
    .boot_mode_i     ( bootmode  ),
    .plic_ext_irqs_i ( '0        )
  );

  task automatic preload_hex();
    string       binary_path;
    int          fd;
    string       line;
    logic [31:0] first_addr;

    if ($value$plusargs("binary=%s", binary_path)) begin
      log($sformatf("Running program \"%s\"", binary_path));

      fd = $fopen(binary_path, "r");
      if (fd == 0) begin
        log($sformatf("ERROR: cannot open \"%s\"", binary_path));
        return;
      end
      void'($fgets(line, fd));
      $fclose(fd);

      if (line.len() > 0 && line[0] == "@") begin
        void'($sscanf(line, "@%h", first_addr));
      end else begin
        first_addr = '0;
      end

      if (first_addr == 32'h2000_0000) begin
        bootmode = 1'b1;
        i_dut.load_sim_spm(binary_path);
      end else begin
        bootmode = 1'b0;
        i_dut.load_sim_mem(binary_path);
      end
    end else begin
      log("WARNING: no binary path provided");
    end
  endtask

  initial begin
    bootmode = 1'b0;
    // Parse command line arguments
    parse_args();
    setup(fstfile);
    log($sformatf("Core tuning: %0d load pipeline registers, %s with %0d entries and history %0d, RAS depth %0d",
                  CVA6Cfg.NrLoadPipeRegs, CVA6Cfg.BPType.name(), CVA6Cfg.BHTEntries, CVA6Cfg.BHTHist,
                  CVA6Cfg.RASDepth));
    // Preload binary into simulation memory and set bootmode
    preload_hex();
    // Main memory latency
    i_dut.set_mem_delay(MemDelay);
    // Wait for reset de-assertion
    wait(rst_n);
    // Poll EOC
    for (int unsigned i = 0; i < MaxCycles; ++i) begin
      @(posedge clk);
      if(eoc == 1'b1) begin
        log($sformatf("EOC register was set to one - return code: %0d", retcode));
        if (retcode == RetCodeSuccess) pass();
        else fail();
      end
    end
    log($sformatf("Simluation timed out after %d cycles", MaxCycles));
    fail();
  end

endmodule
