// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// Basic testbench pre-loading a binary and polling the EOC register for success.
// Times out after a specified number of cycles.

module test;

  import ariane_pkg::*;
  import cva6tb_pkg::*;
  import cva6tb_regs_pkg::EOCRegOffset;

  // localparam time         ClkPeriod = 1ns;
  // localparam time         RtcPeriod = 1us;
  localparam int unsigned ClkPeriodPs = 1000;
  localparam int unsigned RtcPeriodPs = 1000000;
  localparam time         ApplDelay = (ClkPeriodPs * 1ps) * 0.1;
  localparam time         AcqDelay  = (ClkPeriodPs * 1ps) * 0.9;
  localparam int unsigned RstCycles = 10;
  localparam time         ClkPeriod = ClkPeriodPs * 1ps;

  int unsigned RetCodeSuccess;
  int unsigned MaxCycles;
  int unsigned IrqPeriod;
  bit          InterruptsEnabled;

  logic bootmode;

  function automatic void parse_args();
    if (!$value$plusargs("RetCodeSuccess=%d",    RetCodeSuccess))       RetCodeSuccess = 0;
    if (!$value$plusargs("MaxCycles=%d",         MaxCycles))                 MaxCycles = 10_000;
    if (!$value$plusargs("IrqPeriod=%d",         IrqPeriod))                 IrqPeriod = 100_000;
    if (!$value$plusargs("InterruptsEnabled=%b", InterruptsEnabled)) InterruptsEnabled = 1'b0;
  endfunction

  logic        clk;
  logic        rst_n;
  logic        rtc;
  logic        eoc;
  logic [30:0] retcode;

  // Parameters for interrupt generation
  localparam int unsigned IrqId          = 1;
  localparam int unsigned NumInterrupts  = 50;
  localparam int unsigned IrqHighCycles  = 10;

  assign eoc     = i_dut.i_regs.control_regs[EOCRegOffset][0];
  assign retcode = i_dut.i_regs.control_regs[EOCRegOffset][31:1];

  logic [NumClicExtIrqs-1:0] clic_ext_irqs;
  logic                [7:0] clic_irq_id;
  logic                      clic_irq_valid;
  logic                      clic_irq_ready;
  logic                      eret;
  logic                      commit_valid;
  logic               [63:0] commit_pc;
  logic               [63:0] exception_pc;
  event                      irq_start_log;
  event                      irq_start_gen;

  assign clic_irq_valid = i_dut.clic_irq_valid;
  assign clic_irq_ready = i_dut.clic_irq_ready;
  assign eret           = i_dut.i_cva6.eret;
  assign commit_valid   = i_dut.i_cva6.commit_instr_id_commit[0].valid & i_dut.i_cva6.commit_ack_commit_id[0];
  assign commit_pc      = i_dut.i_cva6.pc_commit;
  assign exception_pc   = i_dut.i_cva6.commit_stage_i.commit_instr_i[0].pc;

  // Interrupt generation
  initial begin : irq_gen
    automatic int count = 1;
    clic_ext_irqs = '0;
    wait(irq_start_gen);
    #(ClkPeriod * (1000000 - IrqPeriod)); // Initial delay before first interrupt
    forever begin
      #(ClkPeriod * (IrqPeriod-IrqHighCycles));
      log($sformatf("Generating interrupt %0d", IrqId));
      #ApplDelay;
      clic_ext_irqs[IrqId] = 1'b1;
      #(ClkPeriod - ApplDelay);
      #(ClkPeriod * (IrqHighCycles - 1));
      clic_ext_irqs[IrqId] = 1'b0;
      if (count == NumInterrupts) break;
      count += 1;
    end
    #(ClkPeriod * 100000);
    log("**Interrupt generation finished**");
    cleanup;
  end

  cva6tb_irq_log #(
    .NumIrqs        ( NumClicExtIrqs ),
    .FirstInstAddr  ( 64'h8000_0000  )
  ) i_irq_log (
    .clk_i          ( clk            ),
    .rst_ni         ( rst_n          ),
    .start_log_i    ( irq_start_log  ),
    .irqs_i         ( clic_ext_irqs  ),
    .commit_valid_i ( commit_valid   ),
    .commit_pc_i    ( commit_pc      ),
    .exception_pc_i ( exception_pc   ),
    .irq_id_i       ( clic_irq_id    ),
    .irq_valid_i    ( clic_irq_valid ),
    .irq_ack_i      ( clic_irq_ready ),
    .eret_i         ( eret           )
  );

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
    .ApplDelay       ( ApplDelay     ),
    .AcqDelay        ( AcqDelay      )
  ) i_dut (
    .clk_i           ( clk           ),
    .rst_ni          ( rst_n         ),
    .rtc_i           ( rtc           ),
    .boot_mode_i     ( bootmode      ),
    .clic_ext_irqs_i ( clic_ext_irqs ),
    .plic_ext_irqs_i ( '0            )
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
    setup();
    // Parse command line arguments
    parse_args();
    // Preload binary into simulation memory and set bootmode
    preload_hex();
    // Wait for reset de-assertion
    wait(rst_n);
    // Start logging interrupts
    if (InterruptsEnabled) -> irq_start_log;
    if (InterruptsEnabled) -> irq_start_gen;
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
