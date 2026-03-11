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

  localparam time         ClkPeriod = 2ns;
  localparam time         ApplDelay = ClkPeriod * 0.1;
  localparam time         AcqDelay  = ClkPeriod * 0.9;
  localparam int unsigned RstCycles = 10;

  int unsigned RetCodeSuccess;
  int unsigned MaxCycles;

  function automatic void parse_args();
    if (!$value$plusargs("RetCodeSccess=%d", RetCodeSuccess)) RetCodeSuccess = 0;
    if (!$value$plusargs("MaxCycles=%d",     MaxCycles))           MaxCycles = 10_000;
  endfunction

  logic        clk;
  logic        rst_n;
  logic        eoc;
  logic [30:0] retcode;

  assign eoc     = i_dut.i_regs.control_regs[EOCRegOffset][0];
  assign retcode = i_dut.i_regs.control_regs[EOCRegOffset][31:1];

  clk_rst_gen #(
    .ClkPeriod    ( ClkPeriod ),
    .RstClkCycles ( RstCycles )
  ) i_clk_rst_gen (
    .clk_o        ( clk       ),
    .rst_no       ( rst_n     )
  );

  cva6tb_soc #(
    .ApplDelay   ( ApplDelay ),
    .AcqDelay    ( AcqDelay  )
  ) i_dut (
    .clk_i       ( clk       ),
    .rst_ni      ( rst_n     ),
    .clic_irqs_i ( '0        )
  );

  initial begin
    setup();
    // Parse command line arguments
    parse_args();
    // Preload binary into simulation memory
    i_dut.load_hex();
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
