// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// Platform control registers, described in cva6tb_regs.rdl

module cva6tb_regs #(
  parameter type apb_req_t = logic,
  parameter type apb_rsp_t = logic
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic        boot_mode_i,
  input  apb_req_t    apb_req_i,
  output apb_rsp_t    apb_rsp_o,
  // End of computation and the program's return code
  output logic        eoc_o,
  output logic [30:0] retcode_o,
  // External interrupt lines software raises and lowers: bit 0 M-mode, bit 1 S-mode
  output logic  [1:0] irq_o
);

  import cva6tb_regs_reg_pkg::*;

  cva6tb_regs__in_t  hw2reg;
  cva6tb_regs__out_t reg2hw;

  cva6tb_regs_reg i_cva6tb_regs_reg (
    .clk           ( clk_i                                                ),
    .arst_n        ( rst_ni                                               ),
    .s_apb_psel    ( apb_req_i.psel                                       ),
    .s_apb_penable ( apb_req_i.penable                                    ),
    .s_apb_pwrite  ( apb_req_i.pwrite                                     ),
    .s_apb_pprot   ( apb_req_i.pprot                                      ),
    .s_apb_paddr   ( apb_req_i.paddr[CVA6TB_REGS_REG_MIN_ADDR_WIDTH-1:0]  ),
    .s_apb_pwdata  ( apb_req_i.pwdata                                     ),
    .s_apb_pstrb   ( apb_req_i.pstrb                                      ),
    .s_apb_pready  ( apb_rsp_o.pready                                     ),
    .s_apb_prdata  ( apb_rsp_o.prdata                                     ),
    .s_apb_pslverr ( apb_rsp_o.pslverr                                    ),
    .hwif_in       ( hw2reg                                               ),
    .hwif_out      ( reg2hw                                               )
  );

  assign hw2reg.bootmode.spm.next = boot_mode_i;

  assign eoc_o     = reg2hw.eoc.eoc.value;
  assign retcode_o = reg2hw.eoc.retcode.value;
  assign irq_o     = {reg2hw.irq.seip.value, reg2hw.irq.meip.value};

endmodule
