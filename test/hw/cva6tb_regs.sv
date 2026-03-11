// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>

package cva6tb_regs_pkg;
  localparam int unsigned NumRegs      = 16;
  localparam int unsigned RegAddrWidth = $clog2(NumRegs);

  localparam logic [RegAddrWidth-1:0] BootaddrRegOffset = 4'h0; // Boot address register
  localparam logic [RegAddrWidth-1:0] EOCRegOffset      = 4'h1; // End Of Computation register
endpackage

module cva6tb_regs #(
  parameter type reg_req_t = logic,
  parameter type reg_rsp_t = logic
) (
  input  logic            clk_i,
  input  logic            rst_ni,
  input  reg_req_t        reg_req_i,
  output reg_rsp_t        reg_rsp_o,
  output logic     [31:0] boot_addr_o
);

  import cva6tb_regs_pkg::*;

  logic        [31:0] control_regs [NumRegs];
  logic [NumRegs-1:0] reg_select;
  logic        [31:0] write_mask;

  assign boot_addr_o = control_regs[BootaddrRegOffset];

  // Register selection logic
  genvar i;
  generate
    for (i = 0; i < NumRegs; i++) begin : gen_reg_select
      assign reg_select[i] = (reg_req_i.addr[(RegAddrWidth+2)-1:2] == i);
    end
  endgenerate

  // Write bitmask
  genvar j;
  generate
    for (j = 0; j < 4; j++) begin : gen_write_mask
      assign write_mask[j*8 +: 8] = {8{reg_req_i.wstrb[j]}};
    end
  endgenerate

  // Read logic
  always_comb begin
    reg_rsp_o = '0;
    reg_rsp_o.ready = 1'b1;
    reg_rsp_o.error = 1'b0;
    reg_rsp_o.rdata = 32'd0;
    for (int i = 0; i < NumRegs; i++) begin
      if (reg_select[i]) begin
        reg_rsp_o.rdata = control_regs[i];
      end
    end
  end

  // Register read/write logic
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      for (int i = 0; i < NumRegs; i++) begin
        control_regs[i] <= 32'd0;
      end
      control_regs[BootaddrRegOffset] <= 32'h8000_0000;
    end else begin
      for (int i = 0; i < NumRegs; i++) begin
        if (reg_req_i.valid && reg_req_i.write && reg_select[i]) begin
          control_regs[i] <= (control_regs[i] & ~write_mask) | (reg_req_i.wdata & write_mask);
        end
      end
    end
  end

endmodule
