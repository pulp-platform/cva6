//Copyright (C) 2018 to present,
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 2.0 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-2.0. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba, ETH Zurich
// Date: 08.02.2018
// Migrated: Luis Vitorio Cargnini, IEEE
// Date: 09.06.2018

// return address stack

`include "common_cells/registers.svh"

module ras #(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty,
    parameter int unsigned DEPTH = 2,
    parameter bit          EccEnable = 1'b0
) (
    // Subsystem Clock - SUBSYSTEM
    input logic clk_i,
    // Asynchronous reset active low - SUBSYSTEM
    input logic rst_ni,
    // Synchronous clear active high - SUBSYSTEM
    input logic clear_i,
    // Fetch flush request - CONTROLLER
    input logic flush_i,
    // Push address in RAS - FRONTEND
    input logic push_i,
    // Pop address from RAS - FRONTEND
    input logic pop_i,
    // Data to be pushed - FRONTEND
    input logic [riscv::VLEN-1:0] data_i,
    // Popped data - FRONTEND
    output ariane_pkg::ras_t data_o
);

  // ECC parameters
  localparam int unsigned RasBits = $bits(ariane_pkg::ras_t) * DEPTH;
  localparam int unsigned RasCorrBits = EccEnable ? $clog2(RasBits) + 2 : 0;
  localparam int unsigned RasSize = RasBits + RasCorrBits;

  logic [RasSize-1:0] stack_d, stack_q, ras_update_ecc_out;
  ariane_pkg::ras_t [DEPTH-1:0] ras_stack_dec, ras_update_ecc_in, ras_stack_update;

  if (EccEnable) begin
    hsiao_ecc_enc #(
      .DataWidth ( RasBits ),
      .ProtWidth ( RasCorrBits )
    ) i_ecc_write_enc (
      .in  ( ras_update_ecc_in  ),
      .out ( ras_update_ecc_out )
    );

    hsiao_ecc_dec #(
      .DataWidth ( RasBits ),
      .ProtWidth ( RasCorrBits )
    ) i_ecc_read_dec (
      .in  ( stack_q ),
      .out ( ras_stack_dec ),
      .syndrome_o (),
      .err_o      ()      
    );
  end else begin
    assign ras_stack_dec = stack_q;
    assign ras_update_ecc_out = ras_update_ecc_in;
  end

  assign data_o = ras_stack_dec[0];

  always_comb begin
    ras_update_ecc_in = ras_stack_dec;

    // push on the stack
    if (push_i) begin
      ras_update_ecc_in[0].ra = data_i;
      // mark the new return address as valid
      ras_update_ecc_in[0].valid = 1'b1;
      ras_update_ecc_in[DEPTH-1:1] = ras_stack_dec[DEPTH-2:0];
    end

    if (pop_i) begin
      ras_update_ecc_in[DEPTH-2:0] = ras_stack_dec[DEPTH-1:1];
      // we popped the value so invalidate the end of the stack
      ras_update_ecc_in[DEPTH-1].valid = 1'b0;
      ras_update_ecc_in[DEPTH-1].ra = 'b0;
    end
    // leave everything untouched and just push the latest value to the
    // top of the stack
    if (pop_i && push_i) begin
      ras_update_ecc_in = ras_stack_dec;
      ras_update_ecc_in[0].ra = data_i;
      ras_update_ecc_in[0].valid = 1'b1;
    end

    if (flush_i) begin
      ras_update_ecc_in = '0;
    end
  end

  assign stack_d = ras_update_ecc_out;

  `FFARNC(stack_q, stack_d, clear_i, '0, clk_i, rst_ni)

endmodule
