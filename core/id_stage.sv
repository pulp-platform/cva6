// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba, ETH Zurich
// Date: 15.04.2017
// Description: Instruction decode, contains the logic for decode,
//              issue and read operands.

module id_stage #(
    parameter ariane_pkg::cva6_cfg_t cva6_cfg = ariane_pkg::cva6_cfg_empty,
    parameter type x_issue_req_t = core_v_xif_pkg::x_issue_req_t,
    parameter type x_issue_resp_t = core_v_xif_pkg::x_issue_resp_t
) (
    input  logic                          clk_i,
    input  logic                          rst_ni,

    input  logic                          flush_i,
    input  logic                          debug_req_i,
    // from IF
    input  ariane_pkg::fetch_entry_t      fetch_entry_i,
    input  logic                          fetch_entry_valid_i,
    output logic                          fetch_entry_ready_o, // acknowledge the instruction (fetch entry)
    // to ID
    output ariane_pkg::scoreboard_entry_t issue_entry_o,       // a decoded instruction
    output logic                          issue_entry_valid_o, // issue entry is valid
    output logic                          is_ctrl_flow_o,      // the instruction we issue is a ctrl flow instructions
    input  logic                          issue_instr_ack_i,   // issue stage acknowledged sampling of instructions
    // from CSR file
    input  riscv::priv_lvl_t              priv_lvl_i,          // current privilege level
    input  riscv::xs_t                    fs_i,                // floating point extension status
    input  logic [2:0]                    frm_i,               // floating-point dynamic rounding mode
    input  riscv::xs_t                    vs_i,                // vector extension status
    input  logic [1:0]                    irq_i,
    input  ariane_pkg::irq_ctrl_t         irq_ctrl_i,
    input  logic                          debug_mode_i,        // we are in debug mode
    input  logic                          tvm_i,
    input  logic                          tw_i,
    input  logic                          tsr_i,
    // XIF issue interface
    output logic               core_v_xif_issue_valid_o, // XIF issue handshake valid
    output x_issue_req_t       core_v_xif_issue_req_o,  // XIF issue interface request to coprocessor
    input  logic               core_v_xif_issue_ready_i, // XIF issue handshake ready
    input  x_issue_resp_t      core_v_xif_issue_resp_i  // XIF issue interface response from coprocessor
);
    // ID/ISSUE register stage
    typedef struct packed {
        logic                          valid;
        ariane_pkg::scoreboard_entry_t sbe;
        logic                          is_ctrl_flow;
    } issue_struct_t;
    issue_struct_t issue_n, issue_q;

    logic                            is_control_flow_instr;
    ariane_pkg::scoreboard_entry_t   decoded_instruction;

    logic                is_illegal;
    logic                [31:0] instruction;
    logic                is_compressed;

    if (ariane_pkg::RVC) begin
      // ---------------------------------------------------------
      // 1. Check if they are compressed and expand in case they are
      // ---------------------------------------------------------
      compressed_decoder #(
          .cva6_cfg   ( cva6_cfg   )
      ) compressed_decoder_i (
          .instr_i                 ( fetch_entry_i.instruction   ),
          .instr_o                 ( instruction                 ),
          .illegal_instr_o         ( is_illegal                  ),
          .is_compressed_o         ( is_compressed               )
      );
    end else begin
      assign instruction = fetch_entry_i.instruction;
      assign is_illegal = '0;
      assign is_compressed = '0;
    end
    // ---------------------------------------------------------
    // 2. Decode and emit instruction to issue stage
    // ---------------------------------------------------------
    decoder #(
        .cva6_cfg   ( cva6_cfg   ),
        .x_issue_req_t (x_issue_req_t),
        .x_issue_resp_t (x_issue_resp_t)
    ) decoder_i (
        .debug_req_i,
        .irq_ctrl_i,
        .irq_i,
        .pc_i                    ( fetch_entry_i.address           ),
        .is_compressed_i         ( is_compressed                   ),
        .is_illegal_i            ( is_illegal                      ),
        .instruction_i           ( instruction                     ),
        .compressed_instr_i      ( fetch_entry_i.instruction[15:0] ),
        .branch_predict_i        ( fetch_entry_i.branch_predict    ),
        .ex_i                    ( fetch_entry_i.ex                ),
        .priv_lvl_i              ( priv_lvl_i                      ),
        .debug_mode_i            ( debug_mode_i                    ),
        .fs_i,
        .frm_i,
        .vs_i,
        .tvm_i,
        .tw_i,
        .tsr_i,
        .instruction_o           ( decoded_instruction          ),
        .is_control_flow_instr_o ( is_control_flow_instr        ),
        .core_v_xif_issue_valid_o ( core_v_xif_issue_valid_o    ),
        .core_v_xif_issue_req_o  ( core_v_xif_issue_req_o       ),
        .core_v_xif_issue_ready_i ( core_v_xif_issue_ready_i    ),
        .core_v_xif_issue_resp_i ( core_v_xif_issue_resp_i      )
    );


    // ---------------------------------------------
    // Registser to hold the ouput from the decoder
    // ---------------------------------------------
    logic       decoded_instruction_valid;      // Value in decoded_instruction_d is valid
    logic       holding_decoded_instruction_d, holding_decoded_instruction_q;    // Register is holding a decoded instruction

    ariane_pkg::scoreboard_entry_t      decoded_instruction_d, decoded_instruction_q;

    assign decoded_instruction_valid = core_v_xif_issue_ready_i || holding_decoded_instruction_d;

    always_comb begin

        // default values
        holding_decoded_instruction_d = holding_decoded_instruction_q;
        decoded_instruction_d = decoded_instruction_q;

        // If the decoder has an instruction ready we want to store it in the register
        if (core_v_xif_issue_ready_i && core_v_xif_issue_valid_o) begin
            holding_decoded_instruction_d   = 1'b1;
            decoded_instruction_d           = decoded_instruction;
        end

        // If the instruction has been acknowledged by the Issue stage or the pipeline
        // is being flushed we need to update the holding instruction flag
        if (issue_instr_ack_i || flush_i)
            holding_decoded_instruction_d = 1'b0;
    end
    // ---------------------------------
    // Register (Decoder <-> Pipeleine)
    // ---------------------------------
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if(~rst_ni) begin
            decoded_instruction_q           <= '0;
            holding_decoded_instruction_q   <= '0;
        end else begin
            decoded_instruction_q           <= decoded_instruction_d;
            holding_decoded_instruction_q   <= holding_decoded_instruction_d;
        end
    end



    // ------------------
    // Pipeline Register
    // ------------------
    assign issue_entry_o = issue_q.sbe;
    assign issue_entry_valid_o = issue_q.valid;
    assign is_ctrl_flow_o = issue_q.is_ctrl_flow;

    always_comb begin
        issue_n     = issue_q;
        fetch_entry_ready_o = 1'b0;

        // Clear the valid flag if issue has acknowledged the instruction
        if (issue_instr_ack_i)
            issue_n.valid = 1'b0;

        // if we have a space in the register and the fetch is valid, go get it
        // or the issue stage is currently acknowledging an instruction, which means that we will have space
        // for a new instruction
        if ((!issue_q.valid || issue_instr_ack_i) && fetch_entry_valid_i && decoded_instruction_valid) begin
            fetch_entry_ready_o = 1'b1;
            issue_n = '{1'b1, decoded_instruction_d, is_control_flow_instr};
        end

        // invalidate the pipeline register on a flush
        if (flush_i)
            issue_n.valid = 1'b0;
    end
    // -------------------------
    // Registers (ID <-> Issue)
    // -------------------------
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if(~rst_ni) begin
            issue_q <= '0;
        end else begin
            issue_q <= issue_n;
        end
    end
endmodule
