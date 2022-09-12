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

module id_stage import ariane_soc::*; (
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
    input  logic [NumInterruptSrc-1:0]    irq_i,
    input  logic [7:0]                    irq_level_i,         // interrupt level
    input  logic [7:0]                    mintthresh_i,        // interrupt threshold
    input  riscv::intstatus_rv_t          mintstatus_i,        // interrupt status
    output logic                          irq_ack_o,           // core side interrupt handshake (ready)
    input  ariane_pkg::irq_ctrl_t         irq_ctrl_i,
    input  logic                          clic_mode_i,
    input  logic                          debug_mode_i,        // we are in debug mode
    input  logic                          tvm_i,
    input  logic                          tw_i,
    input  logic                          tsr_i
);
    // ID/ISSUE register stage
    struct packed {
        logic                          valid;
        ariane_pkg::scoreboard_entry_t sbe;
        logic                          is_ctrl_flow;
    } issue_n, issue_q;

    logic                            is_control_flow_instr;
    ariane_pkg::scoreboard_entry_t   decoded_instruction;

    logic                is_illegal;
    logic                [31:0] instruction;
    logic                is_compressed;

    // irq_i is one hot encoded (due to how clic is the only source
    // requesting interrupts). Extract interrupt request and interrupt id.
    localparam int unsigned IrqIdWidth = $clog2(NumInterruptSrc);
    logic [NumInterruptSrc-1:0] irq_q;
    logic [7:0] irq_level, irq_level_ctrl;
    logic [$clog2(NumInterruptSrc)-1:0] irq_id_ctrl;
    // register all interrupt inputs
    always_ff @(posedge clk_i, negedge rst_ni) begin
      if (rst_ni == 1'b0) begin
        irq_q     <= '0;
        irq_level <= '0;
      end else begin
        irq_q     <= irq_i;
        irq_level <= irq_level_i;
      end
    end
    // decode one-hot to get request + id information
    // TODO: Directly pass req and id to the core interrupt interface
    for (genvar j = 0; j < IrqIdWidth; j++) begin : jl
      logic [NumInterruptSrc-1:0] tmp_mask;
      for (genvar i = 0; i < NumInterruptSrc; i++) begin : il
        logic [IrqIdWidth-1:0] tmp_i;
        assign tmp_i = i;
        assign tmp_mask[i] = tmp_i[j];
      end
      assign irq_id_ctrl[j] = |(tmp_mask & irq_q);
    end
    // pragma translate_off
`ifndef VERILATOR
      assert final ($onehot0(irq_q)) else
        $fatal(1, "[cva6] More than two bit set in irq_i (one-hot)");
`endif
    // pragma translate_on
    // Check if the interrupt level of the current interrupt exceeds the current
    // irq threshold and global interrupt are enabled (otherwise it wont' fire).
    // The effective interrupt threshold is the maximum of mintstatus.mil and
    // mintthresh, because interrupts with higher level have priority.
    logic [7:0] max_thresh;
    assign max_thresh = mintthresh_i > mintstatus_i.mil ? mintthresh_i : mintstatus_i.mil;
    assign irq_req_ctrl = (irq_level > max_thresh) && (|irq_q) && irq_ctrl_i.mie;
    assign irq_level_ctrl = irq_level;

    // ---------------------------------------------------------
    // 1. Check if they are compressed and expand in case they are
    // ---------------------------------------------------------
    compressed_decoder compressed_decoder_i (
        .instr_i                 ( fetch_entry_i.instruction   ),
        .instr_o                 ( instruction                 ),
        .illegal_instr_o         ( is_illegal                  ),
        .is_compressed_o         ( is_compressed               )
    );
    // ---------------------------------------------------------
    // 2. Decode and emit instruction to issue stage
    // ---------------------------------------------------------
    decoder decoder_i (
        .debug_req_i,
        .irq_ctrl_i,
        .irq_req_ctrl_i          ( irq_req_ctrl                    ),
        .irq_id_ctrl_i           ( irq_id_ctrl                     ),
        .irq_level_ctrl_i        ( irq_level_ctrl                  ),
        .irq_ack_o,
        .clic_mode_i,
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
        .tvm_i,
        .tw_i,
        .tsr_i,
        .instruction_o           ( decoded_instruction          ),
        .is_control_flow_instr_o ( is_control_flow_instr        )
    );

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
        if ((!issue_q.valid || issue_instr_ack_i) && fetch_entry_valid_i) begin
            fetch_entry_ready_o = 1'b1;
            issue_n = '{1'b1, decoded_instruction, is_control_flow_instr};
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