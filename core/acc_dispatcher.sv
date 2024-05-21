// Copyright 2020 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Authors: Matheus Cavalcante, ETH Zurich
//          Nils Wistoff, ETH Zurich
// Date: 20.11.2020
// Description: Functional unit that dispatches CVA6 instructions to accelerators.

module acc_dispatcher import ariane_pkg::*; import riscv::*; #(
    parameter type x_req_t = core_v_xif_pkg::x_req_t,
    parameter type x_resp_t = core_v_xif_pkg::x_resp_t,
    parameter type x_issue_req_t = core_v_xif_pkg::x_issue_req_t,
    parameter type x_issue_resp_t = core_v_xif_pkg::x_issue_resp_t,
    parameter type x_commit_t = core_v_xif_pkg::x_commit_t
  ) (
    input  logic                                  clk_i,
    input  logic                                  rst_ni,
    // Interface with the CSR regfile
    input  logic                                  acc_cons_en_i,        // Accelerator memory consistent mode
    // Interface with the CSRs
    input  logic                            [2:0] fcsr_frm_i,
    output logic                                  dirty_v_state_o,
    // Interface with the issue stage
    input  scoreboard_entry_t                     issue_instr_i,
    input  logic                                  issue_instr_hs_i,
    output logic                                  issue_stall_o,
    input  fu_data_t                              fu_data_i,
    input  scoreboard_entry_t [NR_COMMIT_PORTS-1:0] commit_instr_i,
    output logic              [TRANS_ID_BITS-1:0] acc_trans_id_o,
    output xlen_t                                 acc_result_o,
    output logic                                  acc_valid_o,
    output exception_t                            acc_exception_o,
    // Interface with the execute stage
    output logic                                  acc_valid_ex_o,       // FU executed
    // Interface with the commit stage
    input  logic            [NR_COMMIT_PORTS-1:0] commit_ack_i,
    input  logic                                  commit_st_barrier_i,  // A store barrier was commited
    // Interface with the load/store unit
    input  logic                                  acc_no_st_pending_i,
    // Interface with the controller
    output logic                                  ctrl_halt_o,
    input  logic                                  flush_unissued_instr_i,
    input  logic                                  flush_ex_i,
    // Accelerator interface
    input  logic                  issue_if_valid_i,
    input  x_issue_req_t          issue_if_i,
    output x_req_t                core_v_xif_req_o,
    input  x_resp_t               core_v_xif_resp_i
  );

  `include "common_cells/registers.svh"

  import cf_math_pkg::idx_width;

  /***********************
   *  Common signals     *
   ***********************/

  logic acc_ready;
  logic acc_valid_d, acc_valid_q;

  /**************************
   *  Accelerator issue     *
   **************************/

  // Issue accelerator instructions
  `FF(acc_valid_q, acc_valid_d, '0)

  assign acc_valid_ex_o = acc_valid_q;
  assign acc_valid_d    = ~issue_instr_i.ex.valid &
                          issue_instr_hs_i &
                          (issue_instr_i.fu == ACCEL) &
                          ~flush_unissued_instr_i;

  // Accelerator load/store pending signals
  logic acc_no_ld_pending;
  logic acc_no_st_pending;

  // Stall issue stage in three cases:
  always_comb begin : stall_issue
    unique case (issue_instr_i.fu)
      ACCEL:
        // 1. We're issuing an accelerator instruction but the dispatcher isn't ready yet
        issue_stall_o = ~acc_ready;
      LOAD:
        // 2. We're issuing a scalar load but there is an inflight accelerator store.
        issue_stall_o = acc_cons_en_i & ~acc_no_st_pending;
      STORE:
        // 3. We're issuing a scalar store but there is an inflight accelerator load or store.
        issue_stall_o = acc_cons_en_i & (~acc_no_st_pending | ~acc_no_ld_pending);
      default:
        issue_stall_o = 1'b0;
    endcase
  end

  /***********************
   *  Instruction queue  *
   ***********************/

  localparam InstructionQueueDepth = 3;

  fu_data_t                                        acc_data;
  fu_data_t                                        acc_insn_queue_o;
  logic                                            acc_insn_queue_pop;
  logic                                            acc_insn_queue_empty;
  logic     [idx_width(InstructionQueueDepth)-1:0] acc_insn_queue_usage;
  logic                                            acc_commit;
  logic     [TRANS_ID_BITS-1:0]                    acc_commit_trans_id;

  assign acc_data = acc_valid_ex_o ? fu_data_i : '0;

  fifo_v3 #(
    .DEPTH       (InstructionQueueDepth),
    .FALL_THROUGH(1'b1                 ),
    .dtype       (fu_data_t            )
  ) i_acc_insn_queue (
    .clk_i     (clk_i               ),
    .rst_ni    (rst_ni              ),
    .flush_i   (flush_ex_i          ),
    .testmode_i(1'b0                ),
    .data_i    (fu_data_i           ),
    .push_i    (acc_valid_q         ),
    .full_o    (/* Unused */        ),
    .data_o    (acc_insn_queue_o    ),
    .pop_i     (acc_insn_queue_pop  ),
    .empty_o   (acc_insn_queue_empty),
    .usage_o   (acc_insn_queue_usage)
  );

  // We are ready if the instruction queue is able to accept at least one more entry.
  assign acc_ready = acc_insn_queue_usage < (InstructionQueueDepth-1);

  /**********************************
   *  Non-speculative instructions  *
   **********************************/

  // Keep track of the instructions that were received by the dispatcher.
  logic [NR_SB_ENTRIES-1:0] insn_pending_d, insn_pending_q;
  `FF(insn_pending_q, insn_pending_d, '0)

  // Only non-speculative instructions can be issued to the accelerators.
  // The following block keeps track of which transaction IDs reached the
  // top of the scoreboard, and are therefore no longer speculative.
  logic [NR_SB_ENTRIES-1:0] insn_ready_d, insn_ready_q;
  `FF(insn_ready_q, insn_ready_d, '0)

  always_comb begin: p_non_speculative_ff
    // Maintain state
    insn_pending_d = insn_pending_q;
    insn_ready_d   = insn_ready_q;

    // We received a new instruction
    if (acc_valid_q)
      insn_pending_d[acc_data.trans_id] = 1'b1;
    // Flush all received instructions
    if (flush_ex_i)
      insn_pending_d = '0;

    // An accelerator instruction is no longer speculative.
    if (acc_commit && insn_pending_q[acc_commit_trans_id]) begin
      insn_ready_d[acc_commit_trans_id]   = 1'b1;
      insn_pending_d[acc_commit_trans_id] = 1'b0;
    end

    // An accelerator instruction was issued.
    if (core_v_xif_req_o.register_valid)
      insn_ready_d[core_v_xif_req_o.register.id] = 1'b0;
  end: p_non_speculative_ff

  /*************************
   *  Accelerator request  *
   *************************/

  acc_pkg::accelerator_req_t acc_req;
  logic                      acc_req_valid;
  logic                      acc_req_ready;

  acc_pkg::accelerator_req_t acc_req_int;
  fall_through_register #(
    .T(acc_pkg::accelerator_req_t)
  ) i_accelerator_req_register (
    .clk_i     (clk_i          ),
    .rst_ni    (rst_ni         ),
    .clr_i     (1'b0           ),
    .testmode_i(1'b0           ),
    .data_i    (acc_req        ),
    .valid_i   (acc_req_valid  ),
    .ready_o   (acc_req_ready  ),
    .data_o    (acc_req_int    ),
    .valid_o   (core_v_xif_req_o.register_valid),
    .ready_i   (core_v_xif_resp_i.register_ready)
  );

  assign core_v_xif_req_o.acc_req.instr          = acc_req_int.insn;
  assign core_v_xif_req_o.register.rs[0]           = acc_req_int.rs1;
  assign core_v_xif_req_o.register.rs[1]           = acc_req_int.rs2;
  assign core_v_xif_req_o.acc_req.frm              = acc_req_int.frm;
  assign core_v_xif_req_o.register.id              = acc_req_int.trans_id;
  assign core_v_xif_req_o.acc_req.store_pending    = !acc_no_st_pending_i && acc_cons_en_i;
  assign core_v_xif_req_o.acc_req.acc_cons_en      = acc_cons_en_i;
  // Will be overwritten by dcache
  assign core_v_xif_req_o.acc_req.inval_ready      = '0;

  always_comb begin: accelerator_req_dispatcher
    // Do not fetch from the instruction queue
    acc_insn_queue_pop = 1'b0;

    // Default values
    acc_req       = '0;
    acc_req_valid = 1'b0;

    // Unpack fu_data_t into accelerator_req_t
    if (!acc_insn_queue_empty) begin
      acc_req = '{
        // Instruction is forwarded from the decoder as an immediate
        // -
        // frm rounding information is up to date during a valid request to the accelerator
        // The scoreboard synchronizes it with previous fcsr writes, and future fcsr writes
        // do not take place until the accelerator answers (Ariane commits in-order)
        insn    : acc_insn_queue_o.imm[31:0],
        rs1     : acc_insn_queue_o.operand_a,
        rs2     : acc_insn_queue_o.operand_b,
        frm     : fpnew_pkg::roundmode_e'(fcsr_frm_i),
        trans_id: acc_insn_queue_o.trans_id,
        default : '0
      };
      // Wait until the instruction is no longer speculative.
      acc_req_valid      = insn_ready_q[acc_insn_queue_o.trans_id] ||
                           (acc_commit && insn_pending_q[acc_commit_trans_id]);
      acc_insn_queue_pop = acc_req_valid && acc_req_ready;
    end
  end

  /**************************
   *  Accelerator response  *
   **************************/

  logic acc_ld_disp;
  logic acc_st_disp;

  // Unpack the accelerator response
  assign acc_trans_id_o  = core_v_xif_resp_i.result.id;
  assign acc_result_o    = core_v_xif_resp_i.result.data;
  assign acc_valid_o     = core_v_xif_resp_i.result_valid;
  assign acc_exception_o = '{
      cause: riscv::ILLEGAL_INSTR,
      tval : '0,
      valid: core_v_xif_resp_i.acc_resp.error
    };
  // Always ready to receive responses
  assign core_v_xif_req_o.result_ready = 1'b1;

  // Signal dispatched load/store to issue stage
  assign acc_ld_disp = acc_req_valid && (acc_insn_queue_o.operation == ACCEL_OP_LOAD);
  assign acc_st_disp = acc_req_valid && (acc_insn_queue_o.operation == ACCEL_OP_STORE);

  /**************************
   *  Accelerator commit    *
   **************************/

  // Instruction can be issued to the (in-order) back-end if
  // it reached the top of the scoreboard and it hasn't been
  // issued yet
  always_comb begin: accelerator_commit
    acc_commit = 1'b0;
    if (!commit_instr_i[0].valid && commit_instr_i[0].fu == ACCEL)
        acc_commit = 1'b1;
    if (commit_instr_i[0].valid &&
        !commit_instr_i[1].valid && commit_instr_i[1].fu == ACCEL)
        acc_commit = 1'b1;
  end

  // Dirty the V state if we are committing anything related to the vector accelerator
  always_comb begin : dirty_v_state
    dirty_v_state_o = 1'b0;
    for (int i = 0; i < NR_COMMIT_PORTS; i++) begin
      dirty_v_state_o |= commit_ack_i[i] & (commit_instr_i[i].fu == ACCEL);
    end
  end

  assign acc_commit_trans_id = !commit_instr_i[0].valid ? commit_instr_i[0].trans_id
                                                        : commit_instr_i[1].trans_id;

  /**************************
   *  Accelerator barriers  *
   **************************/

  // On a store barrier (i.e. any barrier that requires preceeding stores to complete
  // before continuing execution), halt execution while there are pending stores in
  // the accelerator pipeline.
  logic wait_acc_store_d, wait_acc_store_q;
  `FF(wait_acc_store_q, wait_acc_store_d, '0)

  // Set on store barrier. Clear when no store is pending.
  assign wait_acc_store_d = (wait_acc_store_q | commit_st_barrier_i) & core_v_xif_resp_i.acc_resp.store_pending;
  assign ctrl_halt_o      = wait_acc_store_q;

  /**************************
   *  Load/Store tracking   *
   **************************/

  // Loads
  logic       acc_spec_loads_overflow;
  logic [2:0] acc_spec_loads_pending;
  logic       acc_disp_loads_overflow;
  logic [2:0] acc_disp_loads_pending;

  assign acc_no_ld_pending = (acc_spec_loads_pending == 3'b0) && (acc_disp_loads_pending == 3'b0);

  // Count speculative loads. These can still be flushed.
  counter #(
      .WIDTH           (3),
      .STICKY_OVERFLOW (0)
  ) i_acc_spec_loads (
      .clk_i           (clk_i                   ),
      .rst_ni          (rst_ni                  ),
      .clear_i         (flush_ex_i              ),
      .en_i            ((acc_valid_d && issue_instr_i.op == ACCEL_OP_LOAD) ^ acc_ld_disp),
      .load_i          (1'b0                    ),
      .down_i          (acc_ld_disp             ),
      .d_i             ('0                      ),
      .q_o             (acc_spec_loads_pending  ),
      .overflow_o      (acc_spec_loads_overflow )
  );

  // Count dispatched loads. These cannot be flushed anymore.
  counter #(
      .WIDTH           (3),
      .STICKY_OVERFLOW (0)
  ) i_acc_disp_loads (
      .clk_i           (clk_i                   ),
      .rst_ni          (rst_ni                  ),
      .clear_i         (1'b0                    ),
      .en_i            (acc_ld_disp ^ core_v_xif_resp_i.acc_resp.load_complete),
      .load_i          (1'b0                    ),
      .down_i          (core_v_xif_resp_i.acc_resp.load_complete),
      .d_i             ('0                      ),
      .q_o             (acc_disp_loads_pending  ),
      .overflow_o      (acc_disp_loads_overflow )
  );

  acc_dispatcher_no_load_overflow: assert property (
      @(posedge clk_i) disable iff (~rst_ni) (acc_spec_loads_overflow == 1'b0) && (acc_disp_loads_overflow == 1'b0) )
  else $error("[acc_dispatcher] Too many pending loads.");

  // Stores
  logic       acc_spec_stores_overflow;
  logic [2:0] acc_spec_stores_pending;
  logic       acc_disp_stores_overflow;
  logic [2:0] acc_disp_stores_pending;

  assign acc_no_st_pending = (acc_spec_stores_pending == 3'b0) && (acc_disp_stores_pending == 3'b0);

  // Count speculative stores. These can still be flushed.
  counter #(
      .WIDTH           (3),
      .STICKY_OVERFLOW (0)
  ) i_acc_spec_stores (
      .clk_i           (clk_i                   ),
      .rst_ni          (rst_ni                  ),
      .clear_i         (flush_ex_i              ),
      .en_i            ((acc_valid_d && issue_instr_i.op == ACCEL_OP_STORE) ^ acc_st_disp),
      .load_i          (1'b0                    ),
      .down_i          (acc_st_disp             ),
      .d_i             ('0                      ),
      .q_o             (acc_spec_stores_pending ),
      .overflow_o      (acc_spec_stores_overflow)
  );

  // Count dispatched stores. These cannot be flushed anymore.
  counter #(
      .WIDTH           (3),
      .STICKY_OVERFLOW (0)
  ) i_acc_disp_stores (
      .clk_i           (clk_i                    ),
      .rst_ni          (rst_ni                   ),
      .clear_i         (1'b0                     ),
      .en_i            (acc_st_disp ^ core_v_xif_resp_i.acc_resp.store_complete),
      .load_i          (1'b0                     ),
      .down_i          (core_v_xif_resp_i.acc_resp.store_complete),
      .d_i             ('0                       ),
      .q_o             (acc_disp_stores_pending  ),
      .overflow_o      (acc_disp_stores_overflow )
  );

  /**********************
   *  ISSUE INTERFACE   *
   **********************/

   assign core_v_xif_req_o.issue_valid  = issue_if_valid_i;
   assign core_v_xif_req_o.issue_req    = issue_if_i;

  /***********************
   *  COMMIT INTERFACE   *
   ***********************/

   logic commit_valid;
   x_commit_t commit_if;

   // logic new_top_instruction_d, new_top_instruction_q;

   // always_comb begin
   //  // Default values
   //  commit_valid = '0;
   //  commit_if.hartid = '0;
   //  commit_if.id = '0;
   //  commit_if.commit_kill = '0;


   //  if (flush_unissued_instr_i && new_top_instruction_q) begin
   //    commit_valid = 1'b1;
   //    commit_if.commit_kill = 1'b1;
   //    commit_if.id = id_trans_id_i;
   //  end


   //  if (flush_ex_i) begin
   //    commit_valid = 1'b1;
   //    commit_if.commit_kill = 1'b1;
   //    commit_if.id = issue_trans_id_i;
   //  end
   // end

   // // We need to keep track if we offloaded an instruction in the last cycle
   // assign new_top_instruction_d = (core_v_xif_req_o.issue_valid && core_v_xif_resp_i.issue_ready) ? 1'b1 : 1'b0;

   // always_ff @(posedge clk_i or negedge rst_ni) begin
   //   if(~rst_ni) begin
   //     new_top_instruction_q <= '0;
   //   end else begin
   //     new_top_instruction_q <= new_top_instruction_d;
   //   end
   // end

   logic [TRANS_ID_BITS-1:0]  commit_id;

   logic  new_instruction;
   logic  load_next_instruction;
   logic  push_to_buffer;
   logic  buffer_full;

   logic [TRANS_ID_BITS-1:0]  trans_id_d, trans_id_q;
   logic                      pre_buffer_full_d, pre_buffer_full_q;
   logic                      last_cycle_d, last_cycle_q;

   assign new_instruction       = core_v_xif_req_o.issue_valid && core_v_xif_resp_i.issue_ready && core_v_xif_resp_i.issue_resp.accept;
   assign load_next_instruction = core_v_xif_req_o.register_valid && core_v_xif_resp_i.register_ready;

   always_comb begin
     // Default values
     push_to_buffer     = 1'b0;
     last_cycle_d       = 1'b0;
     trans_id_d         = trans_id_q;
     pre_buffer_full_d  = pre_buffer_full_q;

     // If we have space in the fifo and an isntrucion in the pre buffer then we push it to the fifo
     if (!buffer_full && pre_buffer_full_q) begin
       push_to_buffer     = 1'b1;
       pre_buffer_full_d  = 1'b0;
     end

     // If we have space in the fifo then we can load a new instruction
     if (!buffer_full && new_instruction) begin
       trans_id_d         = core_v_xif_req_o.issue_req.id;
       pre_buffer_full_d  = 1'b1;
       last_cycle_d       = 1'b1;
     end

     // If we are flushing the unissued instructions then we also need to flush the pre buffer
     if (flush_unissued_instr_i && last_cycle_q) begin
       push_to_buffer     = 1'b0;
       pre_buffer_full_d  = 1'b0;
     end
   end

   // Buffer in fornt of the fifo
   always_ff @(posedge clk_i or negedge rst_ni) begin
     if(~rst_ni) begin
       trans_id_q         <= '0;
       pre_buffer_full_q  <= '0;
       last_cycle_q       <= '0;
     end else begin
       trans_id_q         <= trans_id_d;
       pre_buffer_full_q  <= pre_buffer_full_d;
       last_cycle_q       <= last_cycle_d;
     end
   end

   // fifo to hold the issued instruction ids
   fifo_v3 #(
      .DEPTH(4),
      .dtype(logic [TRANS_ID_BITS-1:0])
    ) i_trans_id_buffer (
      .clk_i(clk_i),
      .rst_ni(rst_ni),
      .flush_i(),
      .testmode_i('0),
      .full_o(buffer_full),
      .empty_o(),
      .usage_o(),
      .data_i(trans_id_q),
      .push_i(push_to_buffer),
      .data_o(commit_id),
      .pop_i(load_next_instruction)
    );

    // Construct commit information
    always_comb begin
      // Default values
      commit_valid          = '0;
      commit_if.hartid      = '0;
      commit_if.id          = '0;
      commit_if.commit_kill = '0;

      // Check for flushing of the top instruction
      if (flush_unissued_instr_i && last_cycle_q) begin
        commit_valid          = 1'b1;
        commit_if.id          = trans_id_q;
        commit_if.commit_kill = 1'b1;
      end

      // Check for flushing of the entire buffer
      if (flush_ex_i) begin
        commit_valid          = 1'b1;
        commit_if.id          = commit_id;
        commit_if.commit_kill = 1'b1;
      end
    end

   // Assign output
   assign core_v_xif_req_o.commit_valid = commit_valid;
   assign core_v_xif_req_o.commit = commit_if;

  acc_dispatcher_no_store_overflow: assert property (
      @(posedge clk_i) disable iff (~rst_ni) (acc_spec_stores_overflow == 1'b0) && (acc_disp_stores_overflow == 1'b0) )
  else $error("[acc_dispatcher] Too many pending stores.");

endmodule : acc_dispatcher
