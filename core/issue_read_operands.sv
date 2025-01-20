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
// Date: 08.04.2017
// Description: Issues instruction from the scoreboard and fetches the operands
//              This also includes all the forwarding logic


module issue_read_operands
  import ariane_pkg::*;
#(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty,
    parameter type branchpredict_sbe_t = logic,
    parameter type fu_data_t = logic,
    parameter type scoreboard_entry_t = logic,
    parameter type forwarding_t = logic,
    parameter type writeback_t = logic,
    parameter type rs3_len_t = logic,
    parameter type x_issue_req_t = logic,
    parameter type x_issue_resp_t = logic,
    parameter type x_register_t = logic,
    parameter type x_commit_t = logic

) (
    // Subsystem Clock - SUBSYSTEM
    input logic clk_i,
    // Asynchronous reset active low - SUBSYSTEM
    input logic rst_ni,
    input logic rst_uarch_ni,
    // Flush - CONTROLLER
    input logic flush_i,
    // Stall inserted by Acc dispatcher - ACC_DISPATCHER
    input logic stall_i,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    input scoreboard_entry_t [CVA6Cfg.NrIssuePorts-1:0] issue_instr_i,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    input logic [CVA6Cfg.NrIssuePorts-1:0][31:0] orig_instr_i,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    input logic [CVA6Cfg.NrIssuePorts-1:0] issue_instr_valid_i,
    // Issue stage acknowledge - TO_BE_COMPLETED
    output logic [CVA6Cfg.NrIssuePorts-1:0] issue_ack_o,
    // Forwarding - SCOREBOARD
    input forwarding_t fwd_i,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    output fu_data_t [CVA6Cfg.NrIssuePorts-1:0] fu_data_o,
    // Fusion signals
    output fusion_t fuse_o,
    // Unregistered version of fu_data_o.operanda - TO_BE_COMPLETED
    output logic [CVA6Cfg.NrIssuePorts-1:0][CVA6Cfg.XLEN-1:0] rs1_forwarding_o,
    // Unregistered version of fu_data_o.operandb - TO_BE_COMPLETED
    output logic [CVA6Cfg.NrIssuePorts-1:0][CVA6Cfg.XLEN-1:0] rs2_forwarding_o,
    // Instruction pc - TO_BE_COMPLETED
    output logic [CVA6Cfg.VLEN-1:0] pc_o,
    // Is compressed instruction - TO_BE_COMPLETED
    output logic is_compressed_instr_o,
    // Fixed Latency Unit ready to accept new request - TO_BE_COMPLETED
    input logic flu_ready_i,
    // ALU output is valid - TO_BE_COMPLETED
    output logic [CVA6Cfg.NrIssuePorts-1:0] alu_valid_o,
    // Branch instruction is valid - TO_BE_COMPLETED
    output logic [CVA6Cfg.NrIssuePorts-1:0] branch_valid_o,
    // Transformed instruction - TO_BE_COMPLETED
    output logic [CVA6Cfg.NrIssuePorts-1:0][31:0] tinst_o,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    output branchpredict_sbe_t branch_predict_o,
    // Load Store Unit is ready - TO_BE_COMPLETED
    input logic lsu_ready_i,
    // Load Store Unit result is valid - TO_BE_COMPLETED
    output logic [CVA6Cfg.NrIssuePorts-1:0] lsu_valid_o,
    // Mult result is valid - TO_BE_COMPLETED
    output logic [CVA6Cfg.NrIssuePorts-1:0] mult_valid_o,
    // FPU is ready - TO_BE_COMPLETED
    input logic fpu_ready_i,
    // FPU result is valid - TO_BE_COMPLETED
    output logic [CVA6Cfg.NrIssuePorts-1:0] fpu_valid_o,
    // FPU fmt field from instruction - TO_BE_COMPLETED
    output logic [1:0] fpu_fmt_o,
    // FPU rm field from isntruction - TO_BE_COMPLETED
    output logic [2:0] fpu_rm_o,
    // ALU output is valid - TO_BE_COMPLETED
    output logic [CVA6Cfg.NrIssuePorts-1:0] alu2_valid_o,
    // CSR result is valid - TO_BE_COMPLETED
    output logic [CVA6Cfg.NrIssuePorts-1:0] csr_valid_o,
    // CVXIF result is valid - TO_BE_COMPLETED
    output logic [CVA6Cfg.NrIssuePorts-1:0] cvxif_valid_o,
    // CVXIF is ready - TO_BE_COMPLETED
    input logic cvxif_ready_i,
    // CVXIF offloaded instruction - TO_BE_COMPLETED
    output logic [31:0] cvxif_off_instr_o,
    // CVA6 Hart ID - SUBSYSTEM
    input logic [CVA6Cfg.XLEN-1:0] hart_id_i,
    // CVXIF Issue interface
    input logic x_issue_ready_i,
    input x_issue_resp_t x_issue_resp_i,
    output logic x_issue_valid_o,
    output x_issue_req_t x_issue_req_o,
    // CVXIF Register interface
    input logic x_register_ready_i,
    output logic x_register_valid_o,
    output x_register_t x_register_o,
    // CVXIF Commit interface
    output logic x_commit_valid_o,
    output x_commit_t x_commit_o,
    // Writeback Handling of CVXIF
    output logic x_transaction_accepted_o,
    output logic x_transaction_rejected_o,
    output logic x_issue_writeback_o,
    output logic [CVA6Cfg.TRANS_ID_BITS-1:0] x_id_o,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    input logic [CVA6Cfg.NrCommitPorts-1:0][4:0] waddr_i,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    input logic [CVA6Cfg.NrCommitPorts-1:0][CVA6Cfg.XLEN-1:0] wdata_i,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    input logic [CVA6Cfg.NrCommitPorts-1:0] we_gpr_i,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    input logic [CVA6Cfg.NrCommitPorts-1:0] we_fpr_i,

    // Stall signal, we do not want to fetch any more entries - TO_BE_COMPLETED
    output logic stall_issue_o,
    input logic fpu_early_valid_i
);

  localparam OPERANDS_PER_INSTR = CVA6Cfg.NrRgprPorts / CVA6Cfg.NrIssuePorts;

  typedef struct packed {
    logic none, load, store, alu, alu2, ctrl_flow, mult, csr, fpu, fpu_vec, cvxif, accel;
  } fus_busy_t;

  typedef struct packed {
    logic [CVA6Cfg.NrIssuePorts-1:0] rs1;
    logic [CVA6Cfg.NrIssuePorts-1:0] rs2;
    logic [CVA6Cfg.NrIssuePorts-1:0] rs3;
    logic rs1_rd;
    logic rs2_rd;
    logic rs3_rd;
  } mo_t;

  logic [CVA6Cfg.NrIssuePorts-1:0] stall_raw, stall_waw, stall_rs1, stall_rs2, stall_rs3;
  logic [CVA6Cfg.NrIssuePorts-1:0] fu_busy;  // functional unit is busy
  fus_busy_t [CVA6Cfg.NrIssuePorts-1:0] fus_busy;  // which functional units are considered busy
  logic [CVA6Cfg.NrIssuePorts-1:0] issue_ack;
  // operands coming from regfile
  logic [CVA6Cfg.NrIssuePorts-1:0][CVA6Cfg.XLEN-1:0] operand_a_regfile, operand_b_regfile;
  // third operand from fp regfile or gp regfile if NR_RGPR_PORTS == 3
  rs3_len_t [CVA6Cfg.NrIssuePorts-1:0] operand_c_regfile, operand_c_gpr;
  rs3_len_t operand_c_fpr;
  // output flipflop (ID <-> EX)
  fu_data_t [CVA6Cfg.NrIssuePorts-1:0] fu_data_n, fu_data_q;
  logic [CVA6Cfg.NrIssuePorts-1:0][CVA6Cfg.XLEN-1:0] imm_forward_rs3;

  logic [CVA6Cfg.NrIssuePorts-1:0]                   alu_valid_q;
  logic [CVA6Cfg.NrIssuePorts-1:0]                   mult_valid_q;
  logic [CVA6Cfg.NrIssuePorts-1:0]                   fpu_valid_q;
  logic [                     1:0]                   fpu_fmt_q;
  logic [                     2:0]                   fpu_rm_q;
  logic [CVA6Cfg.NrIssuePorts-1:0]                   alu2_valid_q;
  logic [CVA6Cfg.NrIssuePorts-1:0]                   lsu_valid_q;
  logic [CVA6Cfg.NrIssuePorts-1:0]                   csr_valid_q;
  logic [CVA6Cfg.NrIssuePorts-1:0]                   branch_valid_q;
  logic [CVA6Cfg.NrIssuePorts-1:0]                   cvxif_valid_q;
  logic [                    31:0]                   cvxif_off_instr_q;
  logic                                              cvxif_instruction_valid;

  //fwd logic
  logic [CVA6Cfg.NrIssuePorts-1:0]                   rs1_has_raw;
  logic [CVA6Cfg.NrIssuePorts-1:0]                   rs2_has_raw;
  logic [CVA6Cfg.NrIssuePorts-1:0]                   rs3_has_raw;

  logic [CVA6Cfg.NrIssuePorts-1:0][CVA6Cfg.XLEN-1:0] rs3;
  logic [CVA6Cfg.NrIssuePorts-1:0]                   rs3_fpr;

  logic [CVA6Cfg.NrIssuePorts-1:0]                   rs1_valid;
  logic [CVA6Cfg.NrIssuePorts-1:0]                   rs2_valid;
  logic [CVA6Cfg.NrIssuePorts-1:0]                   rs3_valid;

  logic [CVA6Cfg.NrIssuePorts-1:0][CVA6Cfg.XLEN-1:0] rs1_res;
  logic [CVA6Cfg.NrIssuePorts-1:0][CVA6Cfg.XLEN-1:0] rs2_res;
  logic [CVA6Cfg.NrIssuePorts-1:0][CVA6Cfg.XLEN-1:0] rs3_res;

  // clobber
  logic [2**ariane_pkg::REG_ADDR_SIZE-1:0][CVA6Cfg.TRANS_ID_BITS:0] rd_clobber_gpr, rd_clobber_fpr;

  // Csr issued check
  logic [CVA6Cfg.NR_SB_ENTRIES-1:0] csr_issued_raw;
  logic [CVA6Cfg.NR_SB_ENTRIES:0] csr_issued;
  
  for (genvar i=0; i<CVA6Cfg.NR_SB_ENTRIES; i++) begin
    assign csr_issued_raw[i] = fwd_i.still_issued[i] && fwd_i.sbe[i].fu == ariane_pkg::CSR;
  end

  assign csr_issued = {1'b0, csr_issued_raw};

  //forward logic
  logic [CVA6Cfg.NrIssuePorts-1:0][CVA6Cfg.NR_SB_ENTRIES+CVA6Cfg.NrWbPorts-1:0]
      rs1_fwd_req, rs2_fwd_req, rs3_fwd_req;
  logic [CVA6Cfg.NrIssuePorts-1:0] rs1_is_not_gpr0, rs2_is_not_gpr0, rs3_is_not_gpr0;
  logic [CVA6Cfg.NrIssuePorts-1:0][CVA6Cfg.NR_SB_ENTRIES+CVA6Cfg.NrWbPorts-1:0][CVA6Cfg.XLEN-1:0] rs_data;
  logic [CVA6Cfg.NrIssuePorts-1:0] rs1_available, rs2_available, rs3_available;


  logic [CVA6Cfg.NrIssuePorts-1:0][31:0] tinst_n, tinst_q;  // transformed instruction

  // forwarding signals
  logic [CVA6Cfg.NrIssuePorts-1:0] forward_rs1, forward_rs2, forward_rs3;

  // original instruction
  riscv::instruction_t [CVA6Cfg.NrIssuePorts-1:0] orig_instr;
  for (genvar i=0; i < CVA6Cfg.NrIssuePorts; i++) begin
    assign orig_instr[i] = riscv::instruction_t'(orig_instr_i[i]);
  end

  // Operand fusion signals
  fusion_t fuse, fuse_q;

  // CVXIF Signals
  logic cvxif_req_allowed;
  logic x_transaction_rejected;
  logic [OPERANDS_PER_INSTR-1:0] rs_valid;
  logic [OPERANDS_PER_INSTR-1:0][CVA6Cfg.XLEN-1:0] rs;

  cvxif_issue_register_commit_if_driver #(
      .CVA6Cfg       (CVA6Cfg),
      .x_issue_req_t (x_issue_req_t),
      .x_issue_resp_t(x_issue_resp_t),
      .x_register_t  (x_register_t),
      .x_commit_t    (x_commit_t)
  ) i_cvxif_issue_register_commit_if_driver (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .flush_i         (flush_i),
      .hart_id_i       (hart_id_i),
      .issue_ready_i   (x_issue_ready_i),
      .issue_resp_i    (x_issue_resp_i),
      .issue_valid_o   (x_issue_valid_o),
      .issue_req_o     (x_issue_req_o),
      .register_ready_i(x_register_ready_i),
      .register_valid_o(x_register_valid_o),
      .register_o      (x_register_o),
      .commit_valid_o  (x_commit_valid_o),
      .commit_o        (x_commit_o),
      .valid_i         (cvxif_instruction_valid),
      .x_off_instr_i   (orig_instr_i[0]),
      .x_trans_id_i    (issue_instr_i[0].trans_id),
      .register_i      (rs),
      .rs_valid_i      (rs_valid),
      .cvxif_busy_o    ()
  );
  if (OPERANDS_PER_INSTR == 3) begin
    assign rs_valid = {~stall_rs3[0], ~stall_rs2[0], ~stall_rs1[0]};
    assign rs = {fu_data_n[0].imm, fu_data_n[0].operand_b, fu_data_n[0].operand_a};
  end else begin
    assign rs_valid = {~stall_rs2[0], ~stall_rs1[0]};
    assign rs = {fu_data_n[0].operand_b, fu_data_n[0].operand_a};
  end

  // TODO check only for 1st instruction ??
  // Allow a cvxif transaction if we WaW condition are ok.
  assign cvxif_req_allowed = (issue_instr_i[0].fu == CVXIF) && !stall_waw[0];
  assign cvxif_instruction_valid = !issue_instr_i[0].ex.valid && issue_instr_valid_i[0] && cvxif_req_allowed;
  assign x_transaction_accepted_o = x_issue_valid_o && x_issue_ready_i && x_issue_resp_i.accept;
  assign x_transaction_rejected = x_issue_valid_o && x_issue_ready_i && ~x_issue_resp_i.accept;
  assign x_issue_writeback_o = x_issue_resp_i.writeback;
  assign x_id_o = x_issue_req_o.id;

  // ID <-> EX registers

  for (genvar i = 0; i < CVA6Cfg.NrIssuePorts; i++) begin
    assign rs1_forwarding_o[i] = fu_data_n[i].operand_a[CVA6Cfg.VLEN-1:0];  //forwarding or unregistered rs1 value
    assign rs2_forwarding_o[i] = fu_data_n[i].operand_b[CVA6Cfg.VLEN-1:0];  //forwarding or unregistered rs2 value
  end

  assign fuse_o = fuse_q;
  assign fu_data_o = fu_data_q;
  assign alu_valid_o = alu_valid_q;
  assign branch_valid_o = branch_valid_q;
  assign lsu_valid_o = lsu_valid_q;
  assign csr_valid_o = csr_valid_q;
  assign mult_valid_o = mult_valid_q;
  assign fpu_valid_o = fpu_valid_q;
  assign fpu_fmt_o = fpu_fmt_q;
  assign fpu_rm_o = fpu_rm_q;
  assign alu2_valid_o = alu2_valid_q;
  assign cvxif_valid_o = CVA6Cfg.CvxifEn ? cvxif_valid_q : '0;
  assign cvxif_off_instr_o = CVA6Cfg.CvxifEn ? cvxif_off_instr_q : '0;
  assign stall_issue_o = stall_raw[0];
  assign tinst_o = CVA6Cfg.RVH ? tinst_q : '0;
  // ---------------
  // Issue Stage
  // ---------------

  always_comb begin : structural_hazards
    fus_busy = '0;
    // CVXIF is always ready to try a new transaction on 1st issue port
    // If a transaction is already pending then we stall until the transaction is done.(issue_ack_o[0] = 0)
    // Since we can not have two CVXIF instruction on 1st issue port, CVXIF is always ready for the pending instruction.
    if (!flu_ready_i) begin
      fus_busy[0].alu = 1'b1;
      fus_busy[0].ctrl_flow = 1'b1;
      fus_busy[0].csr = 1'b1;
      fus_busy[0].mult = 1'b1;
    end

    // after a multiplication was issued we can only issue another multiplication
    // otherwise we will get contentions on the fixed latency bus
    if (mult_valid_q) begin
      fus_busy[0].alu = 1'b1;
      fus_busy[0].ctrl_flow = 1'b1;
      fus_busy[0].csr = 1'b1;
    end

    if (CVA6Cfg.FpPresent && !fpu_ready_i) begin
      fus_busy[0].fpu = 1'b1;
      fus_busy[0].fpu_vec = 1'b1;
      if (CVA6Cfg.SuperscalarEn) fus_busy[0].alu2 = 1'b1;
    end

    if (!lsu_ready_i) begin
      fus_busy[0].load  = 1'b1;
      fus_busy[0].store = 1'b1;
    end

    if (fpu_early_valid_i) begin
        fus_busy[0].alu2 = 1'b1;
    end

    if (CVA6Cfg.SuperscalarEn) begin
      fus_busy[1] = fus_busy[0];

      // Never issue CSR instruction on second issue port.
      fus_busy[1].csr = 1'b1;
      // Never issue CVXIF instruction on second issue port.
      fus_busy[1].cvxif = 1'b1;

      unique case (issue_instr_i[0].fu)
        NONE:  fus_busy[1].none = 1'b1;
        CTRL_FLOW: begin
          if (CVA6Cfg.SpeculativeSb) begin
            // Issue speculative instruction, will be removed on BMISS
            fus_busy[1].alu = 1'b1;
            fus_busy[1].ctrl_flow = 1'b1;
            fus_busy[1].csr = 1'b1;
            // Speculative non-idempotent loads are not supported yet
            fus_busy[1].load = 1'b1;
            // The store buffer cannot be partially flushed yet
            fus_busy[1].store = 1'b1;
          end else begin
            // There are no branch misses on a JAL
            if (issue_instr_i[0].op == ariane_pkg::ADD) begin
              fus_busy[1].alu = 1'b1;
              fus_busy[1].ctrl_flow = 1'b1;
              fus_busy[1].csr = 1'b1;
            end else begin
              // Control hazard
              fus_busy[1] = '1;
            end
          end
        end
        ALU: begin
          if (CVA6Cfg.SuperscalarEn && !fus_busy[0].alu2) begin
            if (issue_instr_i[1].fu inside {FPU, FPU_VEC} && !flu_ready_i) begin
              fus_busy[0].alu2 = 1'b1; // Here we block the ALU2 from being used from [0].
                                       // If not, it would use the same WB port of the FPU.
            end else if (issue_instr_i[1].fu == CTRL_FLOW) begin
              fus_busy[0].alu  = 1'b1;
            end else if (CVA6Cfg.FUSE && issue_instr_i[1].fu == ALU && issue_instr_valid_i[1] && !fus_busy[0].alu) begin
              fus_busy[0].alu2 = 1'b1;
            end else begin
              fus_busy[1].alu2 = 1'b1;
            end
          end else begin
            fus_busy[1].alu = 1'b1;
            fus_busy[1].ctrl_flow = 1'b1;
            fus_busy[1].csr = 1'b1;
          end
        end
        CSR: begin
          // Control hazard
          fus_busy[1] = '1;
        end
        MULT:  fus_busy[1].mult = 1'b1;
        FPU, FPU_VEC: begin
          fus_busy[1].fpu = 1'b1;
          fus_busy[1].fpu_vec = 1'b1;
          if (issue_instr_i[1].op inside {[FLD:FSB]}) begin
            fus_busy[1].load = 1'b1;
            fus_busy[1].store = 1'b1;
          end
        end
        LOAD, STORE: begin
          fus_busy[1].load  = 1'b1;
          fus_busy[1].store = 1'b1;
          if (issue_instr_i[0].op inside {[FLD:FSB]}) begin //TODO: Change to only Stores, loads do not use FP register from the GPR
            fus_busy[1].fpu = 1'b1;
            fus_busy[1].fpu_vec = 1'b1;
          end
        end
        CVXIF: ;
      endcase
    end
  end

  // select the right busy signal
  // this obviously depends on the functional unit we need
  for (genvar i = 0; i < CVA6Cfg.NrIssuePorts; i++) begin
    always_comb begin
      unique case (issue_instr_i[i].fu)
        NONE: fu_busy[i] = fus_busy[i].none;
        ALU: begin
          if (CVA6Cfg.SuperscalarEn && !fus_busy[i].alu2) begin
            fu_busy[i] = fus_busy[i].alu2;
          end else begin
            fu_busy[i] = fus_busy[i].alu;
          end
        end
        CTRL_FLOW: fu_busy[i] = fus_busy[i].ctrl_flow;
        CSR: fu_busy[i] = fus_busy[i].csr;
        MULT: fu_busy[i] = fus_busy[i].mult;
        FPU: fu_busy[i] = fus_busy[i].fpu;
        FPU_VEC: fu_busy[i] = fus_busy[i].fpu_vec;
        LOAD: fu_busy[i] = fus_busy[i].load;
        STORE: fu_busy[i] = fus_busy[i].store;
        CVXIF: fu_busy[i] = fus_busy[i].cvxif;
        default: fu_busy[i] = 1'b0;
      endcase
    end
  end

  // -------------------
  // RD clobber process
  // -------------------
  // rd_clobber output: output currently clobbered destination registers
  logic [2*CVA6Cfg.NR_SB_ENTRIES-1:0] still_issued_rotated_raw;
  logic [CVA6Cfg.NR_SB_ENTRIES-1:0] still_issued_rotated;
  logic [CVA6Cfg.TRANS_ID_BITS-1:0] unrotated_index;

  assign still_issued_rotated_raw = {fwd_i.still_issued, fwd_i.still_issued} >> fwd_i.commit_pointer;
  assign still_issued_rotated     = still_issued_rotated_raw[CVA6Cfg.NR_SB_ENTRIES-1:0];

  always_comb begin : clobber_assign
    rd_clobber_gpr = '{default: CVA6Cfg.NR_SB_ENTRIES};
    rd_clobber_fpr = '{default: CVA6Cfg.NR_SB_ENTRIES};

    // default (highest entry hast lowest prio in arbiter tree below)
    unrotated_index = '0;
    for (int unsigned i=0; i < CVA6Cfg.NR_SB_ENTRIES; i++) begin
        unrotated_index = i + fwd_i.commit_pointer;
        if (still_issued_rotated[i]) begin
            if (~(CVA6Cfg.FpPresent && ariane_pkg::is_rd_fpr(fwd_i.sbe[unrotated_index].op))) begin
                rd_clobber_gpr[fwd_i.sbe[unrotated_index].rd] = unrotated_index[CVA6Cfg.TRANS_ID_BITS-1:0];
            end else begin
                rd_clobber_fpr[fwd_i.sbe[unrotated_index].rd] = unrotated_index[CVA6Cfg.TRANS_ID_BITS-1:0];
            end
        end
    end

    // GPR[0] is always free
    rd_clobber_gpr[0] = CVA6Cfg.NR_SB_ENTRIES;
  end

  // ----------------------------------
  // Read Operands (a.k.a forwarding)
  // ----------------------------------
  // read operand interface: same logic as register file

  // WB ports have higher prio than entries
  for (genvar i = 0; i < CVA6Cfg.NrIssuePorts; i++) begin
    for (genvar k = 0; unsigned'(k) < CVA6Cfg.NrWbPorts; k++) begin : gen_rs_wb
      assign rs1_fwd_req[i][k] = ((fwd_i.wb[k].trans_id == rd_clobber_gpr[issue_instr_i[i].rs1] && (!ariane_pkg::is_rs1_fpr(issue_instr_i[i].op))) || (fwd_i.wb[k].trans_id == rd_clobber_fpr[issue_instr_i[i].rs1] && (CVA6Cfg.FpPresent && ariane_pkg::is_rs1_fpr(issue_instr_i[i].op)))) & fwd_i.wb[k].valid & (~fwd_i.wb[k].ex_valid);
      assign rs2_fwd_req[i][k] = ((fwd_i.wb[k].trans_id == rd_clobber_gpr[issue_instr_i[i].rs2] && (!ariane_pkg::is_rs2_fpr(issue_instr_i[i].op))) || (fwd_i.wb[k].trans_id == rd_clobber_fpr[issue_instr_i[i].rs2] && (CVA6Cfg.FpPresent && ariane_pkg::is_rs2_fpr(issue_instr_i[i].op)))) & fwd_i.wb[k].valid & (~fwd_i.wb[k].ex_valid);
      assign rs3_fwd_req[i][k] = ((fwd_i.wb[k].trans_id == rd_clobber_gpr[issue_instr_i[i].result[ariane_pkg::REG_ADDR_SIZE-1:0]] && (!ariane_pkg::is_imm_fpr(issue_instr_i[i].op))) || (fwd_i.wb[k].trans_id == rd_clobber_fpr[issue_instr_i[i].result[ariane_pkg::REG_ADDR_SIZE-1:0]] && (CVA6Cfg.FpPresent && ariane_pkg::is_imm_fpr(issue_instr_i[i].op)))) & fwd_i.wb[k].valid & (~fwd_i.wb[k].ex_valid);
      assign rs_data[i][k] = fwd_i.wb[k].data;
    end

    for (genvar k = 0; unsigned'(k) < CVA6Cfg.NR_SB_ENTRIES; k++) begin : gen_rs_entries
      assign rs1_fwd_req[i][k+CVA6Cfg.NrWbPorts] = ((rd_clobber_gpr[issue_instr_i[i].rs1] == k && (!ariane_pkg::is_rs1_fpr(issue_instr_i[i].op))) || (rd_clobber_fpr[issue_instr_i[i].rs1] == k && (CVA6Cfg.FpPresent && ariane_pkg::is_rs1_fpr(issue_instr_i[i].op)))) & fwd_i.still_issued[k] & fwd_i.sbe[k].valid ;
      assign rs2_fwd_req[i][k+CVA6Cfg.NrWbPorts] = ((rd_clobber_gpr[issue_instr_i[i].rs2] == k && (!ariane_pkg::is_rs2_fpr(issue_instr_i[i].op))) || (rd_clobber_fpr[issue_instr_i[i].rs2] == k && (CVA6Cfg.FpPresent && ariane_pkg::is_rs2_fpr(issue_instr_i[i].op)))) & fwd_i.still_issued[k] & fwd_i.sbe[k].valid ;
      assign rs3_fwd_req[i][k+CVA6Cfg.NrWbPorts] = ((rd_clobber_gpr[issue_instr_i[i].result[ariane_pkg::REG_ADDR_SIZE-1:0]] == k && (!ariane_pkg::is_imm_fpr(issue_instr_i[i].op))) || (rd_clobber_fpr[issue_instr_i[i].result[ariane_pkg::REG_ADDR_SIZE-1:0]] == k && (CVA6Cfg.FpPresent && ariane_pkg::is_imm_fpr(issue_instr_i[i].op)))) & fwd_i.still_issued[k] & fwd_i.sbe[k].valid ;
      assign rs_data[i][k+CVA6Cfg.NrWbPorts] = fwd_i.sbe[k].result;
    end

    // use fixed prio here
    // this implicitly gives higher prio to WB ports
    rr_arb_tree #(
        .NumIn(CVA6Cfg.NR_SB_ENTRIES + CVA6Cfg.NrWbPorts),
        .DataWidth(CVA6Cfg.XLEN),
        .ExtPrio(1'b1),
        .AxiVldRdy(1'b1)
    ) i_sel_rs1 (
        .clk_i  (clk_i),
        .rst_ni (rst_ni),
        .flush_i(1'b0),
        .rr_i   ('0),
        .req_i  (rs1_fwd_req[i]),
        .gnt_o  (),
        .data_i (rs_data[i]),
        .gnt_i  (1'b1),
        .req_o  (rs1_available[i]),
        .data_o (rs1_res[i]),
        .idx_o  ()
    );

    rr_arb_tree #(
        .NumIn(CVA6Cfg.NR_SB_ENTRIES + CVA6Cfg.NrWbPorts),
        .DataWidth(CVA6Cfg.XLEN),
        .ExtPrio(1'b1),
        .AxiVldRdy(1'b1)
    ) i_sel_rs2 (
        .clk_i  (clk_i),
        .rst_ni (rst_ni),
        .flush_i(1'b0),
        .rr_i   ('0),
        .req_i  (rs2_fwd_req[i]),
        .gnt_o  (),
        .data_i (rs_data[i]),
        .gnt_i  (1'b1),
        .req_o  (rs2_available[i]),
        .data_o (rs2_res[i]),
        .idx_o  ()
    );


    rr_arb_tree #(
        .NumIn(CVA6Cfg.NR_SB_ENTRIES + CVA6Cfg.NrWbPorts),
        .DataWidth(CVA6Cfg.XLEN),
        .ExtPrio(1'b1),
        .AxiVldRdy(1'b1)
    ) i_sel_rs3 (
        .clk_i  (clk_i),
        .rst_ni (rst_ni),
        .flush_i(1'b0),
        .rr_i   ('0),
        .req_i  (rs3_fwd_req[i]),
        .gnt_o  (),
        .data_i (rs_data[i]),
        .gnt_i  (1'b1),
        .req_o  (rs3_available[i]),
        .data_o (rs3[i]),
        .idx_o  ()
    );

    if (CVA6Cfg.NrRgprPorts == 3) begin : gen_gp_three_port
      assign rs3_res[i] = rs3[i][riscv::XLEN-1:0];
    end else begin : gen_fp_three_port
      assign rs3_res[i] = rs3[i][CVA6Cfg.FLen-1:0];
    end

    assign rs1_has_raw[i] = !issue_instr_i[i].use_zimm && ((CVA6Cfg.FpPresent && is_rs1_fpr(
        issue_instr_i[i].op
    )) ? rd_clobber_fpr[issue_instr_i[i].rs1] != CVA6Cfg.NR_SB_ENTRIES :
        rd_clobber_gpr[issue_instr_i[i].rs1] != CVA6Cfg.NR_SB_ENTRIES);

    assign rs1_valid[i] = rs1_available[i] && (CVA6Cfg.FpPresent && is_rs1_fpr(
        issue_instr_i[i].op
    ) ? 1'b1 : (!(csr_issued[rd_clobber_gpr[issue_instr_i[i].rs1]]) ||
                (CVA6Cfg.RVS && issue_instr_i[i].op == SFENCE_VMA)));

    assign rs2_has_raw[i] = ((CVA6Cfg.FpPresent && is_rs2_fpr(
        issue_instr_i[i].op
    )) ? rd_clobber_fpr[issue_instr_i[i].rs2] != CVA6Cfg.NR_SB_ENTRIES :
        rd_clobber_gpr[issue_instr_i[i].rs2] != CVA6Cfg.NR_SB_ENTRIES);

    assign rs2_valid[i] = rs2_available[i] && (CVA6Cfg.FpPresent && is_rs2_fpr(
        issue_instr_i[i].op
    ) ? 1'b1 : (!(csr_issued[rd_clobber_gpr[issue_instr_i[i].rs2]]) ||
                (CVA6Cfg.RVS && issue_instr_i[i].op == SFENCE_VMA)));

    assign rs3_has_raw[i] = ((CVA6Cfg.FpPresent && is_imm_fpr(
        issue_instr_i[i].op
    )) ? rd_clobber_fpr[issue_instr_i[i].result[REG_ADDR_SIZE-1:0]] != CVA6Cfg.NR_SB_ENTRIES : 0);

    assign rs3_valid[i] = rs3_available[i];
    assign rs3_fpr[i] = (CVA6Cfg.FpPresent && ariane_pkg::is_imm_fpr(issue_instr_i[i].op));

  end

  // Signal for performance monitoring

  mo_t missing_op;

  // ---------------
  // Register stage
  // ---------------
  // check that all operands are available, otherwise stall
  // forward corresponding register
  always_comb begin : operands_available
    missing_op = '0;
    fuse = '0;

    stall_raw   = '{default: stall_i};
    stall_rs1   = '{default: stall_i};
    stall_rs2   = '{default: stall_i};
    stall_rs3   = '{default: stall_i};
    // operand forwarding signals
    forward_rs1 = '0;
    forward_rs2 = '0;
    forward_rs3 = '0;  // FPR only

    for (int unsigned i = 0; i < CVA6Cfg.NrIssuePorts; i++) begin
      if (rs1_has_raw[i]) begin
        if (rs1_valid[i]) begin
          forward_rs1[i] = 1'b1;
        end else begin  // the operand is not available -> stall
          stall_raw[i] = 1'b1;
          stall_rs1[i] = 1'b1;
        end
      end

      if (rs2_has_raw[i]) begin
        if (rs2_valid[i]) begin
          forward_rs2[i] = 1'b1;
        end else begin  // the operand is not available -> stall
          stall_raw[i] = 1'b1;
          stall_rs2[i] = 1'b1;
        end
      end

      if (rs3_has_raw[i] && rs3_fpr[i]) begin
        if (rs3_valid[i]) begin
          forward_rs3[i] = 1'b1;
        end else begin  // the operand is not available -> stall
          stall_raw[i] = 1'b1;
          stall_rs3[i] = 1'b1;
        end
      end
    end

    if (CVA6Cfg.CvxifEn) begin
      // Remove unecessary forward and stall in case source register is not needed by coprocessor.
      if (x_issue_valid_o && x_issue_resp_i.accept) begin
        if (~x_issue_resp_i.register_read[0]) begin
          forward_rs1[0] = 1'b0;
          stall_rs1[0]   = 1'b0;
        end
        if (~x_issue_resp_i.register_read[1]) begin
          forward_rs2[0] = 1'b0;
          stall_rs2[0]   = 1'b0;
        end
        if (OPERANDS_PER_INSTR == 3 && ~x_issue_resp_i.register_read[2]) begin
          forward_rs3[0] = 1'b0;
          stall_rs3[0]   = 1'b0;
        end
      end
      stall_raw[0] = stall_rs1[0] || stall_rs2[0] || stall_rs3[0];
    end

    if (CVA6Cfg.SuperscalarEn) begin
      if (!issue_instr_i[1].use_zimm && (!CVA6Cfg.FpPresent || (is_rs1_fpr(
              issue_instr_i[1].op
          ) == is_rd_fpr(
              issue_instr_i[0].op
          ))) && issue_instr_i[1].rs1 == issue_instr_i[0].rd && issue_instr_i[1].rs1 != '0) begin
           // Check if the operations are ALU -> ALU, if not, stall.
           if ( !CVA6Cfg.FUSE || ( CVA6Cfg.FUSE && !(issue_instr_i[0].fu == ALU && issue_instr_i[1].fu == ALU) ) ) begin
               stall_raw[1] = 1'b1; // RS1[1] NEEDS RD[0]
               missing_op.rs1_rd = 1'b1;
           // If it is a ALU -> ALU, we can fuse all operation beside CPOP (maybe can be optimized OP -> CPOP, to explore).
           end else if ( ((issue_instr_i[0].op inside {CPOP, CPOPW}) || (issue_instr_i[1].op inside {CPOP, CPOPW}) )) begin 
               stall_raw[1] = 1'b1; // RS1[1] NEEDS RD[0]
               missing_op.rs1_rd = 1'b1;                
           end else begin 
               // We cannot fuse if we have the same destination register, it will create a forwarding inconsistency.
               fuse.rs1_into_rd = issue_instr_i[0].rd != issue_instr_i[1].rd;
           end
        end

      if ((!CVA6Cfg.FpPresent || (is_rs2_fpr(
              issue_instr_i[1].op
          ) == is_rd_fpr(
              issue_instr_i[0].op
          ))) && issue_instr_i[1].rs2 == issue_instr_i[0].rd && issue_instr_i[1].rs2 != '0) begin
            if ( !CVA6Cfg.FUSE || ( CVA6Cfg.FUSE && !(issue_instr_i[0].fu == ALU && issue_instr_i[1].fu == ALU) ) ) begin
                stall_raw[1] = 1'b1; // RS2[1] NEEDS RD[0]
                missing_op.rs2_rd = 1'b1;
            // If it is a ALU -> ALU, we can fuse all operation beside CPOP (maybe can be optimized OP -> CPOP, to explore).
            end else if ( ((issue_instr_i[0].op inside {CPOP, CPOPW}) || (issue_instr_i[1].op inside {CPOP, CPOPW}) )) begin 
                stall_raw[1] = 1'b1; // RS1[1] NEEDS RD[0]
                missing_op.rs2_rd = 1'b1;                
            end else begin 
                // We cannot fuse if we have the same destination register, it will create a forwarding inconsistency.
                fuse.rs2_into_rd = issue_instr_i[0].rd != issue_instr_i[1].rd;
            end
      end

      // Only check clobbered gpr for OFFLOADED instruction
      if ((CVA6Cfg.FpPresent && is_imm_fpr(
              issue_instr_i[1].op
          )) ? is_rd_fpr(
              issue_instr_i[0].op
          ) && issue_instr_i[0].rd == issue_instr_i[1].result[REG_ADDR_SIZE-1:0] :
              issue_instr_i[1].op == OFFLOAD && OPERANDS_PER_INSTR == 3 ?
              issue_instr_i[0].rd == issue_instr_i[1].result[REG_ADDR_SIZE-1:0] : 1'b0) begin
        stall_raw[1] = 1'b1;
        missing_op.rs3_rd = 1'b1;
      end
    end
  end

  // third operand from fp regfile or gp regfile if NR_RGPR_PORTS == 3
  for (genvar i = 0; i < CVA6Cfg.NrIssuePorts; i++) begin
    if (OPERANDS_PER_INSTR == 3) begin : gen_gp_rs3
      assign imm_forward_rs3[i] = rs3_res[i];
    end else begin : gen_fp_rs3
      assign imm_forward_rs3[i] = {{CVA6Cfg.XLEN - CVA6Cfg.FLen{1'b0}}, rs3_res[i]};
    end
  end

  // Forwarding/Output MUX
  for (genvar i = 0; i < CVA6Cfg.NrIssuePorts; i++) begin
    always_comb begin : forwarding_operand_select
      // default is regfiles (gpr or fpr)
      fu_data_n[i].operand_a = operand_a_regfile[i];
      fu_data_n[i].operand_b = operand_b_regfile[i];

      // immediates are the third operands in the store case
      // for FP operations, the imm field can also be the third operand from the regfile
      if (OPERANDS_PER_INSTR == 3) begin
        fu_data_n[i].imm = (CVA6Cfg.FpPresent && is_imm_fpr(issue_instr_i[i].op)) ?
            {{CVA6Cfg.XLEN - CVA6Cfg.FLen{1'b0}}, operand_c_regfile[i]} :
            issue_instr_i[i].op == OFFLOAD ? operand_c_regfile[i] : issue_instr_i[i].result;
      end else begin
        fu_data_n[i].imm = (CVA6Cfg.FpPresent && is_imm_fpr(issue_instr_i[i].op)) ?
            {{CVA6Cfg.XLEN - CVA6Cfg.FLen{1'b0}}, operand_c_regfile[i]} : issue_instr_i[i].result;
      end
      fu_data_n[i].trans_id  = issue_instr_i[i].trans_id;
      fu_data_n[i].fu        = issue_instr_i[i].fu;
      fu_data_n[i].operation = issue_instr_i[i].op;
      if (CVA6Cfg.RVH) begin
        tinst_n[i] = issue_instr_i[i].ex.tinst;
      end

      // or should we forward
      if (forward_rs1[i]) begin
        fu_data_n[i].operand_a = rs1_res[i];
      end
      if (forward_rs2[i]) begin
        fu_data_n[i].operand_b = rs2_res[i];
      end
      if ((CVA6Cfg.FpPresent || (CVA6Cfg.CvxifEn && OPERANDS_PER_INSTR == 3)) && forward_rs3[i]) begin
        fu_data_n[i].imm = imm_forward_rs3[i];
      end

      // use the PC as operand a
      if (issue_instr_i[i].use_pc) begin
        fu_data_n[i].operand_a = {
          {CVA6Cfg.XLEN - CVA6Cfg.VLEN{issue_instr_i[i].pc[CVA6Cfg.VLEN-1]}}, issue_instr_i[i].pc
        };
      end

      // use the zimm as operand a
      if (issue_instr_i[i].use_zimm) begin
        // zero extend operand a
        fu_data_n[i].operand_a = {{CVA6Cfg.XLEN - 5{1'b0}}, issue_instr_i[i].rs1[4:0]};
      end
      // or is it an immediate (including PC), this is not the case for a store, control flow, and accelerator instructions
      // also make sure operand B is not already used as an FP operand
      if (issue_instr_i[i].use_imm && (issue_instr_i[i].fu != STORE) && (issue_instr_i[i].fu != CTRL_FLOW) && (issue_instr_i[i].fu != ACCEL) && !(CVA6Cfg.FpPresent && is_rs2_fpr(
              issue_instr_i[i].op
          ))) begin
        fu_data_n[i].operand_b = issue_instr_i[i].result;
      end
    end
  end

  // FU select, assert the correct valid out signal (in the next cycle)
  // This needs to be like this to make verilator happy. I know its ugly.
  always_ff @(posedge clk_i or negedge rst_uarch_ni) begin
    if (!rst_uarch_ni) begin
      alu_valid_q    <= '0;
      lsu_valid_q    <= '0;
      mult_valid_q   <= '0;
      fpu_valid_q    <= '0;
      fpu_fmt_q      <= '0;
      fpu_rm_q       <= '0;
      alu2_valid_q   <= '0;
      csr_valid_q    <= '0;
      branch_valid_q <= '0;
    end else begin
      alu_valid_q    <= '0;
      lsu_valid_q    <= '0;
      mult_valid_q   <= '0;
      fpu_valid_q    <= '0;
      fpu_fmt_q      <= '0;
      fpu_rm_q       <= '0;
      alu2_valid_q   <= '0;
      csr_valid_q    <= '0;
      branch_valid_q <= '0;
      // Exception pass through:
      // If an exception has occurred simply pass it through
      // we do not want to issue this instruction
      for (int unsigned i = 0; i < CVA6Cfg.NrIssuePorts; i++) begin
        if (!issue_instr_i[i].ex.valid && issue_instr_valid_i[i] && issue_ack_o[i]) begin
          case (issue_instr_i[i].fu)
            ALU: begin
              if (CVA6Cfg.SuperscalarEn && !fus_busy[i].alu2) begin
                alu2_valid_q[i] <= 1'b1;
              end else begin
                alu_valid_q[i] <= 1'b1;
              end
            end
            CTRL_FLOW: begin
              branch_valid_q[i] <= 1'b1;
            end
            MULT: begin
              mult_valid_q[i] <= 1'b1;
            end
            LOAD, STORE: begin
              lsu_valid_q[i] <= 1'b1;
            end
            CSR: begin
              csr_valid_q[i] <= 1'b1;
            end
            default: begin
              if (issue_instr_i[i].fu == FPU && CVA6Cfg.FpPresent) begin
                fpu_valid_q[i] <= 1'b1;
                fpu_fmt_q      <= orig_instr[i].rftype.fmt;  // fmt bits from instruction
                fpu_rm_q       <= orig_instr[i].rftype.rm;  // rm bits from instruction
              end else if (issue_instr_i[i].fu == FPU_VEC && CVA6Cfg.FpPresent) begin
                fpu_valid_q[i] <= 1'b1;
                fpu_fmt_q      <= orig_instr[i].rvftype.vfmt;  // vfmt bits from instruction
                fpu_rm_q       <= {2'b0, orig_instr[i].rvftype.repl};  // repl bit from instruction
              end
            end
          endcase
        end
      end
      // if we got a flush request, de-assert the valid flag, otherwise we will start this
      // functional unit with the wrong inputs
      if (flush_i) begin
        alu_valid_q    <= '0;
        lsu_valid_q    <= '0;
        mult_valid_q   <= '0;
        fpu_valid_q    <= '0;
        alu2_valid_q   <= '0;
        csr_valid_q    <= '0;
        branch_valid_q <= '0;
      end
    end
  end

  if (CVA6Cfg.CvxifEn) begin
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) begin
        cvxif_valid_q <= '0;
        cvxif_off_instr_q <= 32'b0;
      end else begin
        cvxif_valid_q <= '0;
        cvxif_off_instr_q <= 32'b0;
        for (int unsigned i = 0; i < CVA6Cfg.NrIssuePorts; i++) begin
          if (!issue_instr_i[i].ex.valid && issue_instr_valid_i[i] && issue_ack_o[i]) begin
            case (issue_instr_i[i].fu)
              CVXIF: begin
                cvxif_valid_q[i]  <= 1'b1;
                cvxif_off_instr_q <= orig_instr[i];
              end
              default: ;
            endcase
          end
        end
        if (flush_i) begin
          cvxif_valid_q <= '0;
          cvxif_off_instr_q <= 32'b0;
        end
      end
    end
  end

  always_comb begin : gen_check_waw_dependencies
    stall_waw = '0;
  end


  // We can issue an instruction if we do not detect that any other instruction is writing the same
  // destination register.
  // We also need to check if there is an unresolved branch in the scoreboard.
  always_comb begin : issue_scoreboard
    for (int unsigned i = 0; i < CVA6Cfg.NrIssuePorts; i++) begin
      // default assignment
      issue_ack[i] = 1'b0;
      // check that the instruction we got is valid
      // and that the functional unit we need is not busy
      if (issue_instr_valid_i[i] && !fu_busy[i]) begin
        if (!stall_raw[i] && !stall_waw[i]) begin
          issue_ack[i] = 1'b1;
          // For now keep this as is, we cannot do inter dual-issue forwarding
          if (i > 0) begin
            if ((issue_instr_i[i].rd[4:0] == issue_instr_i[i-1].rd[4:0]) && (issue_instr_i[i].rd[4:0] != '0)) begin
              if (!(fuse.rs1_into_rd || fuse.rs2_into_rd)) begin
                  issue_ack[i] = 1'b0;
              end
            end
          end
        end
        // we can also issue the instruction under the following two circumstances:
        // we can do this even if we are stalled or no functional unit is ready (as we don't need one)
        // the decoder needs to make sure that the instruction is marked as valid when it does not
        // need any functional unit or if an exception occurred previous to the execute stage.
        // 1. we already got an exception
        if (issue_instr_i[i].ex.valid) begin
          issue_ack[i] = 1'b1;
        end
        // 2. it is an instruction which does not need any functional unit
        if (issue_instr_i[i].fu == NONE) begin
          issue_ack[i] = 1'b1;
        end
      end
    end

    if (CVA6Cfg.SuperscalarEn) begin
      if (!issue_ack[0]) begin
        issue_ack[1] = 1'b0;
      end
    end
    issue_ack_o = issue_ack;
    // Do not acknoledge the issued instruction if transaction is not completed.
    if (issue_instr_i[0].fu == CVXIF && !(x_transaction_accepted_o || x_transaction_rejected)) begin
      issue_ack_o[0] = issue_instr_i[0].ex.valid && issue_instr_valid_i[0];
    end
  end

  // ----------------------
  // Integer Register File
  // ----------------------
  logic [  CVA6Cfg.NrRgprPorts-1:0][CVA6Cfg.XLEN-1:0] rdata;
  logic [  CVA6Cfg.NrRgprPorts-1:0][             4:0] raddr_pack;

  // pack signals
  logic [CVA6Cfg.NrCommitPorts-1:0][             4:0] waddr_pack;
  logic [CVA6Cfg.NrCommitPorts-1:0][CVA6Cfg.XLEN-1:0] wdata_pack;
  logic [CVA6Cfg.NrCommitPorts-1:0]                   we_pack;

  for (genvar i = 0; i < CVA6Cfg.NrIssuePorts; i++) begin
    assign raddr_pack[i*OPERANDS_PER_INSTR+0] = issue_instr_i[i].rs1;
    assign raddr_pack[i*OPERANDS_PER_INSTR+1] = issue_instr_i[i].rs2;
    if (OPERANDS_PER_INSTR == 3) begin
      assign raddr_pack[i*OPERANDS_PER_INSTR+2] = issue_instr_i[i].result[4:0];
    end
  end

  for (genvar i = 0; i < CVA6Cfg.NrCommitPorts; i++) begin : gen_write_back_port
    assign waddr_pack[i] = waddr_i[i];
    assign wdata_pack[i] = wdata_i[i];
    assign we_pack[i]    = we_gpr_i[i];
  end
  if (CVA6Cfg.FpgaEn) begin : gen_fpga_regfile
    ariane_regfile_fpga #(
        .CVA6Cfg      (CVA6Cfg),
        .DATA_WIDTH   (CVA6Cfg.XLEN),
        .NR_READ_PORTS(CVA6Cfg.NrRgprPorts),
        .ZERO_REG_ZERO(1)
    ) i_ariane_regfile_fpga (
        .test_en_i(1'b0),
        .raddr_i  (raddr_pack),
        .rdata_o  (rdata),
        .waddr_i  (waddr_pack),
        .wdata_i  (wdata_pack),
        .we_i     (we_pack),
        .*
    );
  end else begin : gen_asic_regfile
    ariane_regfile #(
        .CVA6Cfg      (CVA6Cfg),
        .DATA_WIDTH   (CVA6Cfg.XLEN),
        .NR_READ_PORTS(CVA6Cfg.NrRgprPorts),
        .ZERO_REG_ZERO(1)
    ) i_ariane_regfile (
        .test_en_i(1'b0),
        .raddr_i  (raddr_pack),
        .rdata_o  (rdata),
        .waddr_i  (waddr_pack),
        .wdata_i  (wdata_pack),
        .we_i     (we_pack),
        .*
    );
  end

  // -----------------------------
  // Floating-Point Register File
  // -----------------------------
  logic [2:0][CVA6Cfg.FLen-1:0] fprdata;

  // pack signals
  logic [2:0][4:0] fp_raddr_pack;
  logic [CVA6Cfg.NrCommitPorts-1:0][CVA6Cfg.XLEN-1:0] fp_wdata_pack;

  always_comb begin : assign_fp_raddr_pack
    fp_raddr_pack = {
      issue_instr_i[0].result[4:0], issue_instr_i[0].rs2[4:0], issue_instr_i[0].rs1[4:0]
    };

    if (CVA6Cfg.SuperscalarEn) begin
      if (!(issue_instr_i[0].fu inside {FPU, FPU_VEC} || issue_instr_i[0].op inside {[FLD:FSB]})) begin
        fp_raddr_pack = {
          issue_instr_i[1].result[4:0], issue_instr_i[1].rs2[4:0], issue_instr_i[1].rs1[4:0]
        };
      end
    end
  end

  generate
    if (CVA6Cfg.FpPresent) begin : float_regfile_gen
      for (genvar i = 0; i < CVA6Cfg.NrCommitPorts; i++) begin : gen_fp_wdata_pack
        assign fp_wdata_pack[i] = {wdata_i[i][CVA6Cfg.FLen-1:0]};
      end
      if (CVA6Cfg.FpgaEn) begin : gen_fpga_fp_regfile
        ariane_regfile_fpga #(
            .CVA6Cfg      (CVA6Cfg),
            .DATA_WIDTH   (CVA6Cfg.FLen),
            .NR_READ_PORTS(3),
            .ZERO_REG_ZERO(0)
        ) i_ariane_fp_regfile_fpga (
            .test_en_i(1'b0),
            .raddr_i  (fp_raddr_pack),
            .rdata_o  (fprdata),
            .waddr_i  (waddr_pack),
            .wdata_i  (fp_wdata_pack),
            .we_i     (we_fpr_i),
            .*
        );
      end else begin : gen_asic_fp_regfile
        ariane_regfile #(
            .CVA6Cfg      (CVA6Cfg),
            .DATA_WIDTH   (CVA6Cfg.FLen),
            .NR_READ_PORTS(3),
            .ZERO_REG_ZERO(0)
        ) i_ariane_fp_regfile (
            .test_en_i(1'b0),
            .raddr_i  (fp_raddr_pack),
            .rdata_o  (fprdata),
            .waddr_i  (waddr_pack),
            .wdata_i  (fp_wdata_pack),
            .we_i     (we_fpr_i),
            .*
        );
      end
    end else begin : no_fpr_gen
      assign fprdata = '{default: '0};
    end
  endgenerate

  if (OPERANDS_PER_INSTR == 3) begin : gen_operand_c
    assign operand_c_fpr = {{CVA6Cfg.XLEN - CVA6Cfg.FLen{1'b0}}, fprdata[2]};
  end else begin
    assign operand_c_fpr = fprdata[2];
  end

  for (genvar i = 0; i < CVA6Cfg.NrIssuePorts; i++) begin
    if (OPERANDS_PER_INSTR == 3) begin : gen_operand_c
      assign operand_c_gpr[i] = rdata[i*OPERANDS_PER_INSTR+2];
    end

    assign operand_a_regfile[i] = (CVA6Cfg.FpPresent && is_rs1_fpr(
        issue_instr_i[i].op
    )) ? {{CVA6Cfg.XLEN - CVA6Cfg.FLen{1'b0}}, fprdata[0]} : rdata[i*OPERANDS_PER_INSTR+0];
    assign operand_b_regfile[i] = (CVA6Cfg.FpPresent && is_rs2_fpr(
        issue_instr_i[i].op
    )) ? {{CVA6Cfg.XLEN - CVA6Cfg.FLen{1'b0}}, fprdata[1]} : rdata[i*OPERANDS_PER_INSTR+1];
    assign operand_c_regfile[i] = (OPERANDS_PER_INSTR == 3) ? ((CVA6Cfg.FpPresent && is_imm_fpr(
        issue_instr_i[i].op
    )) ? operand_c_fpr : operand_c_gpr[i]) : operand_c_fpr;
  end

  // ----------------------
  // Registers (ID <-> EX)
  // ----------------------
  always_ff @(posedge clk_i or negedge rst_uarch_ni) begin
    if (!rst_uarch_ni) begin
      fu_data_q <= '0;
      if (CVA6Cfg.RVH) begin
        tinst_q <= '0;
      end
      pc_o                     <= '0;
      is_compressed_instr_o    <= 1'b0;
      branch_predict_o         <= {cf_t'(0), {CVA6Cfg.VLEN{1'b0}}};
      x_transaction_rejected_o <= 1'b0;
      fuse_q                   <= '0;
    end else begin
      fu_data_q <= fu_data_n;
      fuse_q <= &issue_ack_o ? fuse : '0;
      if (CVA6Cfg.RVH) begin
        tinst_q <= tinst_n;
      end
      if (CVA6Cfg.SuperscalarEn) begin
        if (issue_instr_i[1].fu == CTRL_FLOW) begin
          pc_o                  <= issue_instr_i[1].pc;
          is_compressed_instr_o <= issue_instr_i[1].is_compressed;
          branch_predict_o      <= issue_instr_i[1].bp;
        end
      end
      if (issue_instr_i[0].fu == CTRL_FLOW) begin
        pc_o                  <= issue_instr_i[0].pc;
        is_compressed_instr_o <= issue_instr_i[0].is_compressed;
        branch_predict_o      <= issue_instr_i[0].bp;
      end
      x_transaction_rejected_o <= 1'b0;
      if (issue_instr_i[0].fu == CVXIF) begin
        x_transaction_rejected_o <= x_transaction_rejected;
      end
    end
  end

  //pragma translate_off
  initial begin
    assert (OPERANDS_PER_INSTR == 2 || (OPERANDS_PER_INSTR == 3 && CVA6Cfg.CvxifEn))
    else
      $fatal(
          1,
          "If CVXIF is enable, ariane regfile can have either 2 or 3 read ports. Else it has 2 read ports."
      );
  end

  for (genvar i = 0; i < CVA6Cfg.NrIssuePorts; i++) begin
    assert property (@(posedge clk_i) (branch_valid_q) |-> (!$isunknown(
        fu_data_q[i].operand_a
    ) && !$isunknown(
        fu_data_q[i].operand_b
    )))
    else $warning("Got unknown value in one of the operands");
  end
  //pragma translate_on

endmodule
