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
// Date: 19.04.2017
// Description: Load Store Unit, handles address calculation and memory interface signals


module load_store_unit
  import ariane_pkg::*;
#(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty,
    parameter int unsigned ASID_WIDTH = 1,
    parameter int unsigned VMID_WIDTH = 1
) (
    // Subsystem Clock - SUBSYSTEM
    input logic clk_i,
    // Asynchronous reset active low - SUBSYSTEM
    input logic rst_ni,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    input logic flush_i,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    input logic stall_st_pending_i,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    output logic no_st_pending_o,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    input logic amo_valid_commit_i,
    // FU data needed to execute instruction - ISSUE_STAGE
    input fu_data_t fu_data_i,
    // Load Store Unit is ready - ISSUE_STAGE
    output logic lsu_ready_o,
    // Load Store Unit instruction is valid - ISSUE_STAGE
    input logic lsu_valid_i,

    input riscv::xlen_t tinst_i,

    // Load transaction ID - ISSUE_STAGE
    output logic [TRANS_ID_BITS-1:0] load_trans_id_o,
    // Load result - ISSUE_STAGE
    output riscv::xlen_t load_result_o,
    // Load result is valid - ISSUE_STAGE
    output logic load_valid_o,
    // Load exception - ISSUE_STAGE
    output exception_t load_exception_o,

    // Store transaction ID - ISSUE_STAGE
    output logic [TRANS_ID_BITS-1:0] store_trans_id_o,
    // Store result - ISSUE_STAGE
    output riscv::xlen_t store_result_o,
    // Store result is valid - ISSUE_STAGE
    output logic store_valid_o,
    // Store exception - ISSUE_STAGE
    output exception_t store_exception_o,

    // Commit the first pending store - TO_BE_COMPLETED
    input logic commit_i,
    // Commit queue is ready to accept another commit request - TO_BE_COMPLETED
    output logic commit_ready_o,
    // Commit transaction ID - TO_BE_COMPLETED
    input logic [TRANS_ID_BITS-1:0] commit_tran_id_i,

    // Enable virtual memory translation - TO_BE_COMPLETED
    input logic enable_translation_i,
    input logic enable_g_translation_i,   // enable G-Stage translation
    // Enable virtual memory translation for load/stores - TO_BE_COMPLETED
    input logic en_ld_st_translation_i,
    input logic en_ld_st_g_translation_i, // enable G-stage translation for load/stores

    // Accelerator request for CVA6's MMU
    input  acc_pkg::acc_mmu_req_t acc_mmu_req_i,
    output acc_pkg::acc_mmu_resp_t acc_mmu_resp_o,

    // Instruction cache input request - CACHES
    input  icache_arsp_t icache_areq_i,
    // Instruction cache output request - CACHES
    output icache_areq_t icache_areq_o,

    // Current privilege mode - CSR_REGFILE
    input  riscv::priv_lvl_t                    priv_lvl_i,
    input  logic                                v_i,                     // From CSR register file
    // Privilege level at which load and stores should happen - CSR_REGFILE
    input  riscv::priv_lvl_t                    ld_st_priv_lvl_i,
    input  logic                                ld_st_v_i,               // From CSR register file
    output logic                                csr_hs_ld_st_inst_o,
    // Supervisor User Memory - CSR_REGFILE
    input  logic                                sum_i,
    input  logic                                vs_sum_i,                // From CSR register file
    // Make Executable Readable - CSR_REGFILE
    input  logic                                mxr_i,
    input  logic                                vmxr_i,                  // From CSR register file
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    input  logic             [ riscv::PPNW-1:0] satp_ppn_i,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    input  logic             [  ASID_WIDTH-1:0] asid_i,
    input  logic             [ riscv::PPNW-1:0] vsatp_ppn_i,             // From CSR register file
    input  logic             [  ASID_WIDTH-1:0] vs_asid_i,               // From CSR register file
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    input  logic             [  ASID_WIDTH-1:0] asid_to_be_flushed_i,
    input  logic             [ riscv::PPNW-1:0] hgatp_ppn_i,             // From CSR register file
    input  logic             [  VMID_WIDTH-1:0] vmid_i,                  // From CSR register file
    input  logic             [  VMID_WIDTH-1:0] vmid_to_be_flushed_i,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    input  logic             [ riscv::VLEN-1:0] vaddr_to_be_flushed_i,
    input  logic             [riscv::GPLEN-1:0] gpaddr_to_be_flushed_i,
    // TLB flush - CONTROLLER
    input  logic                                flush_tlb_i,
    input  logic                                flush_tlb_vvma_i,
    input  logic                                flush_tlb_gvma_i,
    // Instruction TLB miss - PERF_COUNTERS
    output logic                                itlb_miss_o,
    // Data TLB miss - PERF_COUNTERS
    output logic                                dtlb_miss_o,

    // Data cache request output - CACHES
    input  dcache_req_o_t  [ 2:0]                  dcache_req_ports_i,
    // Data cache request input - CACHES
    output dcache_req_i_t  [ 2:0]                  dcache_req_ports_o,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    input  logic                                   dcache_wbuffer_empty_i,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    input  logic                                   dcache_wbuffer_not_ni_i,
    // AMO request - CACHE
    output amo_req_t                               amo_req_o,
    // AMO response - CACHE
    input  amo_resp_t                              amo_resp_i,
    // PMP configuration - CSR_REGFILE
    input  riscv::pmpcfg_t [15:0]                  pmpcfg_i,
    // PMP address - CSR_REGFILE
    input  logic           [15:0][riscv::PLEN-3:0] pmpaddr_i,

    // RVFI inforamtion - RVFI
    output lsu_ctrl_t                   rvfi_lsu_ctrl_o,
    // RVFI information - RVFI
    output            [riscv::PLEN-1:0] rvfi_mem_paddr_o
);
  // data is misaligned
  logic                           data_misaligned;
  // --------------------------------------
  // 1st register stage - (stall registers)
  // --------------------------------------
  // those are the signals which are always correct
  // e.g.: they keep the value in the stall case
  lsu_ctrl_t                      lsu_ctrl, lsu_ctrl_byp;

  logic                           pop_st;
  logic                           pop_ld;

  // ------------------------------
  // Address Generation Unit (AGU)
  // ------------------------------
  // virtual address as calculated by the AGU in the first cycle
  logic         [riscv::VLEN-1:0] vaddr_i;
  riscv::xlen_t                   vaddr_xlen;
  logic overflow, g_overflow;
  logic [(riscv::XLEN/8)-1:0] be_i;

  assign vaddr_xlen = $unsigned($signed(fu_data_i.imm) + $signed(fu_data_i.operand_a));
  assign vaddr_i = vaddr_xlen[riscv::VLEN-1:0];
  // we work with SV39 or SV32, so if VM is enabled, check that all bits [XLEN-1:38] or [XLEN-1:31] are equal
  assign overflow = (riscv::IS_XLEN64 && (!((&vaddr_xlen[riscv::XLEN-1:riscv::SV-1]) == 1'b1 || (|vaddr_xlen[riscv::XLEN-1:riscv::SV-1]) == 1'b0)));
  assign g_overflow = (CVA6Cfg.RVH) ? !((|vaddr_xlen[riscv::XLEN-1:riscv::SVX]) == 1'b0) : 1'b0;

  logic                   st_valid_i;
  logic                   ld_valid_i;
  logic                   ld_translation_req;
  logic                   cva6_st_translation_req, acc_st_translation_req, st_translation_req;
  logic [riscv::VLEN-1:0] ld_vaddr;
  logic [riscv::XLEN-1:0] ld_tinst;
  logic                   ld_hs_ld_st_inst;
  logic                   ld_hlvx_inst;
  logic [riscv::VLEN-1:0] st_vaddr;
  logic [riscv::XLEN-1:0] st_tinst;
  logic                   st_hs_ld_st_inst;
  logic                   st_hlvx_inst;
  logic                   cva6_translation_req, acc_translation_req, translation_req;
  logic                   cva6_translation_valid, acc_translataion_valid, translation_valid;
  logic [riscv::VLEN-1:0] cva6_mmu_vaddr, acc_mmu_vaddr, mmu_vaddr;
  logic [riscv::PLEN-1:0] cva6_mmu_paddr, acc_mmu_paddr, mmu_paddr, mmu_vaddr_plen, fetch_vaddr_plen;
  logic         [  riscv::XLEN-1:0] mmu_tinst;
  logic                             mmu_hs_ld_st_inst;
  logic                             mmu_hlvx_inst;
  exception_t                       cva6_mmu_exception, acc_mmu_exception, mmu_exception;
  logic                             cva6_dtlb_hit, acc_dtlb_hit, dtlb_hit;
  logic [riscv::PPNW-1:0]           cva6_dtlb_ppn, acc_dtlb_ppn, dtlb_ppn;

  logic                             ld_valid;
  logic         [TRANS_ID_BITS-1:0] ld_trans_id;
  riscv::xlen_t                     ld_result;
  logic                             st_valid;
  logic         [TRANS_ID_BITS-1:0] st_trans_id;
  riscv::xlen_t                     st_result;

  logic         [             11:0] page_offset;
  logic                             page_offset_matches;

  exception_t                       cva6_misaligned_exception, acc_misaligned_exception, misaligned_exception;
  exception_t                       ld_ex;
  exception_t                       st_ex;

  logic                             hs_ld_st_inst;
  logic                             hlvx_inst;

  // -------------------
  // MMU e.g.: TLBs/PTW
  // -------------------
  if (MMU_PRESENT && CVA6Cfg.RVH && (riscv::XLEN == 64)) begin : gen_mmu_sv39x4
    cva6_mmu_sv39x4 #(
        .CVA6Cfg          (CVA6Cfg),
        .INSTR_TLB_ENTRIES(ariane_pkg::INSTR_TLB_ENTRIES),
        .DATA_TLB_ENTRIES (ariane_pkg::DATA_TLB_ENTRIES),
        .ASID_WIDTH       (ASID_WIDTH),
        .VMID_WIDTH       (VMID_WIDTH)
    ) i_cva6_mmu (
        // misaligned bypass
        .misaligned_ex_i(misaligned_exception),
        .lsu_is_store_i (st_translation_req),
        .lsu_req_i      (translation_req),
        .lsu_vaddr_i    (mmu_vaddr),
        .lsu_tinst_i    (mmu_tinst),
        .lsu_valid_o    (translation_valid),
        .lsu_paddr_o    (mmu_paddr),
        .lsu_exception_o(mmu_exception),
        .lsu_dtlb_hit_o (dtlb_hit),               // send in the same cycle as the request
        .lsu_dtlb_ppn_o (dtlb_ppn),               // send in the same cycle as the request
        // connecting PTW to D$ IF
        .req_port_i     (dcache_req_ports_i[0]),
        .req_port_o     (dcache_req_ports_o[0]),
        // icache address translation requests
        .icache_areq_i  (icache_areq_i),
        .asid_to_be_flushed_i,
        .vmid_to_be_flushed_i,
        .vaddr_to_be_flushed_i,
        .gpaddr_to_be_flushed_i,
        .icache_areq_o  (icache_areq_o),
        .pmpcfg_i,
        .pmpaddr_i,
        // Hypervisor load/store signals
        .hlvx_inst_i    (mmu_hlvx_inst),
        .hs_ld_st_inst_i(mmu_hs_ld_st_inst),
        .*
    );
  end else if (MMU_PRESENT && (riscv::XLEN == 64)) begin : gen_mmu_sv39
    mmu #(
        .CVA6Cfg          (CVA6Cfg),
        .INSTR_TLB_ENTRIES(ariane_pkg::INSTR_TLB_ENTRIES),
        .DATA_TLB_ENTRIES (ariane_pkg::DATA_TLB_ENTRIES),
        .ASID_WIDTH       (ASID_WIDTH)
    ) i_cva6_mmu (
        // misaligned bypass
        .misaligned_ex_i(misaligned_exception),
        .lsu_is_store_i (st_translation_req),
        .lsu_req_i      (translation_req),
        .lsu_vaddr_i    (mmu_vaddr),
        .lsu_valid_o    (translation_valid),
        .lsu_paddr_o    (mmu_paddr),
        .lsu_exception_o(mmu_exception),
        .lsu_dtlb_hit_o (dtlb_hit),               // send in the same cycle as the request
        .lsu_dtlb_ppn_o (dtlb_ppn),               // send in the same cycle as the request
        // connecting PTW to D$ IF
        .req_port_i     (dcache_req_ports_i[0]),
        .req_port_o     (dcache_req_ports_o[0]),
        // icache address translation requests
        .icache_areq_i  (icache_areq_i),
        .asid_to_be_flushed_i,
        .vaddr_to_be_flushed_i,
        .icache_areq_o  (icache_areq_o),
        .pmpcfg_i,
        .pmpaddr_i,
        .*
    );
  end else if (MMU_PRESENT && (riscv::XLEN == 32)) begin : gen_mmu_sv32
    cva6_mmu_sv32 #(
        .CVA6Cfg          (CVA6Cfg),
        .INSTR_TLB_ENTRIES(ariane_pkg::INSTR_TLB_ENTRIES),
        .DATA_TLB_ENTRIES (ariane_pkg::DATA_TLB_ENTRIES),
        .ASID_WIDTH       (ASID_WIDTH)
    ) i_cva6_mmu (
        // misaligned bypass
        .misaligned_ex_i(misaligned_exception),
        .lsu_is_store_i (st_translation_req),
        .lsu_req_i      (translation_req),
        .lsu_vaddr_i    (mmu_vaddr),
        .lsu_valid_o    (translation_valid),
        .lsu_paddr_o    (mmu_paddr),
        .lsu_exception_o(mmu_exception),
        .lsu_dtlb_hit_o (dtlb_hit),               // send in the same cycle as the request
        .lsu_dtlb_ppn_o (dtlb_ppn),               // send in the same cycle as the request
        // connecting PTW to D$ IF
        .req_port_i     (dcache_req_ports_i[0]),
        .req_port_o     (dcache_req_ports_o[0]),
        // icache address translation requests
        .icache_areq_i  (icache_areq_i),
        .asid_to_be_flushed_i,
        .vaddr_to_be_flushed_i,
        .icache_areq_o  (icache_areq_o),
        .pmpcfg_i,
        .pmpaddr_i,
        .*
    );
  end else begin : gen_no_mmu

    if (riscv::VLEN > riscv::PLEN) begin
      assign mmu_vaddr_plen   = cva6_mmu_vaddr[riscv::PLEN-1:0];
      assign fetch_vaddr_plen = icache_areq_i.fetch_vaddr[riscv::PLEN-1:0];
    end else begin
      assign mmu_vaddr_plen   = {{{riscv::PLEN - riscv::VLEN} {1'b0}}, cva6_mmu_vaddr};
      assign fetch_vaddr_plen = {{{riscv::PLEN - riscv::VLEN} {1'b0}}, icache_areq_i.fetch_vaddr};
    end

    assign icache_areq_o.fetch_valid           = icache_areq_i.fetch_req;
    assign icache_areq_o.fetch_paddr           = fetch_vaddr_plen;
    assign icache_areq_o.fetch_exception       = '0;

    assign dcache_req_ports_o[0].address_index = '0;
    assign dcache_req_ports_o[0].address_tag   = '0;
    assign dcache_req_ports_o[0].data_wdata    = '0;
    assign dcache_req_ports_o[0].data_req      = 1'b0;
    assign dcache_req_ports_o[0].data_be       = '1;
    assign dcache_req_ports_o[0].data_size     = 2'b11;
    assign dcache_req_ports_o[0].data_we       = 1'b0;
    assign dcache_req_ports_o[0].kill_req      = '0;
    assign dcache_req_ports_o[0].tag_valid     = 1'b0;

    assign itlb_miss_o                         = 1'b0;
    assign dtlb_miss_o                         = 1'b0;
    assign cva6_dtlb_ppn                       = mmu_vaddr_plen[riscv::PLEN-1:12];
    assign cva6_dtlb_hit                       = 1'b1;

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (~rst_ni) begin
        cva6_mmu_paddr     <= '0;
        translation_valid  <= '0;
        cva6_mmu_exception <= '0;
      end else begin
        cva6_mmu_paddr     <= mmu_vaddr_plen;
        translation_valid  <= translation_req;
        cva6_mmu_exception <= misaligned_exception;
      end
    end
  end

  if (CVA6Cfg.EnableAccelerator) begin
    // The MMU can be connected to CVA6 or the ACCELERATOR
    enum logic {CVA6, ACC} mmu_state_d, mmu_state_q;
    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (~rst_ni) begin
        mmu_state_q <= CVA6;
      end else begin
        mmu_state_q <= mmu_state_d;
      end
    end

    // Straightforward and slow-reactive MMU arbitration logic
    // This logic can be optimized to reduce answer latency and contention
    always_comb begin
      // Maintain state
      mmu_state_d = mmu_state_q;

      // Serve CVA6 and gate the accelerator by default
      // MMU input
      misaligned_exception             = cva6_misaligned_exception;
      st_translation_req               = cva6_st_translation_req;
      translation_req                  = cva6_translation_req;
      mmu_vaddr                        = cva6_mmu_vaddr;
      // MMU output
      cva6_translation_valid           = translation_valid;
      cva6_mmu_paddr                   = mmu_paddr;
      cva6_mmu_exception               = mmu_exception;
      cva6_dtlb_hit                    = dtlb_hit;
      cva6_dtlb_ppn                    = dtlb_ppn;
      acc_mmu_resp_o.acc_mmu_valid     = '0;
      acc_mmu_resp_o.acc_mmu_paddr     = '0;
      acc_mmu_resp_o.acc_mmu_exception = '0;
      acc_mmu_resp_o.acc_mmu_dtlb_hit  = '0;
      acc_mmu_resp_o.acc_mmu_dtlb_ppn  = '0;

      unique case (mmu_state_q)
        CVA6: begin
          // Only the accelerator is requesting, and the lsu bypass queue is empty.
          if (acc_mmu_req_i.acc_mmu_req && !lsu_valid_i && lsu_ready_o) begin
            // Lock the MMU to the accelerator.
            // If the issue stage is firing a mem op in this cycle,
            // the bypass queue will buffer it.
            mmu_state_d = ACC;
          end
          // Make this a mealy FSM to cut some latency.
          // It should be okay timing-wise since cva6's requests already
          // depend on lsu_valid_i. Moreover, lsu_ready_o is sequentially
          // generated by the bypass and, in this first implementation,
          // the acc request already depends combinatorially upon acc_mmu_req_i.acc_mmu_req.
        end
        ACC: begin
          // MMU input
          misaligned_exception             = acc_mmu_req_i.acc_mmu_misaligned_ex;
          st_translation_req               = acc_mmu_req_i.acc_mmu_is_store;
          translation_req                  = acc_mmu_req_i.acc_mmu_req;
          mmu_vaddr                        = acc_mmu_req_i.acc_mmu_vaddr;
          // MMU output
          acc_mmu_resp_o.acc_mmu_valid     = translation_valid;
          acc_mmu_resp_o.acc_mmu_paddr     = mmu_paddr;
          acc_mmu_resp_o.acc_mmu_exception = mmu_exception;
          acc_mmu_resp_o.acc_mmu_dtlb_hit  = dtlb_hit;
          acc_mmu_resp_o.acc_mmu_dtlb_ppn  = dtlb_ppn;
          cva6_translation_valid           = '0;
          cva6_mmu_paddr                   = '0;
          cva6_mmu_exception               = '0;
          cva6_dtlb_hit                    = '0;
          cva6_dtlb_ppn                    = '0;
          // Get back to CVA6 after the translation
          if (translation_valid) mmu_state_d = CVA6;
        end
        default: mmu_state_d = CVA6;
      endcase
    end

    always_comb begin
      // Feed forward
      lsu_ctrl = lsu_ctrl_byp;
      // Mask the lsu valid so that cva6's req gets buffered in the
      // bypass queue when the MMU is being used by the accelerator.
      lsu_ctrl.valid = (mmu_state_q == ACC) ? 1'b0 : lsu_ctrl_byp.valid;
    end
  end else begin
    // MMU input
    assign misaligned_exception   = cva6_misaligned_exception;
    assign st_translation_req     = cva6_st_translation_req;
    assign translation_req        = cva6_translation_req;
    assign mmu_vaddr              = cva6_mmu_vaddr;
    // MMU output
    assign cva6_translation_valid = translation_valid;
    assign cva6_mmu_paddr         = mmu_paddr;
    assign cva6_mmu_exception     = mmu_exception;
    assign cva6_dtlb_hit          = dtlb_hit;
    assign cva6_dtlb_ppn          = dtlb_ppn;
    // No accelerator
    assign acc_mmu_resp_o = '0;
    // Feed forward the lsu_ctrl bypass
    assign lsu_ctrl = lsu_ctrl_byp;
  end

  logic store_buffer_empty;
  // ------------------
  // Store Unit
  // ------------------
  store_unit #(
      .CVA6Cfg(CVA6Cfg)
  ) i_store_unit (
      .clk_i,
      .rst_ni,
      .flush_i,
      .stall_st_pending_i,
      .no_st_pending_o,
      .store_buffer_empty_o(store_buffer_empty),

      .valid_i   (st_valid_i),
      .lsu_ctrl_i(lsu_ctrl),
      .pop_st_o  (pop_st),
      .commit_i,
      .commit_ready_o,
      .amo_valid_commit_i,

      .valid_o              (st_valid),
      .trans_id_o           (st_trans_id),
      .result_o             (st_result),
      .ex_o                 (st_ex),
      // MMU port
      .translation_req_o    (cva6_st_translation_req),
      .vaddr_o              (st_vaddr),
      .rvfi_mem_paddr_o     (rvfi_mem_paddr_o),
      .tinst_o              (st_tinst),
      .hs_ld_st_inst_o      (st_hs_ld_st_inst),
      .hlvx_inst_o          (st_hlvx_inst),
      .paddr_i              (cva6_mmu_paddr),
      .ex_i                 (cva6_mmu_exception),
      .dtlb_hit_i           (cva6_dtlb_hit),
      // Load Unit
      .page_offset_i        (page_offset),
      .page_offset_matches_o(page_offset_matches),
      // AMOs
      .amo_req_o,
      .amo_resp_i,
      // to memory arbiter
      .req_port_i           (dcache_req_ports_i[2]),
      .req_port_o           (dcache_req_ports_o[2])
  );

  // ------------------
  // Load Unit
  // ------------------
  load_unit #(
      .CVA6Cfg(CVA6Cfg)
  ) i_load_unit (
      .valid_i   (ld_valid_i),
      .lsu_ctrl_i(lsu_ctrl),
      .pop_ld_o  (pop_ld),

      .valid_o              (ld_valid),
      .trans_id_o           (ld_trans_id),
      .result_o             (ld_result),
      .ex_o                 (ld_ex),
      // MMU port
      .translation_req_o    (ld_translation_req),
      .vaddr_o              (ld_vaddr),
      .tinst_o              (ld_tinst),
      .hs_ld_st_inst_o      (ld_hs_ld_st_inst),
      .hlvx_inst_o          (ld_hlvx_inst),
      .paddr_i              (cva6_mmu_paddr),
      .ex_i                 (cva6_mmu_exception),
      .dtlb_hit_i           (cva6_dtlb_hit),
      .dtlb_ppn_i           (cva6_dtlb_ppn),
      // to store unit
      .page_offset_o        (page_offset),
      .page_offset_matches_i(page_offset_matches),
      .store_buffer_empty_i (store_buffer_empty),
      // to memory arbiter
      .req_port_i           (dcache_req_ports_i[1]),
      .req_port_o           (dcache_req_ports_o[1]),
      .dcache_wbuffer_not_ni_i,
      .commit_tran_id_i,
      .*
  );

  // ----------------------------
  // Output Pipeline Register
  // ----------------------------

  // amount of pipeline registers inserted for load/store return path
  // can be tuned to trade-off IPC vs. cycle time

  shift_reg #(
      .dtype(logic [$bits(ld_valid) + $bits(ld_trans_id) + $bits(ld_result) + $bits(ld_ex) - 1:0]),
      .Depth(cva6_config_pkg::CVA6ConfigNrLoadPipeRegs)
  ) i_pipe_reg_load (
      .clk_i,
      .rst_ni,
      .d_i({ld_valid, ld_trans_id, ld_result, ld_ex}),
      .d_o({load_valid_o, load_trans_id_o, load_result_o, load_exception_o})
  );

  shift_reg #(
      .dtype(logic [$bits(st_valid) + $bits(st_trans_id) + $bits(st_result) + $bits(st_ex) - 1:0]),
      .Depth(cva6_config_pkg::CVA6ConfigNrStorePipeRegs)
  ) i_pipe_reg_store (
      .clk_i,
      .rst_ni,
      .d_i({st_valid, st_trans_id, st_result, st_ex}),
      .d_o({store_valid_o, store_trans_id_o, store_result_o, store_exception_o})
  );

  // determine whether this is a load or store
  always_comb begin : which_op

    ld_valid_i        = 1'b0;
    st_valid_i        = 1'b0;

    cva6_translation_req = 1'b0;
    cva6_mmu_vaddr       = {riscv::VLEN{1'b0}};
    mmu_tinst            = {riscv::XLEN{1'b0}};
    mmu_hs_ld_st_inst    = 1'b0;
    mmu_hlvx_inst        = 1'b0;

    // check the operation to activate the right functional unit accordingly
    unique case (lsu_ctrl.fu)
      // all loads go here
      LOAD: begin
        ld_valid_i           = lsu_ctrl.valid;
        cva6_translation_req = ld_translation_req;
        cva6_mmu_vaddr       = ld_vaddr;
        if (CVA6Cfg.RVH) begin
          mmu_tinst         = ld_tinst;
          mmu_hs_ld_st_inst = ld_hs_ld_st_inst;
          mmu_hlvx_inst     = ld_hlvx_inst;
        end
      end
      // all stores go here
      STORE: begin
        st_valid_i           = lsu_ctrl.valid;
        cva6_translation_req = st_translation_req;
        cva6_mmu_vaddr       = st_vaddr;
        if (CVA6Cfg.RVH) begin
          mmu_tinst         = st_tinst;
          mmu_hs_ld_st_inst = st_hs_ld_st_inst;
          mmu_hlvx_inst     = st_hlvx_inst;
        end
      end
      // not relevant for the LSU
      default: ;
    endcase
  end

  // ------------------------
  // Hypervisor Load/Store
  // ------------------------
  // determine whether this is a hypervisor load or store
  if (CVA6Cfg.RVH) begin
    always_comb begin : hyp_ld_st
      // check the operator to activate the right functional unit accordingly
      hs_ld_st_inst = 1'b0;
      hlvx_inst     = 1'b0;
      case (lsu_ctrl.operation)
        // all loads go here
        HLV_B, HLV_BU, HLV_H, HLV_HU, HLV_W, HSV_B, HSV_H, HSV_W, HLV_WU, HLV_D, HSV_D: begin
          hs_ld_st_inst = 1'b1;
        end
        HLVX_WU, HLVX_HU: begin
          hs_ld_st_inst = 1'b1;
          hlvx_inst     = 1'b1;
        end
        default: ;
      endcase
    end
  end else begin
    assign hs_ld_st_inst = 1'b0;
    assign hlvx_inst     = 1'b0;
  end

  // ---------------
  // Byte Enable
  // ---------------
  // we can generate the byte enable from the virtual address since the last
  // 12 bit are the same anyway
  // and we can always generate the byte enable from the address at hand

  if (riscv::IS_XLEN64) begin : gen_8b_be
    assign be_i = be_gen(vaddr_i[2:0], extract_transfer_size(fu_data_i.operation));
  end else begin : gen_4b_be
    assign be_i = be_gen_32(vaddr_i[1:0], extract_transfer_size(fu_data_i.operation));
  end

  // ------------------------
  // Misaligned Exception
  // ------------------------
  // we can detect a misaligned exception immediately
  // the misaligned exception is passed to the functional unit via the MMU, which in case
  // can augment the exception if other memory related exceptions like a page fault or access errors
  always_comb begin : data_misaligned_detection
    cva6_misaligned_exception = {
      {riscv::XLEN{1'b0}},
      {riscv::XLEN{1'b0}},
      {riscv::GPLEN{1'b0}},
      {riscv::XLEN{1'b0}},
      1'b0,
      1'b0
    };
    data_misaligned = 1'b0;

    if (lsu_ctrl.valid) begin
      case (lsu_ctrl.operation)
        // double word
        LD, SD, FLD, FSD,
                AMO_LRD, AMO_SCD,
                AMO_SWAPD, AMO_ADDD, AMO_ANDD, AMO_ORD,
                AMO_XORD, AMO_MAXD, AMO_MAXDU, AMO_MIND,
                AMO_MINDU, HLV_D, HSV_D: begin
          if (riscv::IS_XLEN64 && lsu_ctrl.vaddr[2:0] != 3'b000) begin
            data_misaligned = 1'b1;
          end
        end
        // word
        LW, LWU, SW, FLW, FSW,
                AMO_LRW, AMO_SCW,
                AMO_SWAPW, AMO_ADDW, AMO_ANDW, AMO_ORW,
                AMO_XORW, AMO_MAXW, AMO_MAXWU, AMO_MINW,
                AMO_MINWU, HLV_W, HLV_WU, HLVX_WU, HSV_W: begin
          if (lsu_ctrl.vaddr[1:0] != 2'b00) begin
            data_misaligned = 1'b1;
          end
        end
        // half word
        LH, LHU, SH, FLH, FSH, HLV_H, HLV_HU, HLVX_HU, HSV_H: begin
          if (lsu_ctrl.vaddr[0] != 1'b0) begin
            data_misaligned = 1'b1;
          end
        end
        // byte -> is always aligned
        default: ;
      endcase
    end

    if (data_misaligned) begin

      if (lsu_ctrl.fu == LOAD) begin
        cva6_misaligned_exception.cause = riscv::LD_ADDR_MISALIGNED;
        cva6_misaligned_exception.valid = 1'b1;
        if (CVA6Cfg.TvalEn)
          cva6_misaligned_exception.tval = {{riscv::XLEN - riscv::VLEN{1'b0}}, lsu_ctrl.vaddr};
        cva6_misaligned_exception.tval2 = '0;
        cva6_misaligned_exception.tinst = lsu_ctrl.tinst;
        cva6_misaligned_exception.gva   = ld_st_v_i;

      end else if (lsu_ctrl.fu == STORE) begin
        cva6_misaligned_exception.cause = riscv::ST_ADDR_MISALIGNED;
        cva6_misaligned_exception.valid = 1'b1;
        if (CVA6Cfg.TvalEn)
          cva6_misaligned_exception.tval = {{riscv::XLEN - riscv::VLEN{1'b0}}, lsu_ctrl.vaddr};
        cva6_misaligned_exception.tval2 = '0;
        cva6_misaligned_exception.tinst = lsu_ctrl.tinst;
        cva6_misaligned_exception.gva   = ld_st_v_i;
      end
    end

    if (ariane_pkg::MMU_PRESENT && en_ld_st_translation_i && lsu_ctrl.overflow) begin

      if (lsu_ctrl.fu == LOAD) begin
        cva6_misaligned_exception.cause = riscv::LD_ACCESS_FAULT;
        cva6_misaligned_exception.valid = 1'b1;
        if (CVA6Cfg.TvalEn)
          cva6_misaligned_exception.tval = {{riscv::XLEN - riscv::VLEN{1'b0}}, lsu_ctrl.vaddr};
        cva6_misaligned_exception.tval2 = '0;
        cva6_misaligned_exception.tinst = lsu_ctrl.tinst;
        cva6_misaligned_exception.gva   = ld_st_v_i;

      end else if (lsu_ctrl.fu == STORE) begin
        cva6_misaligned_exception.cause = riscv::ST_ACCESS_FAULT;
        cva6_misaligned_exception.valid = 1'b1;
        if (CVA6Cfg.TvalEn)
          cva6_misaligned_exception.tval = {{riscv::XLEN - riscv::VLEN{1'b0}}, lsu_ctrl.vaddr};
        cva6_misaligned_exception.tval2 = '0;
        cva6_misaligned_exception.tinst = lsu_ctrl.tinst;
        cva6_misaligned_exception.gva   = ld_st_v_i;
      end
    end

    if (ariane_pkg::MMU_PRESENT && CVA6Cfg.RVH && en_ld_st_g_translation_i && !en_ld_st_translation_i && lsu_ctrl.g_overflow) begin

      if (lsu_ctrl.fu == LOAD) begin
        cva6_misaligned_exception.cause = riscv::LOAD_GUEST_PAGE_FAULT;
        cva6_misaligned_exception.valid = 1'b1;
        if (CVA6Cfg.TvalEn)
          cva6_misaligned_exception.tval = {{riscv::XLEN - riscv::VLEN{1'b0}}, lsu_ctrl.vaddr};
        cva6_misaligned_exception.tval2 = '0;
        cva6_misaligned_exception.tinst = lsu_ctrl.tinst;
        cva6_misaligned_exception.gva   = ld_st_v_i;
      end else if (lsu_ctrl.fu == STORE) begin
        cva6_misaligned_exception.cause = riscv::STORE_GUEST_PAGE_FAULT;
        cva6_misaligned_exception.valid = 1'b1;
        if (CVA6Cfg.TvalEn)
          cva6_misaligned_exception.tval = {{riscv::XLEN - riscv::VLEN{1'b0}}, lsu_ctrl.vaddr};
        cva6_misaligned_exception.tval2 = '0;
        cva6_misaligned_exception.tinst = lsu_ctrl.tinst;
        cva6_misaligned_exception.gva   = ld_st_v_i;
      end
    end
  end

  // ------------------
  // LSU Control
  // ------------------
  // new data arrives here
  lsu_ctrl_t lsu_req_i;

  assign lsu_req_i = {
    lsu_valid_i,
    vaddr_i,
    tinst_i,
    hs_ld_st_inst,
    hlvx_inst,
    overflow,
    g_overflow,
    fu_data_i.operand_b,
    be_i,
    fu_data_i.fu,
    fu_data_i.operation,
    fu_data_i.trans_id
  };

  lsu_bypass #(
      .CVA6Cfg(CVA6Cfg)
  ) lsu_bypass_i (
      .lsu_req_i      (lsu_req_i),
      .lsu_req_valid_i(lsu_valid_i),
      .pop_ld_i       (pop_ld),
      .pop_st_i       (pop_st),

      .lsu_ctrl_o(lsu_ctrl_byp),
      .ready_o   (lsu_ready_o),
      .*
  );

  assign rvfi_lsu_ctrl_o = lsu_ctrl;

endmodule
