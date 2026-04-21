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
// Date: 08.05.2017
// Description: Flush controller


module controller
  import ariane_pkg::*;
#(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty,
    parameter int unsigned HwstackFifoDepth = 8,
    parameter type bp_resolve_t = logic,
    parameter type icache_dreq_t = logic,
    parameter type icache_drsp_t = logic,
    parameter type dcache_req_i_t = logic,
    parameter type dcache_req_o_t = logic
) (
    // Subsystem Clock - SUBSYSTEM
    input logic clk_i,
    // Asynchronous reset active low - SUBSYSTEM
    input logic rst_ni,
    // Virtualization mode - CSR_REGFILE
    input logic v_i,
    // Reset microarchitecture - SUBSYSTEM
    output logic rst_uarch_no,
    // Set PC om PC Gen - FRONTEND
    output logic set_pc_commit_o,
    // Flush the IF stage - FRONTEND
    output logic flush_if_o,
    // Flush un-issued instructions of the scoreboard - FRONTEND
    output logic flush_unissued_instr_o,
    // Flush ID stage - ID_STAGE
    output logic flush_id_o,
    // Flush EX stage - EX_STAGE
    output logic flush_ex_o,
    // Flush branch predictors - FRONTEND
    output logic flush_bp_o,
    // Flush ICache - CACHE
    output logic flush_icache_o,
    // Flush DCache - CACHE
    output logic flush_dcache_o,
    // Acknowledge the whole DCache Flush - CACHE
    input logic flush_dcache_ack_i,
    // Flush TLBs - EX_STAGE
    output logic flush_tlb_o,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    output logic flush_tlb_vvma_o,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    output logic flush_tlb_gvma_o,
    // Reset vector address - SUBSYSTEM
    input logic [CVA6Cfg.VLEN-1:0] boot_addr_i,
    // Reset vector address - FRONTEND
    output logic [CVA6Cfg.VLEN-1:0] rst_addr_o,
    // PC of currently commited instruction - EX_STAGE
    input logic [CVA6Cfg.VLEN-1:0] pc_commit_i,
    // Halt request from CSR (WFI instruction) - CSR_REGFILE
    input logic halt_csr_i,
    // Halt request from accelerator dispatcher - ACC_DISPATCHER
    input logic halt_acc_i,
    // Halt frontend during fence.i to prevent fetching stale instructions
    output logic halt_frontend_o,
    // Halt signal to commit stage - COMMIT_STAGE
    output logic halt_o,
    // Signal that hardware stacking is being performed - COMMIT_STAGE
    output logic hwstack_pushing_o,
    // Hardware stacking counter - COMMIT_STAGE
    output logic [4:0] hwstack_regs_count_o,
    // Cache is busy - CACHE
    input logic cache_busy_i,
    // Let dcache not accept any new requests - CACHE
    output logic stall_cache_o,
    // Do not init cache - CACHE
    output logic cache_init_no,
    // Pad cycles of fence.t end relative to time interrupt - CSR_REGFILE
    input logic [31:0] fence_t_pad_i,
    // Source for time padding (timer_irq / priv mode switch) - CSR_REGFILE
    input logic fence_t_src_sel_i,
    // Largest recorded execution time of fence.t - CSR_REGFILE
    output logic [31:0] fence_t_ceil_o,
    // Time interrupt - CLINT
    input logic time_irq_i,
    // Current privilege level - CSR_REGFILE
    input riscv::priv_lvl_t priv_lvl_i,
    // Return from exception - CSR_REGFILE
    input logic eret_i,
    // We got an exception, flush the pipeline - FRONTEND
    input logic ex_valid_i,
    // Exception is CLIC interrupt - CSR_REGFILE
    input logic clic_irq_i,
    // Exception is CLIC vectored interrupt - CSR_REGFILE
    input logic clic_vec_irq_i,
    // Trap frame base address - CSR
    input logic [CVA6Cfg.VLEN-1:0] trap_frame_base_i,
    // Address of trap vector table entry - CSR
    input logic [CVA6Cfg.VLEN-1:0] trap_vector_base_i,
    // Integer Register File content - ISSUE_STAGE
    input logic [31:0] [CVA6Cfg.XLEN-1:0] int_regs_i,
    // Floating Point Register File content - ISSUE_STAGE
    input logic [31:0] [CVA6Cfg.XLEN-1:0] fp_regs_i,
    // Page offset for address aliasing checks - EX_STAGE
    input logic [11:0] page_offset_i,
    // Page offset matches - EX_STAGE
    output logic page_offset_matches_o,
    // Set PC - FRONTEND
    output logic frontend_set_pc_o,
    // PC to be set - FRONTEND
    output logic [CVA6Cfg.VLEN-1:0] frontend_next_pc_o,
    // Selects the request from the controller in the i-cache data request mux - CACHES
    output logic icache_dreq_sel_o,
    // Handshake between CACHE and CONTROLLER (vectored irq handler address fetch) - CACHES
    output icache_dreq_t icache_dreq_o,
    // Handshake between CACHE and CONTROLLER (vectored irq handler address fetch) - CACHES
    input icache_drsp_t icache_drsp_i,
    // Data cache request - CACHES
    output dcache_req_i_t dcache_req_o,
    // Data cache response - CACHES
    input dcache_req_o_t dcache_rsp_i,
    // set the debug pc from CSR - FRONTEND
    input logic set_debug_pc_i,
    // We got a resolved branch, check if we need to flush the front-end - EX_STAGE
    input bp_resolve_t resolved_branch_i,
    // We got an instruction which altered the CSR, flush the pipeline - CSR_REGFILE
    input logic flush_csr_i,
    // fence.i in - ACC_DISPATCH
    input logic fence_i_i,
    // fence in - ACC_DISPATCH
    input logic fence_i,
    // fence.t in - EX_STAGE
    input logic fence_t_i,
    // We got an instruction to flush the TLBs and pipeline - COMMIT_STAGE
    input logic sfence_vma_i,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    input logic hfence_vvma_i,
    // TO_BE_COMPLETED - TO_BE_COMPLETED
    input logic hfence_gvma_i,
    // Flush request from commit stage - COMMIT_STAGE
    input logic flush_commit_i,
    // Flush request from accelerator - ACC_DISPATCHER
    input logic flush_acc_i
);

  // active fence - high if we are currently flushing the dcache
  logic fence_active_d, fence_active_q;
  // Added fence_i_active state to track fence.i progress
  logic fence_i_active_d, fence_i_active_q;
  logic                    flush_dcache;

  // Pad counter
  logic             [31:0] pad_cnt;
  logic             [ 3:0] drain_cnt;
  logic                    time_irq_q;
  riscv::priv_lvl_t        priv_lvl_q;

  // cache init shift register. Keep 'no cache init' asserted for 3 cycles.
  logic [2:0] cache_init_d, cache_init_q;
  assign cache_init_d[2:1] = cache_init_q[1:0];
  assign cache_init_no     = |cache_init_q;

  // address to fetch from after coming out of (uarch) reset
  logic [CVA6Cfg.VLEN-1:0] rst_addr_d, rst_addr_q;
  assign rst_addr_o = rst_addr_q;

  // fence.t FSM
  typedef enum logic [2:0] {
    IDLE,
    FLUSH_DCACHE,
    DRAIN_REQS,
    PAD,
    RST_UARCH
  } fence_t_state_e;
  fence_t_state_e fence_t_state_d, fence_t_state_q;
  logic [3:0] rst_uarch_cnt_d, rst_uarch_cnt_q;

  // Vectored interrupt control FSM
  typedef enum logic [1:0] {
    VEC_IRQ_IDLE,
    VEC_IRQ_WAIT_GNT,
    VEC_IRQ_WAIT_DATA
  } vec_irq_state_e;

  logic vec_irq_halt_frontend;
  logic vec_irq_icache_req;
  vec_irq_state_e vec_irq_state_d, vec_irq_state_q;
  logic [CVA6Cfg.VLEN-1:0] vec_irq_address;
  logic [CVA6Cfg.VLEN-1:0] trap_vector_base_d, trap_vector_base_q;

  assign trap_vector_base_d = vec_irq_address;

  assign icache_dreq_o.req     = vec_irq_icache_req;
  assign icache_dreq_o.vaddr   = vec_irq_address;
  assign icache_dreq_o.spec    = '0;
  assign icache_dreq_o.kill_s1 = '0;
  assign icache_dreq_o.kill_s2 = '0;

  assign frontend_next_pc_o = icache_drsp_i.data;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      vec_irq_state_q    <= VEC_IRQ_IDLE;
      trap_vector_base_q <= '0;
    end else begin
      vec_irq_state_q    <= vec_irq_state_d;
      trap_vector_base_q <= trap_vector_base_d;
    end
  end

  // -----------------------------
  // CLIC vectored interrupt logic
  // -----------------------------
  always_comb begin : vec_irq_fsm
    // Default assignments
    icache_dreq_sel_o     = 1'b0;
    frontend_set_pc_o     = 1'b0;
    vec_irq_halt_frontend = 1'b0;
    vec_irq_icache_req    = 1'b0;
    vec_irq_address       = trap_vector_base_i;
    vec_irq_state_d       = vec_irq_state_q;
    unique case (vec_irq_state_q)
      VEC_IRQ_IDLE: begin
        if (ex_valid_i && clic_vec_irq_i) begin
          // Do not enable I-cache requests immediately
          // Leave one cycle to the frontend to kill outstanding
          // i-cache requests.
          vec_irq_halt_frontend = 1'b1;
          vec_irq_state_d = VEC_IRQ_WAIT_GNT;
        end
      end
      VEC_IRQ_WAIT_GNT: begin
        icache_dreq_sel_o     = 1'b1;
        vec_irq_icache_req    = 1'b1;
        vec_irq_halt_frontend = 1'b1;
        vec_irq_address       = trap_vector_base_q;
        if (icache_drsp_i.ready) begin
          vec_irq_state_d = VEC_IRQ_WAIT_DATA;
        end
      end
      VEC_IRQ_WAIT_DATA: begin
        icache_dreq_sel_o     = 1'b1;
        vec_irq_halt_frontend = 1'b1;
        if (icache_drsp_i.valid) begin
          frontend_set_pc_o = 1'b1;
          vec_irq_state_d   = VEC_IRQ_IDLE;
        end
      end
      default: begin
        vec_irq_state_d = VEC_IRQ_IDLE;
      end
    endcase
  end

  // Hardware stacking FSM
  typedef enum logic {
    HWSTACK_FILL_IDLE,
    HWSTACK_FILL_PUSH
  } hwstack_fifo_fill_state_e;

  typedef enum logic [1:0] {
    HWSTACK_DRAIN_IDLE,
    HWSTACK_DRAIN_SEND_REQ,
    HWSTACK_DRAIN_WAIT_GNT
  } hwstack_fifo_drain_state_e;

  // Hwstack fill FSM signals
  hwstack_fifo_fill_state_e hwstack_fill_state_d, hwstack_fill_state_q;
  logic               [4:0] hwstack_regs_count_d, hwstack_regs_count_q;

  // Hwstack drain FSM signals
  hwstack_fifo_drain_state_e hwstack_drain_state_d,   hwstack_drain_state_q;
  logic   [CVA6Cfg.VLEN-1:0] hwstack_drain_address_d, hwstack_drain_address_q;

  // Hwstack FIFO control signals
  logic [HwstackFifoDepth-1:0] [CVA6Cfg.XLEN-1:0] hwstack_fifo_load_data;
  logic                        [CVA6Cfg.XLEN-1:0] hwstack_fifo_wdata;
  logic                        [CVA6Cfg.XLEN-1:0] hwstack_fifo_rdata;
  logic                                           hwstack_fifo_load;
  logic                                           hwstack_fifo_full;
  logic                                           hwstack_fifo_empty;
  logic                                           hwstack_fifo_push;
  logic                                           hwstack_fifo_pop;

  // Data cache request signals
  logic hwstack_dcache_req_valid;

  assign dcache_req_o.data_req      = hwstack_dcache_req_valid;
  assign dcache_req_o.address_index = hwstack_drain_address_q[CVA6Cfg.DCACHE_INDEX_WIDTH-1:0];
  assign dcache_req_o.address_tag   = hwstack_drain_address_q[CVA6Cfg.DCACHE_TAG_WIDTH+CVA6Cfg.DCACHE_INDEX_WIDTH-1:CVA6Cfg.DCACHE_INDEX_WIDTH];
  assign dcache_req_o.data_wdata    = hwstack_fifo_rdata;
  assign dcache_req_o.data_wuser    = '0;
  assign dcache_req_o.data_we       = 1'b1;
  assign dcache_req_o.data_be       = '1;
  assign dcache_req_o.data_size     = '1;
  assign dcache_req_o.data_id       = '0;
  assign dcache_req_o.tag_valid     = '0;
  assign dcache_req_o.kill_req      = '0;
  assign dcache_req_o.cbo_op        = '0;

  assign hwstack_regs_count_o = (hwstack_fill_state_q != HWSTACK_FILL_IDLE) ? hwstack_regs_count_q : '0;

  assign hwstack_fifo_wdata = int_regs_i[hwstack_regs_count_q];

  generate
    for (genvar i = 0; i < HwstackFifoDepth; i++) begin
      assign hwstack_fifo_load_data[i] = int_regs_i[i];
    end
  endgenerate

  hwstack_fifo #(
    .CVA6Cfg      ( CVA6Cfg               ),
    .Depth        ( HwstackFifoDepth      )
  ) i_hwstack_fifo (
    .clk_i       ( clk_i                  ),
    .rst_ni      ( rst_ni                 ),
    .load_i      ( hwstack_fifo_load      ),
    .load_data_i ( hwstack_fifo_load_data ),
    .data_i      ( hwstack_fifo_wdata     ),
    .push_i      ( hwstack_fifo_push      ),
    .pop_i       ( hwstack_fifo_pop       ),
    .data_o      ( hwstack_fifo_rdata     ),
    .empty_o     ( hwstack_fifo_empty     ),
    .full_o      ( hwstack_fifo_full      )
  );

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      hwstack_fill_state_q    <= HWSTACK_FILL_IDLE;
      hwstack_drain_state_q   <= HWSTACK_DRAIN_IDLE;
      hwstack_regs_count_q    <= '0;
      hwstack_drain_address_q <= '0;
    end else begin
      hwstack_fill_state_q    <= hwstack_fill_state_d;
      hwstack_drain_state_q   <= hwstack_drain_state_d;
      hwstack_regs_count_q    <= hwstack_regs_count_d;
      hwstack_drain_address_q <= hwstack_drain_address_d;
    end
  end

  always_comb begin : hwstack_fifo_fill_logic
    // Default assignments
    hwstack_fill_state_d = hwstack_fill_state_q;
    hwstack_regs_count_d = hwstack_regs_count_q;
    hwstack_fifo_load    = 1'b0;
    hwstack_fifo_push    = 1'b0;
    hwstack_pushing_o    = 1'b0;
    unique case (hwstack_fill_state_q)

      HWSTACK_FILL_IDLE: begin
        hwstack_regs_count_d = HwstackFifoDepth;
        if (ex_valid_i && clic_irq_i) begin
          hwstack_fifo_load = 1'b1;
          hwstack_fill_state_d = HWSTACK_FILL_PUSH;
        end
      end

      HWSTACK_FILL_PUSH: begin
        hwstack_pushing_o = 1'b1;
        if (~hwstack_fifo_full) begin
          hwstack_fifo_push = 1'b1;
          if (hwstack_regs_count_q == 'd31) begin
            hwstack_fill_state_d = HWSTACK_FILL_IDLE;
          end else begin
            hwstack_regs_count_d = hwstack_regs_count_q + 1;
          end
        end
      end

      default: begin
        hwstack_fill_state_d = HWSTACK_FILL_IDLE;
      end

    endcase
  end

  always_comb begin : hwstack_load_offset_check
    page_offset_matches_o = 1'b0;
    if (hwstack_drain_state_q != HWSTACK_DRAIN_IDLE) begin
      if ((page_offset_i >= trap_frame_base_i[11:0]) && (page_offset_i < hwstack_drain_address_q[11:0])) begin
        page_offset_matches_o = 1'b1;
      end
    end
  end

  always_comb begin : hwstack_fifo_drain_logic
    // Default assignments
    hwstack_drain_state_d    = hwstack_drain_state_q;
    hwstack_drain_address_d  = hwstack_drain_address_q;
    hwstack_dcache_req_valid = 1'b0;
    hwstack_fifo_pop         = 1'b0;
    unique case (hwstack_drain_state_q)

      HWSTACK_DRAIN_IDLE: begin
        if (ex_valid_i && clic_irq_i) begin
          hwstack_drain_address_d = trap_frame_base_i;
          hwstack_drain_state_d   = HWSTACK_DRAIN_SEND_REQ;
        end
      end

      HWSTACK_DRAIN_SEND_REQ: begin
        if (hwstack_fifo_empty) begin
          if (~hwstack_pushing_o) begin
            hwstack_drain_state_d = HWSTACK_DRAIN_IDLE;
          end
        end else begin
          hwstack_dcache_req_valid = 1'b1;
          if (dcache_rsp_i.data_gnt) begin
            hwstack_fifo_pop        = 1'b1;
            hwstack_drain_address_d = hwstack_drain_address_q + (CVA6Cfg.XLEN/8);
            hwstack_drain_state_d   = HWSTACK_DRAIN_SEND_REQ;
          end else begin
            hwstack_drain_state_d = HWSTACK_DRAIN_WAIT_GNT;
          end
        end
      end

      HWSTACK_DRAIN_WAIT_GNT: begin
        hwstack_dcache_req_valid = 1'b1;
        if (dcache_rsp_i.data_gnt) begin
          hwstack_fifo_pop        = 1'b1;
          hwstack_drain_address_d = hwstack_drain_address_q + (CVA6Cfg.XLEN/8);
          hwstack_drain_state_d   = HWSTACK_DRAIN_SEND_REQ;
        end
      end

      default: begin
        hwstack_drain_state_d = HWSTACK_DRAIN_IDLE;
      end

    endcase
  end

  // ------------
  // Flush CTRL
  // ------------
  always_comb begin : flush_ctrl
    rst_addr_d             = rst_addr_q;
    fence_active_d         = fence_active_q;
    fence_i_active_d       = fence_i_active_q;
    set_pc_commit_o        = 1'b0;
    flush_if_o             = 1'b0;
    flush_unissued_instr_o = 1'b0;
    flush_id_o             = 1'b0;
    flush_ex_o             = 1'b0;
    flush_dcache           = 1'b0;
    flush_icache_o         = 1'b0;
    flush_tlb_o            = 1'b0;
    flush_tlb_vvma_o       = 1'b0;
    flush_tlb_gvma_o       = 1'b0;
    flush_bp_o             = 1'b0;
    // ------------
    // Mis-predict
    // ------------
    // flush on mispredict
    if (resolved_branch_i.is_mispredict) begin
      // flush only un-issued instructions
      flush_unissued_instr_o = 1'b1;
      // and if stage
      flush_if_o             = 1'b1;
    end

    // ---------------------------------
    // FENCE
    // ---------------------------------
    if (fence_i) begin
      // this can be seen as a CSR instruction with side-effect
      set_pc_commit_o        = 1'b1;
      flush_if_o             = 1'b1;
      flush_unissued_instr_o = 1'b1;
      flush_id_o             = 1'b1;
      flush_ex_o             = 1'b1;
      // this is not needed in the case since we
      // have a write-through cache in this case
      // or we are expecting explicit flush/inval via RVZiCbom
      if (CVA6Cfg.DcacheFlushOnFence) begin
        flush_dcache   = 1'b1;
        fence_active_d = 1'b1;
      end
    end

    // ---------------------------------
    // FENCE.I
    // ---------------------------------
    if (fence_i_i) begin
      set_pc_commit_o        = 1'b1;
      flush_if_o             = 1'b1;
      flush_unissued_instr_o = 1'b1;
      flush_id_o             = 1'b1;
      flush_ex_o             = 1'b1;
      flush_icache_o         = 1'b1;
      // this is not needed in the case since we
      // have a write-through cache in this case
      // When handling fence.i, flush both caches and activate fence_i state
      if (CVA6Cfg.DcacheFlushOnFenceI) begin
        flush_dcache = 1'b1;
        fence_active_d = 1'b1;
        fence_i_active_d = 1'b1;
      end
    end

    // this is not needed in the case since we
    // have a write-through cache in this case
    if (CVA6Cfg.DcacheFlushOnFence || CVA6Cfg.DcacheFlushOnFenceI) begin
      // Wait for the acknowledge here
      // Deassert fence_i state only after DCache flush completes
      if (flush_dcache_ack_i && fence_i_active_q) begin
        fence_i_active_d = 1'b0;
      end
      if (flush_dcache_ack_i && fence_active_q) begin
        fence_active_d = 1'b0;
        // keep the flush dcache signal high as long as we didn't get the acknowledge from the cache
      end else if (fence_active_q) begin
        flush_dcache = 1'b1;
      end
    end
    // ---------------------------------
    // SFENCE.VMA
    // ---------------------------------
    if (CVA6Cfg.RVS && sfence_vma_i) begin
      set_pc_commit_o        = 1'b1;
      flush_if_o             = 1'b1;
      flush_unissued_instr_o = 1'b1;
      flush_id_o             = 1'b1;
      flush_ex_o             = 1'b1;

      if (CVA6Cfg.RVH && v_i) flush_tlb_vvma_o = 1'b1;
      else flush_tlb_o = 1'b1;
    end

    // ---------------------------------
    // HFENCE.VVMA
    // ---------------------------------
    if (CVA6Cfg.RVH && hfence_vvma_i) begin
      set_pc_commit_o        = 1'b1;
      flush_if_o             = 1'b1;
      flush_unissued_instr_o = 1'b1;
      flush_id_o             = 1'b1;
      flush_ex_o             = 1'b1;

      flush_tlb_vvma_o       = 1'b1;
    end

    // ---------------------------------
    // HFENCE.GVMA
    // ---------------------------------
    if (CVA6Cfg.RVH && hfence_gvma_i) begin
      set_pc_commit_o        = 1'b1;
      flush_if_o             = 1'b1;
      flush_unissued_instr_o = 1'b1;
      flush_id_o             = 1'b1;
      flush_ex_o             = 1'b1;

      flush_tlb_gvma_o       = 1'b1;
    end

    // ---------------------------------
    // FENCE.T
    // ---------------------------------
    if (fence_t_i) begin
      flush_icache_o = 1'b1;
      flush_dcache   = 1'b1;
      fence_active_d = 1'b1;

      // Save PC to continue from after coming out of reset
      rst_addr_d     = pc_commit_i + {{CVA6Cfg.VLEN - 3{1'b0}}, 3'b100};
    end

    // ---------------------------------
    // CSR side effects and accelerate port
    // ---------------------------------
    // Set PC to commit stage and flush pipeline
    if (flush_csr_i || flush_acc_i) begin
      set_pc_commit_o        = 1'b1;
      flush_if_o             = 1'b1;
      flush_unissued_instr_o = 1'b1;
      flush_id_o             = 1'b1;
      flush_ex_o             = 1'b1;
    end else if (CVA6Cfg.RVA && flush_commit_i) begin
      set_pc_commit_o        = 1'b1;
      flush_if_o             = 1'b1;
      flush_unissued_instr_o = 1'b1;
      flush_id_o             = 1'b1;
      flush_ex_o             = 1'b1;
    end

    // ---------------------------------
    // 1. Exception
    // 2. Return from exception
    // ---------------------------------
    if (ex_valid_i || eret_i || (CVA6Cfg.DebugEn && set_debug_pc_i)) begin
      // don't flush pcgen as we want to take the exception: Flush PCGen is not a flush signal
      // for the PC Gen stage but instead tells it to take the PC we gave it
      set_pc_commit_o        = 1'b0;
      flush_if_o             = 1'b1;
      flush_unissued_instr_o = 1'b1;
      flush_id_o             = 1'b1;
      flush_ex_o             = 1'b1;
      // this potentially reduces performance, but is needed
      // to suppress speculative fetches to virtual memory from
      // machine mode. TODO: remove when PMA checkers have been
      // added to the system
      flush_bp_o             = 1'b1;
    end
  end

  // ----------------------
  // Halt Logic
  // ----------------------
  always_comb begin
    // halt the core if the fence is active
    halt_o = halt_csr_i || halt_acc_i || ((CVA6Cfg.DcacheFlushOnFence || CVA6Cfg.DcacheFlushOnFenceI) && fence_active_q) || (fence_t_state_q != IDLE);
    // Halt frontend during fence.i to synchronize ICache/DCache flushes
    halt_frontend_o = fence_i_active_q | vec_irq_halt_frontend;
  end

  // ----------------------
  // Microreset Logic
  // ----------------------
  always_comb begin : fence_t_fsm
    // Default assignments
    fence_t_state_d = fence_t_state_q;
    rst_uarch_cnt_d = rst_uarch_cnt_q;
    rst_uarch_no    = 1'b1;
    fence_t_ceil_o  = '0;
    cache_init_d[0] = 1'b0;

    unique case (fence_t_state_q)
      // Idle
      IDLE: begin
        if (fence_t_i) fence_t_state_d = FLUSH_DCACHE;
      end

      // Wait for dcache to acknowledge flush
      FLUSH_DCACHE: begin
        if (flush_dcache_ack_i) fence_t_state_d = DRAIN_REQS;
      end

      // Wait for all pending (external) transactions to complete,
      // s.t. we do not violate any handshake protocols.
      DRAIN_REQS: begin
        // The cache controls our only handshaked interface.
        // Wait until it was idle for 16 cycles.
        if (drain_cnt == 4'hf) begin
          fence_t_state_d = PAD;
          fence_t_ceil_o  = (pad_cnt == '0) ? '0 : fence_t_pad_i - pad_cnt;
        end
      end

      // Wait for the padding to complete.
      PAD: begin
        if (pad_cnt == '0) fence_t_state_d = RST_UARCH;
      end

      // Reset microarchitecture
      RST_UARCH: begin
        rst_uarch_no    = 1'b0;
        cache_init_d[0] = 1'b1;

        // Return to IDLE after 16 cycles
        if (rst_uarch_cnt_q == 4'hf) begin
          rst_uarch_cnt_d = 4'b0;
          fence_t_state_d = IDLE;
        end else begin
          rst_uarch_cnt_d = rst_uarch_cnt_q + 1;
        end
      end

      // We should never reach this state
      default: begin
        fence_t_state_d = IDLE;
      end
    endcase
  end


  // Let the caches not accept any new memory requests during fence_t
  assign stall_cache_o = (fence_t_state_q != IDLE);

  // Start padding either from CLINT timer interrupt [0] or exetinig leaving U-mode [1]
  logic load_pad_cnt;
  assign load_pad_cnt = fence_t_src_sel_i ? ((priv_lvl_q == riscv::PRIV_LVL_U) && (priv_lvl_i != riscv::PRIV_LVL_U))
                                            : (time_irq_i & ~time_irq_q);

  counter #(
      .WIDTH          (4),
      .STICKY_OVERFLOW(0)
  ) i_drain_cnt (
      .clk_i,
      .rst_ni,
      .clear_i   (cache_busy_i),       // Start counting from 0 when cache is busy
      .en_i      (drain_cnt != 4'hf),  // Stop counting when saturated
      .load_i    (1'b0),
      .down_i    (1'b0),
      .d_i       ('0),
      .q_o       (drain_cnt),
      .overflow_o()
  );

  counter #(
      .WIDTH          (32),
      .STICKY_OVERFLOW(0)
  ) i_pad_cnt (
      .clk_i,
      .rst_ni,
      .clear_i   (1'b0),
      .en_i      (|pad_cnt),       // Count until 0
      .load_i    (load_pad_cnt),   // Start counting on positive edge of time irq
      .down_i    (1'b1),           // Always count down
      .d_i       (fence_t_pad_i),  // Start counting from FENCE_T_CSR value
      .q_o       (pad_cnt),
      .overflow_o()
  );

  // ----------------------
  // Registers
  // ----------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (~rst_ni) begin
      fence_t_state_q  <= IDLE;
      rst_uarch_cnt_q  <= 4'b0;
      fence_active_q   <= 1'b0;
      fence_i_active_q <= 1'b0;
      flush_dcache_o   <= 1'b0;
      rst_addr_q       <= boot_addr_i;
      time_irq_q       <= 1'b0;
      priv_lvl_q       <= riscv::PRIV_LVL_M;
      cache_init_q     <= '0;
    end else begin
      fence_t_state_q  <= fence_t_state_d;
      fence_active_q   <= fence_active_d;
      fence_i_active_q <= fence_i_active_d;
      rst_uarch_cnt_q  <= rst_uarch_cnt_d;
      // register on the flush signal, this signal might be critical
      flush_dcache_o   <= flush_dcache;
      rst_addr_q       <= rst_addr_d;
      time_irq_q       <= time_irq_i;
      priv_lvl_q       <= priv_lvl_i;
      cache_init_q     <= cache_init_d;
    end
  end
endmodule
