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
// Date: 06.10.2017
// Description: Performance counters


module perf_counters import ariane_pkg::*; #(
  parameter int unsigned                NumPorts      = 3    // number of miss ports
) (
  input  logic                                    clk_i,
  input  logic                                    rst_ni,
  input  logic                                    debug_mode_i, // debug mode
  // SRAM like interface
  input  logic [11:0]                             addr_i,   // read/write address (up to 6 counters possible)
  input  logic                                    we_i,     // write enable
  input  riscv::xlen_t                            data_i,   // data to write
  output riscv::xlen_t                            data_o,   // data to read
  // from commit stage
  input  scoreboard_entry_t [NR_COMMIT_PORTS-1:0] commit_instr_i,     // the instruction we want to commit
  input  logic [NR_COMMIT_PORTS-1:0]              commit_ack_i,       // acknowledge that we are indeed committing
  // from L1 caches
  input  logic                                    l1_icache_miss_i,
  input  logic                                    l1_dcache_miss_i,
  input  logic                                    l1_dcache_hit_i,
  input  logic                                    l1_dcache_flushing_i,
  input  logic                                    l1_dcache_write_hit_unique_i,
  input  logic                                    l1_dcache_write_hit_shared_i,
  input  logic                                    l1_dcache_write_miss_i,
  input  logic                                    l1_dcache_clean_invalid_hit_i,
  input  logic                                    l1_dcache_clean_invalid_miss_i,
  input  logic                                    amo_i,
  // from MMU
  input  logic                                    itlb_miss_i,
  input  logic                                    dtlb_miss_i,
  // from issue stage
  input  logic                                    sb_full_i,
  // from frontend
  input  logic                                    if_empty_i,
  // from PC Gen
  input  exception_t                              ex_i,
  input  logic                                    eret_i,
  input  bp_resolve_t                             resolved_branch_i,
  // for newly added events
  input  exception_t                              branch_exceptions_i,  //Branch exceptions->execute unit-> branch_exception_o
  input  icache_dreq_i_t                          l1_icache_access_i,
  input  dcache_req_i_t[2:0]                      l1_dcache_access_i,
  input  logic [NumPorts-1:0][DCACHE_SET_ASSOC-1:0]miss_vld_bits_i,  //For Cache eviction (3ports-LOAD,STORE,PTW)
  input  logic                                    i_tlb_flush_i,
  input  logic                                    stall_issue_i  //stall-read operands
);

  logic [63:0] generic_counter_d[11:1];
  logic [63:0] generic_counter_q[11:1];

  logic l1_dcache_flushing_q;
  logic amo_q;

  //internal signal to keep track of exception
  logic read_access_exception,update_access_exception;

  logic events[11:1];
  //internal signal for  MUX select line input
  logic [4:0] mhpmevent_d[6:1];
  logic [4:0] mhpmevent_q[6:1];

  //Multiplexer
  /*
   always_comb begin : Mux
        events[6:1]='{default:0};

      for(int unsigned i = 1; i <= 6; i++) begin
        case(mhpmevent_q[i])
           5'b00000 : events[i] = 0;
           5'b00001 : events[i] = l1_icache_miss_i;//L1 I-Cache misses
           5'b00010 : events[i] = l1_dcache_miss_i;//L1 D-Cache misses
           5'b00011 : events[i] = itlb_miss_i;//ITLB misses
           5'b00100 : events[i] = dtlb_miss_i;//DTLB misses
           5'b00101 : for (int unsigned j = 0; j < NR_COMMIT_PORTS; j++) if (commit_ack_i[j]) events[i] = commit_instr_i[j].fu == LOAD;//Load accesses
           5'b00110 : for (int unsigned j = 0; j < NR_COMMIT_PORTS; j++) if (commit_ack_i[j]) events[i] = commit_instr_i[j].fu == STORE;//Store accesses
           5'b00111 : events[i] = ex_i.valid;//Exceptions
           5'b01000 : events[i] = eret_i;//Exception handler returns
           5'b01001 : for (int unsigned j = 0; j < NR_COMMIT_PORTS; j++) if (commit_ack_i[j]) events[i] = commit_instr_i[j].fu == CTRL_FLOW;//Branch instructions
           5'b01010 : events[i] = resolved_branch_i.valid && resolved_branch_i.is_mispredict;//Branch mispredicts
           5'b01011 : events[i] = branch_exceptions_i.valid;//Branch exceptions
                   // The standard software calling convention uses register x1 to hold the return address on a call
                   // the unconditional jump is decoded as ADD op
           5'b01100 : for (int unsigned j = 0; j < NR_COMMIT_PORTS; j++) if (commit_ack_i[j]) events[i] = commit_instr_i[j].fu == CTRL_FLOW && (commit_instr_i[j].op == ADD || commit_instr_i[j].op == JALR) && (commit_instr_i[j].rd == 'd1 || commit_instr_i[j].rd == 'd5);//Call
           5'b01101 : for (int unsigned j = 0; j < NR_COMMIT_PORTS; j++) if (commit_ack_i[j]) events[i] = commit_instr_i[j].op == JALR && commit_instr_i[j].rd == 'd0;//Return
           5'b01110 : events[i] = sb_full_i;//MSB Full
           5'b01111 : events[i] = if_empty_i;//Instruction fetch Empty
           5'b10000 : events[i] = l1_icache_access_i.req;//L1 I-Cache accesses
           5'b10001 : events[i] = l1_dcache_access_i[0].data_req || l1_dcache_access_i[1].data_req || l1_dcache_access_i[2].data_req;//L1 D-Cache accesses
           5'b10010 : events[i] = (l1_dcache_miss_i && miss_vld_bits_i[0] == 8'hFF) || (l1_dcache_miss_i && miss_vld_bits_i[1] == 8'hFF) || (l1_dcache_miss_i && miss_vld_bits_i[2] == 8'hFF);//eviction
           5'b10011 : events[i] = i_tlb_flush_i;//I-TLB flush
           5'b10100 : for (int unsigned j = 0; j < NR_COMMIT_PORTS; j++) if (commit_ack_i[j]) events[i] = commit_instr_i[j].fu == ALU || commit_instr_i[j].fu == MULT;//Integer instructions
           5'b10101 : for (int unsigned j = 0; j < NR_COMMIT_PORTS; j++) if (commit_ack_i[j]) events[i] = commit_instr_i[j].fu == FPU || commit_instr_i[j].fu == FPU_VEC;//Floating Point Instructions
           5'b10110 : events[i] = stall_issue_i;//Pipeline bubbles
           5'b10111 : events[i] = l1_dcache_hit_i;
           5'b11000 : events[i] = l1_dcache_flushing_i;
           default:   events[i] = 0;
         endcase
       end
    end
    */

    assign events[1] = l1_dcache_miss_i;
    assign events[2] = l1_dcache_hit_i;
    assign events[3] = l1_dcache_flushing_i;
    assign events[4] = l1_dcache_flushing_q & (!l1_dcache_flushing_i);
    assign events[5] = amo_q & (!amo_i);
    assign events[6] = 1'b1;
    assign events[7] = l1_dcache_write_hit_unique_i;
    assign events[8] = l1_dcache_write_hit_shared_i;
    assign events[9] = l1_dcache_write_miss_i;
    assign events[10] = l1_dcache_clean_invalid_hit_i;
    assign events[11] = l1_dcache_clean_invalid_miss_i;

    always_comb begin : generic_counter
        generic_counter_d = generic_counter_q;
        data_o = 'b0;
        mhpmevent_d = mhpmevent_q;
        read_access_exception =  1'b0;
        update_access_exception =  1'b0;

      for(int unsigned i = 1; i <= 11; i++) begin
         if ((debug_mode_i == 1'b0) && (we_i == 1'b0)) begin
             if (events[i] == 1'b1) begin
                generic_counter_d[i] = generic_counter_q[i] + 1'b1; end
         end else begin
                generic_counter_d[i] = 'b0;
         end
        end

     //Read
         unique case (addr_i)
            riscv::CSR_MHPM_COUNTER_3,
            riscv::CSR_MHPM_COUNTER_4,
            riscv::CSR_MHPM_COUNTER_5,
            riscv::CSR_MHPM_COUNTER_6,
            riscv::CSR_MHPM_COUNTER_7,
            riscv::CSR_MHPM_COUNTER_8  :begin if (riscv::XLEN == 32) data_o = generic_counter_q[addr_i-riscv::CSR_MHPM_COUNTER_3 + 1][31:0]; else data_o = generic_counter_q[addr_i-riscv::CSR_MHPM_COUNTER_3 + 1];end
            riscv::CSR_MHPM_COUNTER_3H,
            riscv::CSR_MHPM_COUNTER_4H,
            riscv::CSR_MHPM_COUNTER_5H,
            riscv::CSR_MHPM_COUNTER_6H,
            riscv::CSR_MHPM_COUNTER_7H,
            riscv::CSR_MHPM_COUNTER_8H :begin if (riscv::XLEN == 32) data_o = generic_counter_q[addr_i-riscv::CSR_MHPM_COUNTER_3H + 1][63:32]; else read_access_exception = 1'b1;end
            riscv::CSR_MHPM_EVENT_3,
            riscv::CSR_MHPM_EVENT_4,
            riscv::CSR_MHPM_EVENT_5,
            riscv::CSR_MHPM_EVENT_6,
            riscv::CSR_MHPM_EVENT_7,
            riscv::CSR_MHPM_EVENT_8   : data_o = mhpmevent_q[addr_i-riscv::CSR_MHPM_EVENT_3 + 1] ;
            riscv::CSR_L1_ICACHE_MISS,
            riscv::CSR_L1_DCACHE_MISS,
            riscv::CSR_ITLB_MISS,
            riscv::CSR_DTLB_MISS,
            riscv::CSR_LOAD,
            riscv::CSR_STORE,
            riscv::CSR_EXCEPTION,
            riscv::CSR_EXCEPTION_RET,
            riscv::CSR_BRANCH_JUMP,
            riscv::CSR_CALL,
            riscv::CSR_RET,
            riscv::CSR_MIS_PREDICT,
            riscv::CSR_SB_FULL,
            riscv::CSR_IF_EMPTY,
            riscv::CSR_HPM_COUNTER_17,
            riscv::CSR_HPM_COUNTER_18,
            riscv::CSR_HPM_COUNTER_19,
            riscv::CSR_HPM_COUNTER_20,
            riscv::CSR_HPM_COUNTER_21,
            riscv::CSR_HPM_COUNTER_22,
            riscv::CSR_HPM_COUNTER_23,
            riscv::CSR_HPM_COUNTER_24,
            riscv::CSR_HPM_COUNTER_25,
            riscv::CSR_HPM_COUNTER_26,
            riscv::CSR_HPM_COUNTER_27,
            riscv::CSR_HPM_COUNTER_28,
            riscv::CSR_HPM_COUNTER_29,
            riscv::CSR_HPM_COUNTER_30,
            riscv::CSR_HPM_COUNTER_31 : data_o = generic_counter_q[addr_i-riscv::CSR_L1_ICACHE_MISS + 1];
            default: data_o = 'b0;
        endcase

     //Write
     if(we_i) begin
        unique case(addr_i)
            riscv::CSR_MHPM_COUNTER_3,
            riscv::CSR_MHPM_COUNTER_4,
            riscv::CSR_MHPM_COUNTER_5,
            riscv::CSR_MHPM_COUNTER_6,
            riscv::CSR_MHPM_COUNTER_7,
            riscv::CSR_MHPM_COUNTER_8  :begin if (riscv::XLEN == 32) generic_counter_d[addr_i-riscv::CSR_MHPM_COUNTER_3 + 1][31:0] = data_i; else generic_counter_d[addr_i-riscv::CSR_MHPM_COUNTER_3 + 1] = data_i; end
            riscv::CSR_MHPM_COUNTER_3H,
            riscv::CSR_MHPM_COUNTER_4H,
            riscv::CSR_MHPM_COUNTER_5H,
            riscv::CSR_MHPM_COUNTER_6H,
            riscv::CSR_MHPM_COUNTER_7H,
            riscv::CSR_MHPM_COUNTER_8H :begin if (riscv::XLEN == 32) generic_counter_d[addr_i-riscv::CSR_MHPM_COUNTER_3H + 1][63:32] = data_i; else update_access_exception = 1'b1;end
            riscv::CSR_MHPM_EVENT_3,
            riscv::CSR_MHPM_EVENT_4,
            riscv::CSR_MHPM_EVENT_5,
            riscv::CSR_MHPM_EVENT_6,
            riscv::CSR_MHPM_EVENT_7,
            riscv::CSR_MHPM_EVENT_8   : mhpmevent_d[addr_i-riscv::CSR_MHPM_EVENT_3 + 1] = data_i;
            default: update_access_exception =  1'b1;
        endcase
      end
    end

//Registers
  always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            generic_counter_q <= '{default:0};
            mhpmevent_q       <= '{default:0};
            l1_dcache_flushing_q <= 1'b0;
            amo_q <= 1'b0;
        end else begin
            generic_counter_q <= generic_counter_d;
            mhpmevent_q       <= mhpmevent_d;
            l1_dcache_flushing_q <= l1_dcache_flushing_i;
            amo_q <= amo_i;
       end
   end

   xlnx_ila i_ila (
    clk_i, 
    generic_counter_q[1][63:32], 
    generic_counter_q[1][31:0], 
    generic_counter_q[2][63:32], 
    generic_counter_q[2][31:0], 
    generic_counter_q[3][63:32], 
    generic_counter_q[3][31:0], 
    generic_counter_q[4][63:32], 
    generic_counter_q[4][31:0], 
    {debug_mode_i, we_i, l1_dcache_miss_i, l1_dcache_hit_i, l1_dcache_flushing_i},
    '0,
    '0,
    '0,
    '0
  );

endmodule
