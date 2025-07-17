// Copyright 2024 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Christopher Reinwardt <creinwar@student.ethz.ch>
// --------------
// I-Cache SPM controller
// --------------
//
// Description: Arbitrates access to instruction cache memories in SPM mode
//

`include "common_cells/registers.svh"

module ispm_ctrl
  import ariane_pkg::*;
  import wt_cache_pkg::*;
#(
    parameter type icache_dreq_t          = logic,
    parameter type icache_drsp_t          = logic,
    parameter type dcache_req_i_t         = logic,
    parameter type dcache_req_o_t         = logic,
    parameter type paddr_t                = logic,
    parameter int unsigned NR_WAYS        = 4,
    parameter int unsigned LINE_WIDTH     = 128,
    parameter int unsigned ADDR_WIDTH     = 64,
    parameter int unsigned FETCH_WIDTH    = 32,
    parameter int unsigned MEMORY_WIDTH   = 173,
    parameter int unsigned IDX_WIDTH      = 12,    // Cache index + byte offset
    parameter int unsigned NR_WAIT_STAGES = 1,
    // Derived parameters, DO NOT OVERRIDE
    localparam int unsigned BE_WIDTH      = (MEMORY_WIDTH + 7) / 8
) (
    input  logic                                 clk_i,
    input  logic                                 rst_ni,

    input  logic [NR_WAYS-1:0]                   active_ways_i,

    // Read-only cache port
    input  icache_dreq_t                         icache_req_port_i,
    output icache_drsp_t                         icache_req_port_o,

    input  paddr_t                               icache_phys_addr_i,
    input  logic                                 icache_phys_addr_valid_i,

    // Read/Write port from the LSU
    input  dcache_req_i_t                        spm_rw_req_port_i,
    output dcache_req_o_t                        spm_rw_req_port_o,

    output logic [NR_WAYS-1:0]                   req_o,
    output logic [NR_WAYS-1:0][ADDR_WIDTH-1:0]   addr_o,
    output logic [NR_WAYS-1:0][MEMORY_WIDTH-1:0] wdata_o,
    output logic [NR_WAYS-1:0]                   we_o,
    output logic [NR_WAYS-1:0][BE_WIDTH-1:0]     be_o,
    input  logic [NR_WAYS-1:0][MEMORY_WIDTH-1:0] rdata_i
);

    localparam WAY_INDEX_BITS = $clog2(NR_WAYS); 
    localparam UNUSABLE_WIDTH = MEMORY_WIDTH - LINE_WIDTH;
    logic [WAY_INDEX_BITS-1:0] fetch_way_idx;
    logic [WAY_INDEX_BITS-1:0] lsu_way_idx, lsu_way_idx_d, lsu_way_idx_q;
    logic busy;

    logic [$clog2(NR_WAIT_STAGES)-1:0] lsu_wait_stage_d, lsu_wait_stage_q;
    
    logic [LINE_WIDTH-1:0] write_line;

    logic [$clog2(LINE_WIDTH/8)-$clog2(riscv::XLEN/8)-1:0] lsu_cl_offset, lsu_cl_offset_d, lsu_cl_offset_q;
    logic [$clog2(LINE_WIDTH/8)-$clog2(FETCH_WIDTH/8)-1:0] fetch_cl_offset;

    logic [((UNUSABLE_WIDTH+7)/8)-1:0] be_tag;

    logic [ADDR_WIDTH-1:0]                  read_addr_req, read_addr_phys;

    logic [NR_WAYS-1:0]                   fetch_req, lsu_req;
    logic [NR_WAYS-1:0][ADDR_WIDTH-1:0]   fetch_addr, lsu_addr;
    logic [NR_WAYS-1:0][MEMORY_WIDTH-1:0] lsu_wdata;
    logic [NR_WAYS-1:0]                   lsu_we;
    logic [NR_WAYS-1:0][BE_WIDTH-1:0]     lsu_be;

    // Static assignments
    assign fetch_way_idx    = icache_phys_addr_i[IDX_WIDTH +: WAY_INDEX_BITS];
    assign fetch_cl_offset  = icache_phys_addr_i[$clog2(LINE_WIDTH/8)-1:$clog2(FETCH_WIDTH/8)];
    
    assign lsu_way_idx      = spm_rw_req_port_i.address_tag[0 +: WAY_INDEX_BITS];
    assign lsu_cl_offset    = spm_rw_req_port_i.address_index[$clog2(LINE_WIDTH/8)-1:$clog2(riscv::XLEN/8)];

    assign lsu_addr         = {NR_WAYS{spm_rw_req_port_i.address_index[$clog2(LINE_WIDTH/8) +: (IDX_WIDTH - $clog2(LINE_WIDTH/8))]}};
    assign lsu_wdata        = {NR_WAYS{{UNUSABLE_WIDTH{1'b0}}, write_line}};
    assign lsu_we           = {NR_WAYS{spm_rw_req_port_i.data_we}};

    // We always want to write the tag (i.e. zero it)
    assign be_tag = '{default: 1};

    // The address into the cache way is the part of the index
    // that selects the cache line
    // One version from the request (used in the idle state)
    assign read_addr_req  = icache_req_port_i.vaddr[$clog2(LINE_WIDTH/8) +: (IDX_WIDTH - $clog2(LINE_WIDTH/8))];

    // And one taken from the physical address input (as the vaddr in the request
    // can be changed once the request was acknowledged with ready)
    assign read_addr_phys = icache_phys_addr_i[$clog2(LINE_WIDTH/8) +: (IDX_WIDTH - $clog2(LINE_WIDTH/8))];

    typedef enum logic {
        IDLE,
        READ
    } fetch_state_t;

    fetch_state_t fetch_state_d, fetch_state_q;


    // These memory signals are LSU only
    assign wdata_o      = lsu_wdata;
    assign we_o         = lsu_we;
    assign be_o         = lsu_be;

    // The remaining memory interface is muxed according to the busy signal
    always_comb begin
        if(busy) begin
            req_o        = fetch_req;
            addr_o       = fetch_addr;
        end else begin
            req_o        = lsu_req;
            addr_o       = lsu_addr;
        end
    end

    // Instruction fetch port
    always_comb begin
        busy                = 1'b0;
        fetch_addr          = {NR_WAYS{read_addr_req}};
        fetch_req           = '{default: 1'b0};
        fetch_state_d       = fetch_state_q;
        icache_req_port_o   = '{default: 1'b0};
        icache_req_port_o.data  = rdata_i[fetch_way_idx][(fetch_cl_offset * FETCH_WIDTH) +: FETCH_WIDTH];

        unique case(fetch_state_q)
            IDLE: begin
                if(icache_req_port_i.req) begin
                    // This tells the LSU arbiter that the instruction fetch
                    // wants to access the cache way right now
                    busy            = 1'b1;
                    // To save one cycle, we request all memory ways at once
                    // and forward the correct one, depending on the address
                    fetch_req       = '{default: 1'b1};
                    fetch_state_d   = READ;
                end
            end

            READ: begin
                // Once we know the physical address we can proceed
                // Also, if this request is killed, we just output something anyway as it's
                // killed upstream
                if(icache_phys_addr_valid_i || icache_req_port_i.kill_s2) begin
                    // Are we allowed to use this memory?
                    if(active_ways_i[fetch_way_idx]) begin
                        icache_req_port_o.valid = 1'b1; // If we've got a kill_s2, this will be handled upstream
                    // Otherwise just respond with 0x00000000
                    // (as 0xbadcab1e is actually a compressed instruction :/ )
                    end else begin
                        icache_req_port_o.data  = 32'h00000000;
                        icache_req_port_o.valid = 1'b1;
                    end

                    // If we immediately get another request => service it
                    if(icache_req_port_i.req) begin
                        busy            = 1'b1;
                        fetch_req       = '{default: 1'b1};
                        fetch_state_d   = READ;
                    // Otherwise just go back to idle
                    end else begin
                        fetch_state_d   = IDLE;
                    end

                // If the physical address is not valid yet re-read the memory in every
                // cycle
                end else begin
                    busy            = 1'b1;
                    // It is valid to use the invalid physical address here as we're only
                    // interested in the index part, which is provided by the registered
                    // virtual address - TODO: cleanup with it's own vaddr_i input
                    fetch_addr      = {NR_WAYS{read_addr_phys}};
                    fetch_req       = '{default: 1'b1};
                    fetch_state_d   = READ;
                end
            end
        endcase

        // If we get a kill request for the current request we go back to idle
        if(icache_req_port_i.kill_s1) begin
            busy            = 1'b0;
            fetch_req       = '{default: 1'b0};
            fetch_state_d   = IDLE;
        end
    end

    `FF(fetch_state_q, fetch_state_d, IDLE, clk_i, rst_ni)

    // LSU port - largely copied from dspm_ctrl.sv
    always_comb begin
        lsu_cl_offset_d     = lsu_cl_offset_q;
        lsu_wait_stage_d    = lsu_wait_stage_q;
        lsu_way_idx_d       = lsu_way_idx_q;

        spm_rw_req_port_o = '{default: 0};

        write_line = '{default: 0};
        write_line[(lsu_cl_offset * riscv::XLEN) +: riscv::XLEN] = spm_rw_req_port_i.data_wdata;

        lsu_req   = '{default: 0};

        // By default we'll always write the tag (so that it's zeroed)
        lsu_be    = {NR_WAYS{{be_tag, {(LINE_WIDTH/8){1'b0}}}}};

        // Decrease the wait counter if it is not already 0
        if(lsu_wait_stage_q)
            lsu_wait_stage_d = lsu_wait_stage_q - 32'b1;

        if (spm_rw_req_port_i.data_req && lsu_wait_stage_q == '0 && !busy) begin
            lsu_way_idx_d = lsu_way_idx;

            // Reset the counter on every new request
            lsu_wait_stage_d = NR_WAIT_STAGES;

            // Record the cachline offset
            lsu_cl_offset_d = lsu_cl_offset;

            // Are we allowed to use this memory?
            if(active_ways_i[lsu_way_idx]) begin
                lsu_req[lsu_way_idx]        = 1'b1;
                // Only enable the part of the cacheline that we actually want to update
                lsu_be[lsu_way_idx][(lsu_cl_offset * (riscv::XLEN/8)) +: (riscv::XLEN/8)] = spm_rw_req_port_i.data_be;

            // Otherwise just respond with badcable
            end else begin
                spm_rw_req_port_o.data_rdata    = 64'hCA11AB1E_BADCAB1E;
                spm_rw_req_port_o.data_gnt      = spm_rw_req_port_i.data_we;
                spm_rw_req_port_o.data_rvalid   = ~spm_rw_req_port_i.data_we;

                // We don't need to wait for the memory here, as we did not access any
                lsu_wait_stage_d = '0;
            end
        end

        // Later acknowledge => Use the registered version of the select version
        if(lsu_wait_stage_q == 32'b1) begin
            spm_rw_req_port_o.data_gnt      = spm_rw_req_port_i.data_we;
            spm_rw_req_port_o.data_rvalid   = ~spm_rw_req_port_i.data_we;
            spm_rw_req_port_o.data_rdata    = rdata_i[lsu_way_idx_q][(lsu_cl_offset_q * riscv::XLEN) +: riscv::XLEN];

        // Same cycle acknowledge => Use the unregistered version
        end else if(NR_WAIT_STAGES == 0) begin

            spm_rw_req_port_o.data_gnt      = spm_rw_req_port_i.data_we;
            spm_rw_req_port_o.data_rvalid   = ~spm_rw_req_port_i.data_we;
            spm_rw_req_port_o.data_rdata    = rdata_i[lsu_way_idx][(lsu_cl_offset * riscv::XLEN) +: riscv::XLEN];
        end
    end

    `FF(lsu_cl_offset_q, lsu_cl_offset_d, '0, clk_i, rst_ni)
    `FF(lsu_wait_stage_q, lsu_wait_stage_d, '0, clk_i, rst_ni)
    `FF(lsu_way_idx_q, lsu_way_idx_d, '0, clk_i, rst_ni)

endmodule
