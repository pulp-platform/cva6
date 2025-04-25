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
// D-Cache SPM controller
// --------------
//
// Description: Arbitrates access to data cache memories in SPM mode
// Requests from lower index input ports are prioritized
//

`include "common_cells/registers.svh"

module dspm_ctrl
    import std_cache_pkg::*;
    import ariane_pkg::*;
#(
    parameter type dcache_req_i_t         = logic,
    parameter type dcache_req_o_t         = logic,
    parameter int unsigned NR_PORTS       = 3,
    parameter int unsigned NR_WAYS        = 4,
    parameter int unsigned LINE_WIDTH     = 128,
    parameter int unsigned ADDR_WIDTH     = 64,
    parameter int unsigned MEMORY_WIDTH   = 172,
    parameter int unsigned IDX_WIDTH      = 12,    // Cache index + byte offset
    parameter int unsigned NR_WAIT_STAGES = 1,
    // Derived parameters, DO NOT OVERRIDE
    localparam int unsigned BE_WIDTH      = (MEMORY_WIDTH + 7) / 8
) (
    input  logic                                 clk_i,
    input  logic                                 rst_ni,

    input  logic [NR_WAYS-1:0]                   active_ways_i,

    // Request ports
    input  dcache_req_i_t [NR_PORTS-1:0]         spm_req_ports_i,
    output dcache_req_o_t [NR_PORTS-1:0]         spm_req_ports_o,

    output logic [NR_WAYS-1:0]                   req_o,
    output logic [ADDR_WIDTH-1:0]                addr_o,
    output logic [MEMORY_WIDTH-1:0]              wdata_o,
    output logic                                 we_o,
    output logic [BE_WIDTH-1:0]                  be_o,
    input  logic [NR_WAYS-1:0][MEMORY_WIDTH-1:0] rdata_i
);

    localparam WAY_INDEX_BITS = $clog2(NR_WAYS); 
    logic [WAY_INDEX_BITS-1:0] way_idx, way_idx_d, way_idx_q;

    // One hot encoded
    logic [$clog2(NR_PORTS)-1:0] portsel, portsel_d, portsel_q;

    // SRAM latency counter
    logic [$clog2(NR_WAIT_STAGES)-1:0] wait_stage_d, wait_stage_q;
    
    // Helper variable to assemble the full cache-line from parts
    logic [LINE_WIDTH-1:0] write_line;

    // Word offset within the cacheline
    logic [$clog2(LINE_WIDTH/8)-$clog2(riscv::XLEN/8)-1:0] cl_offset, cl_offset_d, cl_offset_q;

    // Port selection logic
    always_comb begin
        portsel = '{default: 0};

        for(int unsigned i = 0; i < NR_PORTS; i++) begin
            // This data request is only coming from the actual requester
            // if it is a write
            // Otherwise this is coming from the request splitter
            // => Only grant if it is a write
            if(spm_req_ports_i[i].data_req) begin
                portsel = i;
                break;
            end
        end
    end

    // Static assignments
    // This saves which cache way this address targets
    assign way_idx = spm_req_ports_i[portsel].address_tag[0 +: WAY_INDEX_BITS];
    
    // This saves at which offset within a cacheline we are
    assign cl_offset = spm_req_ports_i[portsel].address_index[$clog2(LINE_WIDTH/8)-1:$clog2(riscv::XLEN/8)];

    assign addr_o  = spm_req_ports_i[portsel].address_index;
    // This zeros the tag and other status bits,
    // while writing the payload to the SRAM
    assign wdata_o = {{(MEMORY_WIDTH-LINE_WIDTH){1'b0}}, write_line};
    assign we_o    = spm_req_ports_i[portsel].data_we;

    always_comb begin
        cl_offset_d     = cl_offset_q;
        portsel_d       = portsel_q;
        wait_stage_d    = wait_stage_q;
        way_idx_d       = way_idx_q;

        spm_req_ports_o = '{default: 0};

        write_line = '{default: 0};
        // We assemble the write data unconditionally as the
        // write is controlled by the write enable
        write_line[(cl_offset * riscv::XLEN) +: riscv::XLEN] = spm_req_ports_i[portsel].data_wdata;

        req_o   = '{default: 0};

        // By default we'll always write the tag (so that it's zeroed)
        be_o    = '{default: 0};
        be_o[BE_WIDTH-1:(LINE_WIDTH/8)] = '{default: 1'b1};
        // Only enable the part of the cacheline that we actually want to update
        be_o[(cl_offset * (riscv::XLEN/8)) +: (riscv::XLEN/8)] = spm_req_ports_i[portsel].data_be;

        // Decrease the wait counter if it's not already 0
        if(wait_stage_q)
            wait_stage_d = wait_stage_q - ($clog2(NR_WAIT_STAGES)+1)'(1);

        // Accept a new request if one is pending and we're not waiting for
        // the SRAM anymore
        if (spm_req_ports_i[portsel].data_req && wait_stage_q == '0) begin
            portsel_d = portsel;
            way_idx_d = way_idx;
            // Record the offset
            cl_offset_d = cl_offset;

            // Reset the counter on every new request
            wait_stage_d = NR_WAIT_STAGES;

            // Are we allowed to use this memory?
            if(active_ways_i[way_idx]) begin
                req_o[way_idx]  = 1'b1;

            // Otherwise just respond with badcable
            end else begin
                spm_req_ports_o[portsel].data_rdata = 64'hCA11AB1E_BADCAB1E;
                spm_req_ports_o[portsel].data_gnt = spm_req_ports_i[portsel].data_we;
                spm_req_ports_o[portsel].data_rvalid = ~spm_req_ports_i[portsel].data_we;

                // We don't need to wait for the memory here, as we did not access any
                wait_stage_d = '0;
            end
        end

        // Later acknowledge => Use the registered version of the select signals
        if(wait_stage_q == 32'b1) begin
            spm_req_ports_o[portsel_q].data_gnt = spm_req_ports_i[portsel_q].data_we;
            spm_req_ports_o[portsel_q].data_rvalid = ~spm_req_ports_i[portsel_q].data_we;
            spm_req_ports_o[portsel_q].data_rdata = rdata_i[way_idx_q][(cl_offset_q * riscv::XLEN) +: riscv::XLEN];

        // Same cycle acknowledge => Use the unregistered version
        end else if(NR_WAIT_STAGES == 0) begin

            spm_req_ports_o[portsel].data_gnt = spm_req_ports_i[portsel].data_we;
            spm_req_ports_o[portsel].data_rvalid = ~spm_req_ports_i[portsel].data_we;
            spm_req_ports_o[portsel].data_rdata = rdata_i[way_idx][(cl_offset * riscv::XLEN) +: riscv::XLEN];
        end
    end

    `FF(cl_offset_q, cl_offset_d, '0, clk_i, rst_ni)
    `FF(wait_stage_q, wait_stage_d, '0, clk_i, rst_ni)
    `FF(way_idx_q, way_idx_d, '0, clk_i, rst_ni)
    `FF(portsel_q, portsel_d, '0, clk_i, rst_ni)

endmodule
