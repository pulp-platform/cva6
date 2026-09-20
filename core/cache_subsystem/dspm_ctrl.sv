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
    parameter  type         dcache_req_i_t = logic,
    parameter  type         dcache_req_o_t = logic,
    parameter  int unsigned NR_PORTS       = 3,
    parameter  int unsigned NR_WAYS        = 4,
    parameter  int unsigned LINE_WIDTH     = 128,
    parameter  int unsigned ADDR_WIDTH     = 64,
    parameter  int unsigned MEMORY_WIDTH   = 172,
    parameter  int unsigned IDX_WIDTH      = 12,                     // Cache index + byte offset
    parameter  int unsigned NR_WAIT_STAGES = 1,
    // Derived parameters, DO NOT OVERRIDE
    localparam int unsigned BE_WIDTH       = (MEMORY_WIDTH + 7) / 8
) (
    input logic clk_i,
    input logic rst_ni,

    input logic [NR_WAYS-1:0] active_ways_i,

    // Request ports
    input  dcache_req_i_t [NR_PORTS-1:0] spm_req_ports_i,
    output dcache_req_o_t [NR_PORTS-1:0] spm_req_ports_o,

    output logic [     NR_WAYS-1:0]                   req_o,
    output logic [  ADDR_WIDTH-1:0]                   addr_o,
    output logic [MEMORY_WIDTH-1:0]                   wdata_o,
    output logic                                      we_o,
    output logic [    BE_WIDTH-1:0]                   be_o,
    input  logic [     NR_WAYS-1:0][MEMORY_WIDTH-1:0] rdata_i
);

  localparam WAY_INDEX_BITS = $clog2(NR_WAYS);
  logic [WAY_INDEX_BITS-1:0] way_idx, way_idx_d, way_idx_q;

  logic [$clog2(NR_PORTS)-1:0] portsel, portsel_d, portsel_q;

  // SRAM latency counter
  logic [$clog2(NR_WAIT_STAGES)-1:0] wait_stage_d, wait_stage_q;

  // Helper variable to assemble the full cache-line from parts
  logic [LINE_WIDTH-1:0] write_line;

  // Word offset within the cacheline
  logic [$clog2(LINE_WIDTH/8)-$clog2(riscv::XLEN/8)-1:0] cl_offset, cl_offset_d, cl_offset_q;

  // ------------------------------------------------------------------
  // Request classification
  // ------------------------------------------------------------------
  // A read is steered here before its physical tag is known: the request
  // splitter forwards it speculatively with tag_valid low, so that the SRAM
  // read can start from the index alone (the row is fully determined by it;
  // only the way comes from the tag). One cycle later the splitter repeats the
  // request with tag_valid high, and the way is selected then. This mirrors
  // what the cache does with its index/tag split, and what ispm_ctrl already
  // does for instruction fetches.
  //
  //   write          : data_req &  data_we                (tag known already)
  //   confirmed read : data_req & !data_we &  tag_valid
  //   speculative rd : data_req & !data_we & !tag_valid
  //
  // Lower port indices win, as before. Writes have priority over speculation,
  // so the store rate is unaffected.
  logic wr_req, cf_req, sp_req;
  logic [$clog2(NR_PORTS)-1:0] wr_port, cf_port, sp_port;

  always_comb begin
    wr_req  = 1'b0;
    cf_req  = 1'b0;
    sp_req  = 1'b0;
    wr_port = '{default: 0};
    cf_port = '{default: 0};
    sp_port = '{default: 0};

    for (int unsigned i = 0; i < NR_PORTS; i++) begin
      if (spm_req_ports_i[i].data_req) begin
        if (spm_req_ports_i[i].data_we) begin
          if (!wr_req) begin
            wr_req  = 1'b1;
            wr_port = i[$clog2(NR_PORTS)-1:0];
          end
        end else if (spm_req_ports_i[i].tag_valid) begin
          if (!cf_req) begin
            cf_req  = 1'b1;
            cf_port = i[$clog2(NR_PORTS)-1:0];
          end
        end else begin
          if (!sp_req) begin
            sp_req  = 1'b1;
            sp_port = i[$clog2(NR_PORTS)-1:0];
          end
        end
      end
    end
  end

  // Outstanding speculative read (valid for exactly one cycle)
  logic spec_valid_d, spec_valid_q;
  logic [$clog2(NR_PORTS)-1:0] spec_port_d, spec_port_q;
  logic [IDX_WIDTH-1:0] spec_addr_d, spec_addr_q;

  // A confirmed read whose speculative access was issued last cycle: its data
  // is on rdata_i right now, so it can be answered without a further SRAM read.
  logic confirm_hit;
  logic do_accept;  // classic accept: a write, or a read we did not speculate on
  logic do_spec;  // issue a speculative read this cycle

  assign confirm_hit = cf_req && spec_valid_q && (spec_port_q == cf_port) &&
                       (spec_addr_q == spm_req_ports_i[cf_port].address_index) &&
                       (wait_stage_q == '0);
  assign do_accept = !confirm_hit && (wr_req || cf_req) && (wait_stage_q == '0);
  assign do_spec = !do_accept && sp_req && (wait_stage_q == '0);

  // The port whose SRAM access is issued this cycle
  assign portsel = do_accept ? (wr_req ? wr_port : cf_port) : (do_spec ? sp_port : '0);

  // Way and offset of the confirmed read, taken from the request presented now
  logic [WAY_INDEX_BITS-1:0] cf_way_idx;
  logic [$clog2(LINE_WIDTH/8)-$clog2(riscv::XLEN/8)-1:0] cf_cl_offset;

  assign cf_way_idx = spm_req_ports_i[cf_port].address_tag[0+:WAY_INDEX_BITS];
  assign cf_cl_offset = spm_req_ports_i[cf_port].address_index[$clog2(
      LINE_WIDTH/8
  )-1:$clog2(
      riscv::XLEN/8
  )];

  // Static assignments
  // This saves which cache way this address targets
  assign way_idx = spm_req_ports_i[portsel].address_tag[0+:WAY_INDEX_BITS];

  // This saves at which offset within a cacheline we are
  assign cl_offset = spm_req_ports_i[portsel].address_index[$clog2(
      LINE_WIDTH/8
  )-1:$clog2(
      riscv::XLEN/8
  )];

  assign addr_o = spm_req_ports_i[portsel].address_index;
  // This zeros the tag and other status bits,
  // while writing the payload to the SRAM
  assign wdata_o = {{(MEMORY_WIDTH - LINE_WIDTH) {1'b0}}, write_line};
  assign we_o = spm_req_ports_i[portsel].data_we;

  always_comb begin
    cl_offset_d                                        = cl_offset_q;
    portsel_d                                          = portsel_q;
    wait_stage_d                                       = wait_stage_q;
    way_idx_d                                          = way_idx_q;

    spm_req_ports_o                                    = '{default: 0};

    write_line                                         = '{default: 0};
    // We assemble the write data unconditionally as the
    // write is controlled by the write enable
    write_line[(cl_offset*riscv::XLEN)+:riscv::XLEN]   = spm_req_ports_i[portsel].data_wdata;

    req_o                                              = '{default: 0};

    // By default we'll always write the tag (so that it's zeroed)
    be_o                                               = '{default: 0};
    be_o[BE_WIDTH-1:(LINE_WIDTH/8)]                    = '{default: 1'b1};
    // Only enable the part of the cacheline that we actually want to update
    be_o[(cl_offset*(riscv::XLEN/8))+:(riscv::XLEN/8)] = spm_req_ports_i[portsel].data_be;

    // Speculation is only valid for the cycle right after it was issued
    spec_valid_d                                       = do_spec;
    spec_port_d                                        = sp_port;
    spec_addr_d                                        = spm_req_ports_i[sp_port].address_index;

    // Decrease the wait counter if it's not already 0
    if (wait_stage_q) wait_stage_d = wait_stage_q - ($clog2(NR_WAIT_STAGES) + 1)'(1);

    // A confirmed read we speculated on last cycle: the data is already on
    // rdata_i, so answer in this very cycle (one cycle earlier than the
    // classic path below).
    if (confirm_hit) begin
      if (active_ways_i[cf_way_idx]) begin
        spm_req_ports_o[cf_port].data_rdata =
            rdata_i[cf_way_idx][(cf_cl_offset*riscv::XLEN)+:riscv::XLEN];
      end else begin
        spm_req_ports_o[cf_port].data_rdata = 64'hCA11AB1E_BADCAB1E;
      end
      spm_req_ports_o[cf_port].data_rvalid = 1'b1;
      spm_req_ports_o[cf_port].data_rid    = spm_req_ports_i[cf_port].data_id;
    end

    // Start a speculative read: the row comes from the index, the way is not
    // known yet, so read every active way and select on the confirm cycle.
    if (do_spec) begin
      req_o = active_ways_i;
    end

    // Accept a new request if one is pending and we're not waiting for
    // the SRAM anymore
    if (do_accept) begin
      portsel_d = portsel;
      way_idx_d = way_idx;
      // Record the offset
      cl_offset_d = cl_offset;

      // Reset the counter on every new request
      wait_stage_d = NR_WAIT_STAGES;

      // Are we allowed to use this memory?
      if (active_ways_i[way_idx]) begin
        req_o[way_idx] = 1'b1;

        // Otherwise just respond with badcable
      end else begin
        spm_req_ports_o[portsel].data_rdata = 64'hCA11AB1E_BADCAB1E;
        spm_req_ports_o[portsel].data_gnt = spm_req_ports_i[portsel].data_we;
        spm_req_ports_o[portsel].data_rvalid = ~spm_req_ports_i[portsel].data_we;
        spm_req_ports_o[portsel].data_rid = spm_req_ports_i[portsel].data_id;

        // We don't need to wait for the memory here, as we did not access any
        wait_stage_d = '0;
      end
    end

    // Later acknowledge => Use the registered version of the select signals
    if (wait_stage_q == 32'b1) begin
      spm_req_ports_o[portsel_q].data_gnt = spm_req_ports_i[portsel_q].data_we;
      spm_req_ports_o[portsel_q].data_rvalid = ~spm_req_ports_i[portsel_q].data_we;
      spm_req_ports_o[portsel_q].data_rdata = rdata_i[way_idx_q][(cl_offset_q * riscv::XLEN) +: riscv::XLEN];
      spm_req_ports_o[portsel_q].data_rid = spm_req_ports_i[portsel_q].data_id;

      // Same cycle acknowledge => Use the unregistered version
    end else if (NR_WAIT_STAGES == 0) begin

      spm_req_ports_o[portsel].data_gnt = spm_req_ports_i[portsel].data_we;
      spm_req_ports_o[portsel].data_rvalid = ~spm_req_ports_i[portsel].data_we;
      spm_req_ports_o[portsel].data_rdata = rdata_i[way_idx][(cl_offset*riscv::XLEN)+:riscv::XLEN];
      spm_req_ports_o[portsel].data_rid = spm_req_ports_i[portsel].data_id;
    end
  end

  `FF(spec_valid_q, spec_valid_d, 1'b0, clk_i, rst_ni)
  `FF(spec_port_q, spec_port_d, '0, clk_i, rst_ni)
  `FF(spec_addr_q, spec_addr_d, '0, clk_i, rst_ni)

  `FF(cl_offset_q, cl_offset_d, '0, clk_i, rst_ni)
  `FF(wait_stage_q, wait_stage_d, '0, clk_i, rst_ni)
  `FF(way_idx_q, way_idx_d, '0, clk_i, rst_ni)
  `FF(portsel_q, portsel_d, '0, clk_i, rst_ni)

endmodule
