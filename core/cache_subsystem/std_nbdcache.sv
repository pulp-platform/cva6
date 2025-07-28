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
// Date: 13.10.2017
// Description: Nonblocking private L1 dcache

`include "common_cells/registers.svh"

module std_nbdcache
  import std_cache_pkg::*;
  import ariane_pkg::*;
#(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty,
    parameter type dcache_req_i_t = logic,
    parameter type dcache_req_o_t = logic,
    parameter int unsigned NumPorts = 4,
    parameter type axi_req_t = logic,
    parameter type axi_rsp_t = logic,
    parameter type impl_in_t = logic
) (
    input logic clk_i,  // Clock
    input logic rst_ni,  // Asynchronous reset active low
    // Cache management
    input logic enable_i,  // from CSR
    input logic flush_i,  // high until acknowledged
    output logic flush_ack_o,  // send a single cycle acknowledge signal when the cache is flushed
    output logic miss_o,  // we missed on a LD/ST
    output logic busy_o,
    input logic stall_i,  // stall new memory requests
    input logic init_ni,
    input logic [CVA6Cfg.DCACHE_SET_ASSOC-1:0] dcache_spm_ways_i,
    input impl_in_t [CVA6Cfg.DCACHE_SET_ASSOC:0] sram_impl_i,
    // AMOs
    input amo_req_t amo_req_i,
    output amo_resp_t amo_resp_o,
    // Request ports
    input dcache_req_i_t [NumPorts-1:0] req_ports_i,  // request ports
    output dcache_req_o_t [NumPorts-1:0] req_ports_o,  // request ports
    // Request ports to instruction SPM (within icache)
    input  dcache_req_o_t ispm_req_i,
    output dcache_req_i_t ispm_req_o,
    // Cache AXI refill port
    output axi_req_t axi_data_o,
    input axi_rsp_t axi_data_i,
    output axi_req_t axi_bypass_o,
    input axi_rsp_t axi_bypass_i
);

  import std_cache_pkg::*;

  localparam DCACHE_DIRTY_WIDTH = CVA6Cfg.DCACHE_SET_ASSOC * 2;
  localparam DCACHE_MEMORY_WIDTH = CVA6Cfg.DCACHE_TAG_WIDTH + CVA6Cfg.DCACHE_LINE_WIDTH;
  localparam DCACHE_BE_WIDTH = (DCACHE_MEMORY_WIDTH + 7) / 8;

  localparam type cache_line_t = struct packed {
    logic [CVA6Cfg.DCACHE_TAG_WIDTH-1:0]          tag;    // tag array
    logic [CVA6Cfg.DCACHE_LINE_WIDTH-1:0]         data;   // data array
    logic                                         valid;  // state array
    logic [(CVA6Cfg.DCACHE_LINE_WIDTH-1+7)/8-1:0] dirty;  // state array
  };
  typedef struct packed {
    logic [CVA6Cfg.DCACHE_LINE_WIDTH/8-1:0] dirty;
    logic                                   valid;
  } vldrty_t;
  localparam type cl_be_t = struct packed {
    logic [(CVA6Cfg.DCACHE_TAG_WIDTH+7)/8-1:0] tag;  // byte enable into tag array
    logic [(CVA6Cfg.DCACHE_LINE_WIDTH+7)/8-1:0] data;  // byte enable into data array
    vldrty_t [CVA6Cfg.DCACHE_SET_ASSOC-1:0]     vldrty; // bit enable into state array (valid for a pair of dirty/valid bits)
  };

  // -------------------------------
  // Controller <-> Arbiter
  // -------------------------------
  // 1. Miss handler
  // 2. PTW
  // 3. Load Unit
  // 4. Accelerator
  // 5. Store unit
  logic        [                    NumPorts:0][  CVA6Cfg.DCACHE_SET_ASSOC-1:0] req;
  logic        [                    NumPorts:0][CVA6Cfg.DCACHE_INDEX_WIDTH-1:0] addr;
  logic        [                    NumPorts:0]                                 gnt;
  cache_line_t [  CVA6Cfg.DCACHE_SET_ASSOC-1:0]                                 rdata;
  logic        [                    NumPorts:0][  CVA6Cfg.DCACHE_TAG_WIDTH-1:0] tag;

  cache_line_t [                    NumPorts:0]                                 wdata;
  logic        [                    NumPorts:0]                                 we;
  cl_be_t      [                    NumPorts:0]                                 be;
  logic        [  CVA6Cfg.DCACHE_SET_ASSOC-1:0]                                 hit_way;
  // -------------------------------
  // Controller <-> Miss unit
  // -------------------------------
  logic        [                  NumPorts-1:0]                                 busy;
  logic        [                  NumPorts-1:0][                          55:0] mshr_addr;
  logic        [                  NumPorts-1:0]                                 mshr_addr_matches;
  logic        [                  NumPorts-1:0]                                 mshr_index_matches;
  logic        [                          63:0]                                 critical_word;
  logic                                                                         critical_word_valid;

  logic        [                  NumPorts-1:0][         $bits(miss_req_t)-1:0] miss_req;
  logic        [                  NumPorts-1:0]                                 miss_gnt;
  logic        [                  NumPorts-1:0]                                 active_serving;

  logic        [                  NumPorts-1:0]                                 bypass_gnt;
  logic        [                  NumPorts-1:0]                                 bypass_valid;
  logic        [                  NumPorts-1:0][                          63:0] bypass_data;
  // -------------------------------
  // Arbiter <-> Datram,
  // -------------------------------
  logic        [  CVA6Cfg.DCACHE_SET_ASSOC-1:0]                                 req_cache;
  logic        [CVA6Cfg.DCACHE_INDEX_WIDTH-1:0]                                 addr_cache;
  logic                                                                         we_cache;
  cache_line_t                                                                  wdata_cache;
  cache_line_t [  CVA6Cfg.DCACHE_SET_ASSOC-1:0]                                 rdata_cache;
  cl_be_t                                                                       be_cache;
  vldrty_t     [  CVA6Cfg.DCACHE_SET_ASSOC-1:0]                                 be_valid_dirty_ram;

  logic        [  CVA6Cfg.DCACHE_SET_ASSOC-1:0]                                 req_spm;
  logic        [CVA6Cfg.DCACHE_INDEX_WIDTH-1:0]                                 addr_spm;
  logic                                                                         we_spm;
  logic        [       DCACHE_MEMORY_WIDTH-1:0]                                 wdata_spm;
  logic        [  CVA6Cfg.DCACHE_SET_ASSOC-1:0][       DCACHE_MEMORY_WIDTH-1:0] rdata_spm;
  logic        [           DCACHE_BE_WIDTH-1:0]                                 be_spm;

  logic        [  CVA6Cfg.DCACHE_SET_ASSOC-1:0]                                 req_ram;
  logic        [  CVA6Cfg.DCACHE_SET_ASSOC-1:0][CVA6Cfg.DCACHE_INDEX_WIDTH-1:0] addr_ram;
  logic        [  CVA6Cfg.DCACHE_SET_ASSOC-1:0]                                 we_ram;
  logic        [  CVA6Cfg.DCACHE_SET_ASSOC-1:0][       DCACHE_MEMORY_WIDTH-1:0] wdata_ram;
  logic        [  CVA6Cfg.DCACHE_SET_ASSOC-1:0][       DCACHE_MEMORY_WIDTH-1:0] rdata_ram;
  logic        [  CVA6Cfg.DCACHE_SET_ASSOC-1:0][           DCACHE_BE_WIDTH-1:0] be_ram;

  // Busy signals
  logic                                                                         miss_handler_busy;
  assign busy_o = |busy | miss_handler_busy;

  // States of the Cache vs. SPM demux
  typedef enum logic [2:0] {
    IDLE = 0,
    WAIT_TAG,
    WAIT_CACHE,
    WAIT_SPM,
    WRITE
  } addr_decode_state_t;

  dcache_req_i_t [NumPorts-1:0] cache_ports_in;
  dcache_req_o_t [NumPorts-1:0] cache_ports_out;

  dcache_req_i_t [NumPorts-1:0] dspm_ports_in;
  dcache_req_o_t [NumPorts-1:0] dspm_ports_out;

  dcache_req_i_t [NumPorts-1:0] ispm_ports_out;
  dcache_req_o_t [NumPorts-1:0] ispm_ports_in;

  // ------------------
  // Cache vs. SPM split
  // ------------------

  typedef logic [CVA6Cfg.PLEN-1:0] paddr_t;

  // Possible request destinations
  typedef enum logic [1:0] {
    CACHE_REQ = 2'h0,
    DSPM_REQ  = 2'h1,
    ISPM_REQ  = 2'h2,
    INVALID_REQ
  } req_dest_t;

  typedef struct packed {
    logic [1:0] idx;
    paddr_t     start_addr;
    paddr_t     end_addr;
  } rule_t;

  localparam paddr_t DCacheSpmAddrEnd = CVA6Cfg.DCacheSpmAddrBase + CVA6Cfg.DCacheSpmLength;
  localparam paddr_t ICacheSpmAddrEnd = CVA6Cfg.ICacheSpmAddrBase + CVA6Cfg.ICacheSpmLength;

  // This defines which address ranges are attributed to the SPMs
  // Any space between the DSPM and ISPM does not get an extra rule
  // but is handled by the address decoders default index
  // (which is set to CACHE)
  rule_t [3:0] dcache_spm_map;
  assign dcache_spm_map = {
    { CACHE_REQ,                     56'h0, CVA6Cfg.DCacheSpmAddrBase },
    {  DSPM_REQ, CVA6Cfg.DCacheSpmAddrBase,          DCacheSpmAddrEnd },
    {  ISPM_REQ, CVA6Cfg.ICacheSpmAddrBase,          ICacheSpmAddrEnd },
    { CACHE_REQ,          ICacheSpmAddrEnd,                     56'h0 }
  };

  // This uses 2'b11 as "no current request" state
  logic [2:0] ispm_port_next, ispm_port_d, ispm_port_q;

  // Icache SPM arbitration
  // This handles which port gets forwarded to the ISPM
  always_comb begin
    ispm_port_d    = ispm_port_q;
    ispm_port_next = 3'b111;
    ispm_req_o     = '{default: 0};
    ispm_ports_in  = '{default: 0};
    // Lower indices are prioritized
    for(int unsigned i = 0; i < NumPorts; i++) begin
      if(ispm_ports_out[i].data_req) begin
        ispm_port_next = 3'(i);
        break;
      end
    end
    // Not serving a request => take the next one
    if(ispm_port_q == 3'b111) begin
      if(!(&ispm_port_next)) begin
        ispm_port_d = ispm_port_next;
        ispm_req_o = ispm_ports_out[ispm_port_next];
        ispm_ports_in[ispm_port_next] = ispm_req_i;
      end
    // We're still serving a request so wait for it
    // to complete
    end else begin
      ispm_req_o = ispm_ports_out[ispm_port_q];
      ispm_ports_in[ispm_port_q] = ispm_req_i;
      if(ispm_ports_in[ispm_port_q].data_rvalid || ispm_ports_in[ispm_port_q].data_gnt) begin
        ispm_port_d = 3'b111;
      end
    end
  end

  `FF(ispm_port_q, ispm_port_d, 3'b111, clk_i, rst_ni)

  // Address decoding logic
  // One for each port
  generate
    for (genvar i = 0; i < NumPorts; i++) begin: address_decode

      addr_decode_state_t adec_state_d, adec_state_q;
      logic [CVA6Cfg.DCACHE_INDEX_WIDTH-1:0] addr_idx_d, addr_idx_q;
      logic [CVA6Cfg.DCACHE_TAG_WIDTH-1:0] addr_tag_d, addr_tag_q;
      req_dest_t req_dest_d, req_dest_q;
      logic [1:0] cur_idx;

      // Decode the address (rather, the address tag)
      // of an incoming request
      addr_decode #(
          .NoIndices ( 3       ),
          .NoRules   ( 4       ),
          .addr_t    ( paddr_t ),
          .rule_t    ( rule_t  ),
          .Napot     ( 0       )
      ) i_addr_dec (
          .addr_i           ( {req_ports_i[i].address_tag, {CVA6Cfg.DCACHE_INDEX_WIDTH{1'b0}}}  ),
          .addr_map_i       ( dcache_spm_map    ),
          .idx_o            ( cur_idx           ),
          .dec_valid_o      (                   ),
          .dec_error_o      (                   ),
          .en_default_idx_i ( 1'b1              ),
          .default_idx_i    ( CACHE_REQ         )
      );

      always_comb begin
        addr_idx_d   = addr_idx_q;
        addr_tag_d   = addr_tag_q;
        adec_state_d = adec_state_q;
        req_dest_d   = req_dest_q;

        // By default we forward all communication to
        // all targets, but without the valid and kill signals
        // That way fewer muxes/logic levels are needed for
        // the majority of the signals
        cache_ports_in[i]           = req_ports_i[i];
        cache_ports_in[i].data_req  = 1'b0;
        cache_ports_in[i].tag_valid = 1'b0;
        cache_ports_in[i].kill_req  = 1'b0;

        dspm_ports_in[i]            = req_ports_i[i];
        dspm_ports_in[i].data_req   = 1'b0;
        dspm_ports_in[i].tag_valid  = 1'b0;
        dspm_ports_in[i].kill_req   = 1'b0;

        ispm_ports_out[i]           = req_ports_i[i];
        ispm_ports_out[i].data_req  = 1'b0;
        ispm_ports_out[i].tag_valid = 1'b0;
        ispm_ports_out[i].kill_req  = 1'b0;

        // As the answer will most likely come from the
        // cache we forward that by default, also without valid
        req_ports_o[i]              = cache_ports_out[i];
        req_ports_o[i].data_gnt     = 1'b0;
        req_ports_o[i].data_rvalid  = 1'b0;

        unique case (adec_state_q)
          IDLE: begin
            // By default, everything goes to the cache
            req_dest_d = CACHE_REQ;
            // If we got a request and are not stalled, process it
            if (req_ports_i[i].data_req && !stall_i) begin
              // Save the address index, as it's usually only valid
              // until the request is granted (for a read)
              addr_idx_d = req_ports_i[i].address_index;
              // If this is a write, we know both the address index
              // and the tag, so we can decide where to forward it to
              // immediately
              if(req_ports_i[i].data_we) begin
                adec_state_d = WRITE;
                req_dest_d = req_dest_t'(cur_idx);
                // Connect the full signals to the correct port
                unique case (req_dest_q)
                  CACHE_REQ: begin
                    cache_ports_in[i] = req_ports_i[i];
                    req_ports_o[i] = cache_ports_out[i];
                  end
                  DSPM_REQ: begin
                      dspm_ports_in[i] = req_ports_i[i];
                      req_ports_o[i] = dspm_ports_out[i];
                  end
                  ISPM_REQ: begin
                    ispm_ports_out[i] = req_ports_i[i];
                    req_ports_o[i] = ispm_ports_in[i];
                  end
                  default: begin
                    cache_ports_in[i] = req_ports_i[i];
                    req_ports_o[i] = cache_ports_out[i];
                  end
                endcase
                // If the write could be acknowledged already
                // we stay in the idle state
                if(req_ports_o[i].data_gnt) begin
                  adec_state_d = IDLE;
                end
              // If it's a read on the other hand, we need to wait
              // for the tag to be valid to decide. Until then,
              // forward it to the cache
              end else begin
                adec_state_d = WAIT_TAG;
                cache_ports_in[i] = req_ports_i[i];
                req_ports_o[i] = cache_ports_out[i];
              end
            end
          end

          WAIT_TAG: begin
            // By default we forward to the cache
            cache_ports_in[i] = req_ports_i[i];
            req_ports_o[i] = cache_ports_out[i];
            // Once the tag is valid, we can see where this request
            // needs to go
            if (req_ports_i[i].tag_valid) begin
              req_dest_d = req_dest_t'(cur_idx);
              // Here we enter if it is NOT a cache request
              if(req_dest_d != CACHE_REQ) begin
                adec_state_d = WAIT_SPM;
                // Now we can also record the rag
                addr_tag_d = req_ports_i[i].address_tag;
                // Now dispatch the request to the correct SPM
                if(req_dest_d == DSPM_REQ) begin
                  dspm_ports_in[i] = req_ports_i[i];
                  req_ports_o[i] = dspm_ports_out[i];
                  // Inject the previously recorded index
                  dspm_ports_in[i].address_index = addr_idx_q;
                  // All reads have been granted by the cache before,
                  // this is just to signal a pending request to the SPM controller
                  dspm_ports_in[i].data_req = 1'b1;
                end else begin
                  ispm_ports_out[i] = req_ports_i[i];
                  req_ports_o[i] = ispm_ports_in[i];
                  // Inject the previously recorded index
                  ispm_ports_out[i].address_index = addr_idx_q;
                  // All reads have been granted by the cache before,
                  // this is just to signal a pending request to the SPM controller
                  ispm_ports_out[i].data_req = 1'b1;
                end
                // Kill the started cache request as this is an SPM access
                cache_ports_in[i].kill_req = 1'b1;
              // If it is a normal cache request we just wait it out
              end else begin
                adec_state_d = WAIT_CACHE;
              end
              // If the request could already be answered in this cycle
              // we return to the idle state
              if(req_ports_o[i].data_rvalid) begin
                adec_state_d = IDLE;
              end
              // Speculative grants are only handed out to reads so we can
              // just re-enter the WAIT_TAG state while recording the new
              // index
              if(req_ports_o[i].data_gnt) begin
                addr_idx_d = req_ports_i[i].address_index;
                adec_state_d = WAIT_TAG;
              end
            end
          end

          // This state connects the port with the cache
          // and waits for it to finish
          WAIT_CACHE: begin
            cache_ports_in[i] = req_ports_i[i];
            req_ports_o[i] = cache_ports_out[i];
            if(req_ports_o[i].data_rvalid) begin
              adec_state_d = IDLE;
            end
            // If the next read was already granted, go right
            // back to the state that waits for the tag
            if(req_ports_o[i].data_gnt) begin
              adec_state_d = WAIT_TAG;
            end
          end

          // This state just waits for the SPM(s) to finish
          WAIT_SPM: begin
            if(req_dest_q == DSPM_REQ) begin
              dspm_ports_in[i] = req_ports_i[i];
              req_ports_o[i] = dspm_ports_out[i];
              dspm_ports_in[i].address_index = addr_idx_q;
              dspm_ports_in[i].address_tag = addr_tag_q;
              // All reads have been granted by the cache before,
              // this is just to signal a pending request to the SPM controller
              dspm_ports_in[i].data_req = 1'b1;
            // If we end up here it has to be an ISPM request
            end else begin
              ispm_ports_out[i] = req_ports_i[i];
              req_ports_o[i] = ispm_ports_in[i];
              ispm_ports_out[i].address_index = addr_idx_q;
              ispm_ports_out[i].address_tag = addr_tag_q;
              ispm_ports_out[i].data_req = 1'b1;
            end
            // As soon as the read data is valid, we go back to idle
            if(req_ports_o[i].data_rvalid) begin
              adec_state_d = IDLE;
            end
          end

          // As the name implies, this state just waits for a write to finish
          WRITE: begin
            unique case (req_dest_q)
              CACHE_REQ: begin
                cache_ports_in[i] = req_ports_i[i];
                req_ports_o[i] = cache_ports_out[i];
              end
              DSPM_REQ: begin
                dspm_ports_in[i] = req_ports_i[i];
                req_ports_o[i] = dspm_ports_out[i];
              end
              ISPM_REQ: begin
                ispm_ports_out[i] = req_ports_i[i];
                req_ports_o[i] = ispm_ports_in[i];
              end
              default: begin
                cache_ports_in[i] = req_ports_i[i];
                req_ports_o[i] = cache_ports_out[i];
              end
            endcase
            if(req_ports_o[i].data_gnt) begin
              adec_state_d = IDLE;
            end
          end

          default: begin
          end
        endcase
        // If we get a kill request at any point in time, we just return to idle
        if(req_ports_i[i].kill_req) begin
          adec_state_d = IDLE;
        end
      end

      `FF(addr_idx_q, addr_idx_d, '0, clk_i, rst_ni)
      `FF(addr_tag_q, addr_tag_d, '0, clk_i, rst_ni)
      `FF(adec_state_q, adec_state_d, IDLE, clk_i, rst_ni)
      `FF(req_dest_q, req_dest_d, CACHE_REQ, clk_i, rst_ni)
    end
  endgenerate

  // ------------------
  // SPM Controller
  // ------------------

  dspm_ctrl #(
      .dcache_req_i_t ( dcache_req_i_t             ),
      .dcache_req_o_t ( dcache_req_o_t             ),
      .NR_PORTS       ( NumPorts                   ),
      .NR_WAYS        ( CVA6Cfg.DCACHE_SET_ASSOC   ),
      .LINE_WIDTH     ( CVA6Cfg.DCACHE_LINE_WIDTH  ),
      .ADDR_WIDTH     ( CVA6Cfg.DCACHE_INDEX_WIDTH ),
      .MEMORY_WIDTH   ( DCACHE_MEMORY_WIDTH        ),
      .IDX_WIDTH      ( CVA6Cfg.DCACHE_INDEX_WIDTH ),
      .NR_WAIT_STAGES ( 1                          )
  ) i_dspm_ctrl (
      .clk_i              ( clk_i             ),
      .rst_ni             ( rst_ni            ),
      .active_ways_i      ( dcache_spm_ways_i ),
      .spm_req_ports_i    ( dspm_ports_in     ),
      .spm_req_ports_o    ( dspm_ports_out    ),
      .req_o              ( req_spm           ),
      .addr_o             ( addr_spm          ),
      .wdata_o            ( wdata_spm         ),
      .we_o               ( we_spm            ),
      .be_o               ( be_spm            ),
      .rdata_i            ( rdata_spm         )
  );

  // ------------------
  // Cache Controller
  // ------------------
  generate
    for (genvar i = 0; i < NumPorts; i++) begin : master_ports
      cache_ctrl #(
          .CVA6Cfg(CVA6Cfg),
          .cache_line_t(cache_line_t),
          .cl_be_t(cl_be_t),
          .dcache_req_i_t(dcache_req_i_t),
          .dcache_req_o_t(dcache_req_o_t)
      ) i_cache_ctrl (
          .bypass_i  (~enable_i),
          .busy_o    (busy[i]),
          .stall_i   (stall_i | flush_i),
          // from core
          .req_port_i(cache_ports_in[i]),
          .req_port_o(cache_ports_out[i]),
          // to SRAM array
          .req_o     (req[i+1]),
          .addr_o    (addr[i+1]),
          .gnt_i     (gnt[i+1]),
          .data_i    (rdata),
          .tag_o     (tag[i+1]),
          .data_o    (wdata[i+1]),
          .we_o      (we[i+1]),
          .be_o      (be[i+1]),
          // Ensure that only cache owned ways can hit
          .hit_way_i (hit_way & ~dcache_spm_ways_i),

          .miss_req_o           (miss_req[i]),
          .miss_gnt_i           (miss_gnt[i]),
          .active_serving_i     (active_serving[i]),
          .critical_word_i      (critical_word),
          .critical_word_valid_i(critical_word_valid),
          .bypass_gnt_i         (bypass_gnt[i]),
          .bypass_valid_i       (bypass_valid[i]),
          .bypass_data_i        (bypass_data[i]),

          .mshr_addr_o         (mshr_addr[i]),
          .mshr_addr_matches_i (mshr_addr_matches[i]),
          .mshr_index_matches_i(mshr_index_matches[i]),
          .*
      );
    end
  endgenerate

  // ------------------
  // Miss Handling Unit
  // ------------------
  miss_handler #(
      .CVA6Cfg(CVA6Cfg),
      .NR_PORTS(NumPorts),
      .axi_req_t(axi_req_t),
      .axi_rsp_t(axi_rsp_t),
      .cache_line_t(cache_line_t),
      .cl_be_t(cl_be_t)
  ) i_miss_handler (
      .busy_o               (miss_handler_busy),
      .flush_i              (flush_i),
      .busy_i               (|busy),
      .spm_ways_i           (dcache_spm_ways_i),
      // AMOs
      .amo_req_i            (amo_req_i),
      .amo_resp_o           (amo_resp_o),
      .miss_req_i           (miss_req),
      .miss_gnt_o           (miss_gnt),
      .bypass_gnt_o         (bypass_gnt),
      .bypass_valid_o       (bypass_valid),
      .bypass_data_o        (bypass_data),
      .critical_word_o      (critical_word),
      .critical_word_valid_o(critical_word_valid),
      .mshr_addr_i          (mshr_addr),
      .mshr_addr_matches_o  (mshr_addr_matches),
      .mshr_index_matches_o (mshr_index_matches),
      .active_serving_o     (active_serving),
      .req_o                (req[0]),
      .addr_o               (addr[0]),
      .data_i               (rdata),
      .be_o                 (be[0]),
      .data_o               (wdata[0]),
      .we_o                 (we[0]),
      .axi_bypass_o,
      .axi_bypass_i,
      .axi_data_o,
      .axi_data_i,
      .*
  );

  assign tag[0] = '0;

  // --------------
  // Memory Arrays
  // --------------
  for (genvar i = 0; i < CVA6Cfg.DCACHE_SET_ASSOC; i++) begin : sram_block
      always_comb begin
        // Is this cache way SPM owned?
        if(dcache_spm_ways_i[i]) begin
          req_ram[i]      = req_spm[i];
          we_ram[i]       = we_spm;
          addr_ram[i]     = addr_spm;
          wdata_ram[i]    = wdata_spm;
          be_ram[i]       = be_spm;
        // Otherwise the cache accesses it
        end else begin
          req_ram[i]      = req_cache[i];
          we_ram[i]       = we_cache;
          addr_ram[i]     = addr_cache;
          wdata_ram[i]    = {wdata_cache.tag, wdata_cache.data};
          be_ram[i]       = {be_cache.tag, be_cache.data};
        end
      end

      // Read data can be forwarded to both
      assign {rdata_cache[i].tag, rdata_cache[i].data} = rdata_ram[i];
      assign rdata_spm[i] = rdata_ram[i];

      sram #(
        .DATA_WIDTH(DCACHE_MEMORY_WIDTH),
        .NUM_WORDS (CVA6Cfg.DCACHE_NUM_WORDS),
        .impl_in_t (impl_in_t)
      ) i_cachline_sram (
        .req_i  (req_ram[i]),
        .rst_ni (rst_ni),
        .impl_i (sram_impl_i[i]),
        .we_i   (we_ram[i]),
        .addr_i (addr_ram[i][CVA6Cfg.DCACHE_INDEX_WIDTH-1:CVA6Cfg.DCACHE_OFFSET_WIDTH]),
        .wuser_i('0),
        .wdata_i(wdata_ram[i]),
        .be_i   (be_ram[i]),
        .ruser_o(),
        .rdata_o(rdata_ram[i]),
        .*
      );
  end

  // ----------------
  // Valid/Dirty Regs
  // ----------------

  vldrty_t [CVA6Cfg.DCACHE_SET_ASSOC-1:0] dirty_wdata, dirty_rdata;

  for (genvar i = 0; i < CVA6Cfg.DCACHE_SET_ASSOC; i++) begin
    // If the way is owned by the SPM it's always clean and not valid
    assign dirty_wdata[i] = (dcache_spm_ways_i[i]) ? '{dirty: {CVA6Cfg.DCACHE_SET_ASSOC{1'b0}}, valid: 1'b0}
                                                   : '{dirty: wdata_cache.dirty, valid: wdata_cache.valid};
    assign rdata_cache[i].dirty = dirty_rdata[i].dirty;
    assign rdata_cache[i].valid = dirty_rdata[i].valid;
    // Always write the zeroes of SPM owned ways to the memory array
    assign be_valid_dirty_ram[i].valid = be_cache.vldrty[i].valid | dcache_spm_ways_i[i];
    assign be_valid_dirty_ram[i].dirty = be_cache.vldrty[i].dirty | {((CVA6Cfg.DCACHE_LINE_WIDTH+7)/8){dcache_spm_ways_i[i]}};
  end

  sram #(
      .USER_WIDTH(1),
      .DATA_WIDTH(CVA6Cfg.DCACHE_SET_ASSOC * $bits(vldrty_t)),
      .BYTE_WIDTH(1),
      .NUM_WORDS (CVA6Cfg.DCACHE_NUM_WORDS),
      .impl_in_t (impl_in_t)
  ) valid_dirty_sram (
      .clk_i  (clk_i),
      .rst_ni (rst_ni),
      .impl_i (sram_impl_i[CVA6Cfg.DCACHE_SET_ASSOC]),
      .req_i  (|req_cache),
      .we_i   (we_cache),
      .addr_i (addr_cache[CVA6Cfg.DCACHE_INDEX_WIDTH-1:CVA6Cfg.DCACHE_OFFSET_WIDTH]),
      .wuser_i('0),
      .wdata_i(dirty_wdata),
      .be_i   (be_valid_dirty_ram),
      .ruser_o(),
      .rdata_o(dirty_rdata)
  );

  // ------------------------------------------------
  // Tag Comparison and memory arbitration
  // ------------------------------------------------
  tag_cmp #(
      .CVA6Cfg   (CVA6Cfg),
      .NR_PORTS  (NumPorts + 1),
      .ADDR_WIDTH(CVA6Cfg.DCACHE_INDEX_WIDTH),
      .l_data_t  (cache_line_t),
      .l_be_t    (cl_be_t)
  ) i_tag_cmp (
      .req_i    (req),
      .gnt_o    (gnt),
      .addr_i   (addr),
      .wdata_i  (wdata),
      .we_i     (we),
      .be_i     (be),
      .rdata_o  (rdata),
      .tag_i    (tag),
      .hit_way_o(hit_way),

      .req_o  (req_cache),
      .addr_o (addr_cache),
      .wdata_o(wdata_cache),
      .we_o   (we_cache),
      .be_o   (be_cache),
      .rdata_i(rdata_cache),
      .*
  );


  //pragma translate_off
  initial begin
    assert (CVA6Cfg.DCACHE_LINE_WIDTH / CVA6Cfg.AxiDataWidth inside {2, 4, 8, 16})
    else $fatal(1, "Cache line size needs to be a power of two multiple of AxiDataWidth");
  end
  //pragma translate_on
endmodule
