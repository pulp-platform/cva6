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
// Author: Michael Schaffner <schaffner@iis.ee.ethz.ch>, ETH Zurich
// Date: 15.08.2018
// Description: Instruction cache that is compatible with openpiton.
//
// Some notes:
//
// 1) refills always have the size of one cache line, except for accesses to the I/O region, which is mapped
//    to the top half of the physical address space (bit 39 = 1). the data width of the interface has the width
//    of one cache line, and hence the ifills can be transferred in a single cycle. note that the ifills must be
//    consumed unconditionally.
//
// 2) instruction fetches are always assumed to be aligned to 32bit (lower 2 bits are ignored)
//
// 3) NC accesses to I/O space are expected to return 32bit from memory.
//


module cva6_icache
  import ariane_pkg::*;
  import wt_cache_pkg::*;
#(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty,
    parameter type icache_areq_t = logic,
    parameter type icache_arsp_t = logic,
    parameter type icache_dreq_t = logic,
    parameter type icache_drsp_t = logic,
    parameter type icache_req_t = logic,
    parameter type icache_rtrn_t = logic,
    parameter type dcache_req_i_t = logic,
    parameter type dcache_req_o_t = logic,
    /// ID to be used for read transactions
    parameter logic [CVA6Cfg.MEM_TID_WIDTH-1:0] RdTxId = 0,
    parameter type impl_in_t = logic
) (
    input logic clk_i,
    input logic rst_ni,

    /// flush the icache, flush and kill have to be asserted together
    input  logic         flush_i,
    /// enable icache
    input  logic         en_i,
    /// to performance counter
    output logic         miss_o,
    output logic         busy_o,
    input  logic         stall_i,
    input  logic         init_ni,         // do not init after enabling
    input  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0] icache_spm_ways_i,
    input  impl_in_t [CVA6Cfg.ICACHE_SET_ASSOC-1:0] sram_impl_i,
    // address translation requests
    input  icache_areq_t areq_i,
    output icache_arsp_t areq_o,
    // data requests
    input  icache_dreq_t dreq_i,
    output icache_drsp_t dreq_o,
    input  dcache_req_i_t ispm_req_i,
    output dcache_req_o_t ispm_req_o,
    // refill port
    input  logic         mem_rtrn_vld_i,
    input  icache_rtrn_t mem_rtrn_i,
    output logic         mem_data_req_o,
    input  logic         mem_data_ack_i,
    output icache_req_t  mem_data_o
);

  localparam ICACHE_OFFSET_WIDTH = $clog2(CVA6Cfg.ICACHE_LINE_WIDTH / 8);
  localparam ICACHE_NUM_WORDS = 2 ** (CVA6Cfg.ICACHE_INDEX_WIDTH - ICACHE_OFFSET_WIDTH);
  localparam ICACHE_CL_IDX_WIDTH = $clog2(ICACHE_NUM_WORDS);  // excluding byte offset

  // functions
  function automatic logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0] icache_way_bin2oh(
      input logic [CVA6Cfg.ICACHE_SET_ASSOC_WIDTH-1:0] in);
    logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0] out;
    out     = '0;
    out[in] = 1'b1;
    return out;
  endfunction

  // signals
  logic cache_en_d, cache_en_q;  // cache is enabled
  logic [CVA6Cfg.VLEN-1:0] vaddr_d, vaddr_q;
  logic paddr_is_nc;  // asserted if physical address is non-cacheable
  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0] cl_hit;  // hit from tag compare
  logic cache_rden;  // triggers cache lookup
  logic cache_wren;  // triggers write to cacheline
  logic
      cmp_en_d,
      cmp_en_q;  // enable tag comparison in next cycle. used to cut long path due to NC signal.
  logic flush_d, flush_q;  // used to register and signal pending flushes

  // replacement strategy
  logic                                      update_lfsr;  // shift the LFSR
  logic [CVA6Cfg.ICACHE_SET_ASSOC_WIDTH-1:0] inv_way;  // first non-valid encountered
  logic [CVA6Cfg.ICACHE_SET_ASSOC_WIDTH-1:0] rnd_way;  // random index for replacement
  logic [CVA6Cfg.ICACHE_SET_ASSOC_WIDTH-1:0] repl_way;  // way to replace
  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0] repl_way_oh_raw;
  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0] repl_way_oh_d, repl_way_oh_q;  // way to replace (onehot)
  logic all_ways_valid;  // we need to switch repl strategy since all are valid

  // invalidations / flushing
  logic inv_en;  // incoming invalidations
  logic inv_d, inv_q;  // invalidation in progress
  logic flush_en, flush_done;  // used to flush cache entries
  logic [ICACHE_CL_IDX_WIDTH-1:0] flush_cnt_d, flush_cnt_q;  // used to flush cache entries

  // mem arrays
  logic                                cl_we;  // write enable to memory array
  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0] cl_req;  // request to memory array
  logic [     ICACHE_CL_IDX_WIDTH-1:0] cl_index;  // this is a cache-line index, to memory array
  logic [ICACHE_OFFSET_WIDTH-1:0] cl_offset_d, cl_offset_q;  // offset in cache line
  logic [CVA6Cfg.ICACHE_TAG_WIDTH-1:0] cl_tag_d, cl_tag_q;  // this is the cache tag
  logic [CVA6Cfg.ICACHE_TAG_WIDTH-1:0]          cl_tag_rdata [CVA6Cfg.ICACHE_SET_ASSOC-1:0]; // these are the tags coming from the tagmem
  logic [CVA6Cfg.ICACHE_LINE_WIDTH-1:0]         cl_rdata     [CVA6Cfg.ICACHE_SET_ASSOC-1:0]; // these are the cachelines coming from the cache
  logic [CVA6Cfg.ICACHE_USER_LINE_WIDTH-1:0]    cl_ruser[CVA6Cfg.ICACHE_SET_ASSOC-1:0]; // these are the cachelines coming from the user cache
  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0][CVA6Cfg.FETCH_WIDTH-1:0] cl_sel;  // selected word from each cacheline
  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0][CVA6Cfg.FETCH_USER_WIDTH-1:0] cl_user;  // selected word from each cacheline
  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0] vld_req;  // bit enable for valid regs
  logic vld_we;  // valid bits write enable
  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0] vld_wdata;  // valid bits to write
  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0] vld_rdata;  // valid bits coming from valid regs
  logic [ICACHE_CL_IDX_WIDTH-1:0] vld_addr;  // valid bit

  // Demux ports
  icache_dreq_t spm_port_in;
  icache_drsp_t spm_port_out;

  // cpmtroller FSM
  typedef enum logic [2:0] {
    FLUSH,
    IDLE,
    READ,
    MISS,
    KILL_ATRANS,
    KILL_MISS
  } state_e;
  state_e state_d, state_q;

  assign busy_o = (state_q != IDLE);

  ///////////////////////////////////////////////////////
  // address -> cl_index mapping, interface plumbing
  ///////////////////////////////////////////////////////

  // extract tag from physical address, check if NC
  assign cl_tag_d  = (areq_i.fetch_valid) ? areq_i.fetch_paddr[CVA6Cfg.ICACHE_TAG_WIDTH+CVA6Cfg.ICACHE_INDEX_WIDTH-1:CVA6Cfg.ICACHE_INDEX_WIDTH] : cl_tag_q;

  // noncacheable if request goes to I/O space, or if cache is disabled
  assign paddr_is_nc = (~cache_en_q) | (~config_pkg::is_inside_cacheable_regions(
      CVA6Cfg, {{64 - CVA6Cfg.PLEN{1'b0}}, cl_tag_d, {CVA6Cfg.ICACHE_INDEX_WIDTH{1'b0}}}
  ));

  // pass exception through
  assign dreq_o.ex = areq_i.fetch_exception;

  // latch this in case we have to stall later on
  // make sure this is 32bit aligned
  assign vaddr_d = (dreq_o.ready & dreq_i.req) ? dreq_i.vaddr : vaddr_q;
  assign areq_o.fetch_vaddr = (vaddr_q >> CVA6Cfg.FETCH_ALIGN_BITS) << CVA6Cfg.FETCH_ALIGN_BITS;

  // split virtual address into index and offset to address cache arrays
  assign cl_index = vaddr_d[CVA6Cfg.ICACHE_INDEX_WIDTH-1:ICACHE_OFFSET_WIDTH];


  if (CVA6Cfg.NOCType == config_pkg::NOC_TYPE_AXI4_ATOP) begin : gen_axi_offset
    // if we generate a noncacheable access, the word will be at offset 0 or 4 in the cl coming from memory
    assign cl_offset_d = ( dreq_o.ready & dreq_i.req)      ? (dreq_i.vaddr >> CVA6Cfg.FETCH_ALIGN_BITS) << CVA6Cfg.FETCH_ALIGN_BITS :
                         ( paddr_is_nc  & mem_data_req_o ) ? {{ICACHE_OFFSET_WIDTH-1{1'b0}}, cl_offset_q[2]}<<2 : // needed since we transfer 32bit over a 64bit AXI bus in this case
        cl_offset_q;
    // request word address instead of cl address in case of NC access
    assign mem_data_o.paddr = (paddr_is_nc) ? {cl_tag_d, vaddr_q[CVA6Cfg.ICACHE_INDEX_WIDTH-1:3], 3'b0} :                                         // align to 64bit
        {cl_tag_d, vaddr_q[CVA6Cfg.ICACHE_INDEX_WIDTH-1:ICACHE_OFFSET_WIDTH], {ICACHE_OFFSET_WIDTH{1'b0}}}; // align to cl
  end else begin : gen_piton_offset
    // icache fills are either cachelines or 4byte fills, depending on whether they go to the Piton I/O space or not.
    // since the piton cache system replicates the data, we can always index the full CL
    assign cl_offset_d = (dreq_o.ready & dreq_i.req) ? {dreq_i.vaddr >> 2, 2'b0} : cl_offset_q;

    // request word address instead of cl address in case of NC access
    assign mem_data_o.paddr = (paddr_is_nc) ? {cl_tag_d, vaddr_q[CVA6Cfg.ICACHE_INDEX_WIDTH-1:2], 2'b0} :                                         // align to 32bit
        {cl_tag_d, vaddr_q[CVA6Cfg.ICACHE_INDEX_WIDTH-1:ICACHE_OFFSET_WIDTH], {ICACHE_OFFSET_WIDTH{1'b0}}}; // align to cl
  end


  assign mem_data_o.tid = RdTxId;

  assign mem_data_o.nc  = paddr_is_nc;
  // way that is being replaced
  assign mem_data_o.way = repl_way;
  assign dreq_o.vaddr   = vaddr_q;

  // invalidations take two cycles
  assign inv_d          = inv_en;

///////////////////////////////////////////////////////
// demultiplexing cache/spm logic
///////////////////////////////////////////////////////
  typedef logic [CVA6Cfg.PLEN-1:0] paddr_t;

  typedef struct packed {
      logic   idx;
      paddr_t start_addr;
      paddr_t end_addr;
  } rule_t;

  localparam logic [55:0] ICacheSpmAddrEnd = CVA6Cfg.ICacheSpmAddrBase + CVA6Cfg.ICacheSpmLength;

  // The address space map dividing it into ISPM and anything else
  rule_t [2:0] icache_spm_map;
  assign icache_spm_map = {
      { 1'b0,                     56'h0, CVA6Cfg.ICacheSpmAddrBase },
      { 1'b1, CVA6Cfg.ICacheSpmAddrBase,          ICacheSpmAddrEnd },
      { 1'b0,          ICacheSpmAddrEnd,                     56'h0 }
  };

  // We only need one bit to decide between SPM and not
  logic cur_idx;

  addr_decode #(
      .NoIndices  ( 2       ),
      .NoRules    ( 3       ),
      .addr_t     ( paddr_t ),
      .rule_t     ( rule_t  ),
      .Napot      ( 0       )
  ) i_addr_dec (
      .addr_i           ( areq_i.fetch_paddr ),
      .addr_map_i       ( icache_spm_map     ),
      .idx_o            ( cur_idx            ),
      .dec_valid_o      (                    ),
      .dec_error_o      (                    ),
      .en_default_idx_i ( 1'b1               ),
      .default_idx_i    ( '0                 )
  );

  ///////////////////////////////////////////////////////
  // main control logic
  ///////////////////////////////////////////////////////
  logic addr_ni;
  assign addr_ni = config_pkg::is_inside_nonidempotent_regions(
      CVA6Cfg, {{64 - CVA6Cfg.PLEN{1'b0}}, areq_i.fetch_paddr}
  );
  always_comb begin : p_fsm
    // default assignment
    state_d = state_q;
    cache_en_d   = (cache_en_q | init_ni) & en_i;// disabling the cache is always possible, enable needs to go via flush if we init
    flush_en = 1'b0;
    cmp_en_d = 1'b0;
    cache_rden = 1'b0;
    cache_wren = 1'b0;
    inv_en = 1'b0;
    flush_d = flush_q | flush_i;  // register incoming flush
    // I-SPM: by default we forward everything except the request signal
    spm_port_in = dreq_i;
    spm_port_in.req = 1'b0;
    // And we always kill the previous request, if we do not want to explicitely keep it
    spm_port_in.kill_s2 = 1'b1;

    // interfaces
    dreq_o.ready = 1'b0;
    areq_o.fetch_req = 1'b0;
    dreq_o.valid = 1'b0;
    mem_data_req_o = 1'b0;
    // performance counter
    miss_o = 1'b0;

    // handle invalidations unconditionally
    // note: invald are mutually exclusive with
    // ifills, since both arrive over the same IF
    // however, we need to make sure below that we
    // do not trigger a cache readout at the same time...
    if (mem_rtrn_vld_i && mem_rtrn_i.rtype == ICACHE_INV_REQ) begin
      inv_en = 1'b1;
    end

    unique case (state_q)
      //////////////////////////////////
      // this clears all valid bits
      FLUSH: begin
        flush_en = 1'b1;
        if (flush_done) begin
          state_d = IDLE;
          flush_d = 1'b0;
          // if the cache was not enabled set this
          cache_en_d = en_i;
        end
      end
      //////////////////////////////////
      // wait for an incoming request
      IDLE: begin
        // only enable tag comparison if cache is enabled
        cmp_en_d = cache_en_q;

        // handle pending flushes, or perform cache clear upon enable
        if (flush_d || (en_i && !cache_en_q && !init_ni)) begin
          state_d = FLUSH;
          // wait for incoming requests
        end else if (!stall_i) begin
          // mem requests are for sure invals here
          if (!mem_rtrn_vld_i) begin
            dreq_o.ready = 1'b1;
            // we have a new request
            if (dreq_i.req) begin
              // speculatively forward the request to the SPM
              spm_port_in = dreq_i;
              cache_rden = 1'b1;
              state_d    = READ;
            end
          end
          if (dreq_i.kill_s1) begin
            state_d = IDLE;
          end
        end
      end
      //////////////////////////////////
      // check whether we have a hit
      // in case the cache is disabled,
      // or in case the address is NC, we
      // reuse the miss mechanism to handle
      // the request
      READ: begin
        areq_o.fetch_req = '1;
        // only enable tag comparison if cache is enabled
        cmp_en_d    = cache_en_q;
        // readout speculatively
        cache_rden  = cache_en_q;
        spm_port_in = dreq_i;
        // Continue if we have the physical address (for the tag) AND
        // - a non-speculative request, OR
        // - a request to idempotent memory, OR
        // - a request to the I-SPM
        if (areq_i.fetch_valid && (!dreq_i.spec || ((CVA6Cfg.NonIdemPotenceEn && !addr_ni) || (!CVA6Cfg.NonIdemPotenceEn)) || cur_idx)) begin
          // check if we have to flush
          if (flush_d) begin
            state_d = IDLE;
          // this is an SPM access
          end else if (cur_idx) begin
            dreq_o.valid = spm_port_out.valid & ~dreq_i.kill_s2;
            // the SPM request is killed by default but we want to keep it here
            spm_port_in.kill_s2 = dreq_i.kill_s2;
            // wait for the SPM to finish the request
            // this valid is not masked by kill_s2 as it also serves as
            // a "ready" indication to this FSM
            if(spm_port_out.valid) begin
              state_d = IDLE;
              // we can accept another request
              // and stay here, but only if no inval is coming in
              // note: we are not expecting ifill return packets here...
              if (!mem_rtrn_vld_i) begin
                dreq_o.ready  = 1'b1;
                if (dreq_i.req) begin
                  state_d     = READ;
                end
              end
              // if a request is being killed at this stage,
              // we have to bail out and wait for the address translation to complete
              if (dreq_i.kill_s1) begin
                state_d = IDLE;
              end
            end
            // we have a hit or an exception output valid result
          end else if (((|cl_hit && cache_en_q) || areq_i.fetch_exception.valid) && !inv_q) begin
            dreq_o.valid = ~dreq_i.kill_s2;  // just don't output in this case
            state_d      = IDLE;

            // we can accept another request
            // and stay here, but only if no inval is coming in
            // note: we are not expecting ifill return packets here...
            if (!mem_rtrn_vld_i) begin
              dreq_o.ready = 1'b1;
              if (dreq_i.req) begin
                state_d = READ;
              end
            end
            // if a request is being killed at this stage,
            // we have to bail out and wait for the address translation to complete
            if (dreq_i.kill_s1) begin
              state_d = IDLE;
            end
            // we have a miss / NC transaction
          end else if (dreq_i.kill_s2) begin
            state_d = IDLE;
          end else if (!inv_q) begin
            cmp_en_d = 1'b0;
            // only count this as a miss if the cache is enabled, and
            // the address is cacheable
            // send out ifill request
            mem_data_req_o = 1'b1;
            if (mem_data_ack_i) begin
              miss_o  = ~paddr_is_nc;
              state_d = MISS;
            end
          end
          // bail out if this request is being killed (and we missed on the TLB)
        end else if (dreq_i.kill_s2 || flush_d) begin
          state_d = KILL_ATRANS;
        end
      end
      //////////////////////////////////
      // wait until the memory transaction
      // returns. do not write to memory
      // if the nc bit is set.
      MISS: begin
        // note: this is mutually exclusive with ICACHE_INV_REQ,
        // so we do not have to check for invals here
        if (mem_rtrn_vld_i && mem_rtrn_i.rtype == ICACHE_IFILL_ACK) begin
          state_d = IDLE;
          // only return data if request is not being killed
          if (!(dreq_i.kill_s2 || flush_d)) begin
            dreq_o.valid = 1'b1;
            // only write to cache if this address is cacheable
            cache_wren   = ~paddr_is_nc;
          end
          // bail out if this request is being killed
        end else if (dreq_i.kill_s2 || flush_d) begin
          state_d = KILL_MISS;
        end
      end
      //////////////////////////////////
      // killed address translation,
      // wait until paddr is valid, and go
      // back to idle
      KILL_ATRANS: begin
        areq_o.fetch_req = '1;
        if (areq_i.fetch_valid) begin
          state_d = IDLE;
        end
      end
      //////////////////////////////////
      // killed miss,
      // wait until memory responds and
      // go back to idle
      KILL_MISS: begin
        if (mem_rtrn_vld_i && mem_rtrn_i.rtype == ICACHE_IFILL_ACK) begin
          state_d = IDLE;
        end
      end
      default: begin
        // we should never get here
        state_d = FLUSH;
      end
    endcase  // state_q
  end

  ///////////////////////////////////////////////////////
  // valid bit invalidation and replacement strategy
  ///////////////////////////////////////////////////////

  // note: it cannot happen that we get an invalidation + a cl replacement
  // in the same cycle as these requests arrive via the same interface
  // flushes take precedence over invalidations (it is ok if we ignore
  // the inval since the cache is cleared anyway)

  assign flush_cnt_d = (flush_done) ? '0 : (flush_en) ? flush_cnt_q + 1 : flush_cnt_q;

  assign flush_done = (flush_cnt_q == (ICACHE_NUM_WORDS - 1));

  // invalidation/clearing address
  // flushing takes precedence over invals
  assign vld_addr = (flush_en)       ? flush_cnt_q        :
                    (inv_en)         ? mem_rtrn_i.inv.idx[CVA6Cfg.ICACHE_INDEX_WIDTH-1:ICACHE_OFFSET_WIDTH] :
                                       cl_index;

  assign vld_req  = (flush_en || cache_rden)        ? '1                                    :
                    (mem_rtrn_i.inv.all && inv_en)  ? '1                                    :
                    (mem_rtrn_i.inv.vld && inv_en)  ? icache_way_bin2oh(
      mem_rtrn_i.inv.way
  ) : repl_way_oh_q;

  assign vld_wdata = (cache_wren) ? '1 : '0;

  assign vld_we = (cache_wren | inv_en | flush_en);
  // assign vld_req   = (vld_we | cache_rden);


  // chose random replacement if all are valid
  assign update_lfsr = cache_wren & all_ways_valid;
  assign repl_way = (all_ways_valid) ? rnd_way : inv_way;
  assign repl_way_oh_raw  = icache_way_bin2oh(repl_way);
  // If the selected replacement would choose an SPM way we have to patch
  // it to use the first non-SPM way
  assign repl_way_oh_d    = (cmp_en_q) ?
                                      ( (repl_way_oh_raw & icache_spm_ways_i) ?
                                          ((icache_spm_ways_i + 1) & ~icache_spm_ways_i)
                                          : repl_way_oh_raw
                                      )
                                      : repl_way_oh_q;

  // enable signals for memory arrays
  assign cl_req = (cache_rden) ? '1 : (cache_wren) ? repl_way_oh_q : '0;
  assign cl_we = cache_wren;


  // find invalid cache line
  lzc #(
      .WIDTH(CVA6Cfg.ICACHE_SET_ASSOC)
  ) i_lzc (
      .in_i   (~(vld_rdata|icache_spm_ways_i)),  // count SPM ways as valid so they are not replaced
      .cnt_o  (inv_way),
      .empty_o(all_ways_valid)
  );

  // generate random cacheline index
  lfsr #(
      .LfsrWidth(8),
      .OutWidth (CVA6Cfg.ICACHE_SET_ASSOC_WIDTH)
  ) i_lfsr (
      .clk_i (clk_i),
      .rst_ni(rst_ni),
      .en_i  (update_lfsr),
      .out_o (rnd_way)
  );


  ///////////////////////////////////////////////////////
  // tag comparison, hit generation
  ///////////////////////////////////////////////////////

  logic [CVA6Cfg.ICACHE_SET_ASSOC_WIDTH-1:0] hit_idx;

  for (genvar i = 0; i < CVA6Cfg.ICACHE_SET_ASSOC; i++) begin : gen_tag_cmpsel
    assign cl_hit[i] = (cl_tag_rdata[i] == cl_tag_d) & vld_rdata[i];
    assign cl_sel[i] = cl_rdata[i][{cl_offset_q, 3'b0}+:CVA6Cfg.FETCH_WIDTH];
    assign cl_user[i] = CVA6Cfg.FETCH_USER_EN ? cl_ruser[i][{cl_offset_q, 3'b0}+:CVA6Cfg.FETCH_USER_WIDTH] : '0;
  end


  lzc #(
      .WIDTH(CVA6Cfg.ICACHE_SET_ASSOC)
  ) i_lzc_hit (
      .in_i   (cl_hit),
      .cnt_o  (hit_idx),
      .empty_o()
  );

  always_comb begin
    if (cur_idx) begin
      dreq_o.data = spm_port_out.data;
      dreq_o.user = '0;
    end else if(cmp_en_q) begin
      dreq_o.data = cl_sel[hit_idx];
      dreq_o.user = CVA6Cfg.FETCH_USER_EN ? cl_user[hit_idx] : '0;
    end else begin
      dreq_o.data = mem_rtrn_i.data[{cl_offset_q, 3'b0}+:CVA6Cfg.FETCH_WIDTH];
      dreq_o.user = CVA6Cfg.FETCH_USER_EN ? mem_rtrn_i.user[{cl_offset_q, 3'b0}+:CVA6Cfg.FETCH_USER_WIDTH] : '0;
    end
  end

  ///////////////////////////////////////////////////////
  // SPM memory arbiter
  ///////////////////////////////////////////////////////

  localparam int unsigned ICACHE_MEMORY_WIDTH    = 1 + CVA6Cfg.ICACHE_TAG_WIDTH + CVA6Cfg.ICACHE_LINE_WIDTH;
  localparam int unsigned ICACHE_MEMORY_BE_WIDTH = (ICACHE_MEMORY_WIDTH + 7) / 8;

  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0]                             spm_req;
  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0][ICACHE_CL_IDX_WIDTH-1:0]    spm_addr;
  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0][ICACHE_MEMORY_WIDTH-1:0]    spm_wdata;
  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0][ICACHE_MEMORY_WIDTH-1:0]    spm_rdata;
  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0][ICACHE_MEMORY_BE_WIDTH-1:0] spm_be;
  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0]                             spm_we;

  paddr_t icache_phys_addr;

  assign icache_phys_addr = {areq_i.fetch_paddr[CVA6Cfg.PLEN-1:CVA6Cfg.ICACHE_INDEX_WIDTH], vaddr_q[CVA6Cfg.ICACHE_INDEX_WIDTH-1:0]};

  ispm_ctrl #(
      .icache_dreq_t  ( icache_dreq_t              ),
      .icache_drsp_t  ( icache_drsp_t              ),
      .dcache_req_i_t ( dcache_req_i_t             ),
      .dcache_req_o_t ( dcache_req_o_t             ),
      .paddr_t        ( paddr_t                    ),
      .NR_WAYS        ( CVA6Cfg.ICACHE_SET_ASSOC   ),
      .LINE_WIDTH     ( CVA6Cfg.ICACHE_LINE_WIDTH  ),
      .ADDR_WIDTH     ( ICACHE_CL_IDX_WIDTH        ),
      .FETCH_WIDTH    ( CVA6Cfg.FETCH_WIDTH        ),
      .MEMORY_WIDTH   ( ICACHE_MEMORY_WIDTH        ),
      .IDX_WIDTH      ( CVA6Cfg.ICACHE_INDEX_WIDTH ), // Cache index + byte offset
      .NR_WAIT_STAGES ( 1 )
  ) i_ispm_ctrl (
      .clk_i                    ( clk_i              ),
      .rst_ni                   ( rst_ni             ),
      .active_ways_i            ( icache_spm_ways_i  ),
      .icache_req_port_i        ( spm_port_in        ),
      .icache_req_port_o        ( spm_port_out       ),
      .icache_phys_addr_i       ( icache_phys_addr   ),
      .icache_phys_addr_valid_i ( areq_i.fetch_valid ),
      .spm_rw_req_port_i        ( ispm_req_i         ),
      .spm_rw_req_port_o        ( ispm_req_o         ),
      .req_o                    ( spm_req            ),
      .addr_o                   ( spm_addr           ),
      .wdata_o                  ( spm_wdata          ),
      .we_o                     ( spm_we             ),
      .be_o                     ( spm_be             ),
      .rdata_i                  ( spm_rdata          )
  );

  ///////////////////////////////////////////////////////
  // memory arrays and regs
  ///////////////////////////////////////////////////////

  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0]                             ram_req;
  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0][ICACHE_CL_IDX_WIDTH-1:0]    ram_addr;
  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0][ICACHE_MEMORY_WIDTH-1:0]    ram_wdata;
  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0][ICACHE_MEMORY_WIDTH-1:0]    ram_rdata;
  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0][ICACHE_MEMORY_BE_WIDTH-1:0] ram_be;
  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0]                             ram_we;

  for (genvar i = 0; i < CVA6Cfg.ICACHE_SET_ASSOC; i++) begin : gen_sram
    always_comb begin
      // Is this a SPM way?
      if(icache_spm_ways_i[i]) begin
        ram_req[i]    = spm_req[i];
        ram_we[i]     = spm_we[i];
        ram_addr[i]   = spm_addr[i];
        ram_wdata[i]  = spm_wdata[i];
        ram_be[i]     = spm_be[i];
        vld_rdata[i]  = 1'b0;
      end else begin
        ram_req[i]    = cl_req[i] | vld_req[i];
        ram_we[i]     = cl_we | vld_we;
        ram_addr[i]   = vld_addr;
        ram_wdata[i]  = {{vld_wdata[i], cl_tag_q}, mem_rtrn_i.data};
        ram_be[i]     = {{((CVA6Cfg.ICACHE_TAG_WIDTH+1+7)/8){vld_we}}, {(CVA6Cfg.ICACHE_LINE_WIDTH/8){cl_we}}};
        vld_rdata[i]  = ram_rdata[i][ICACHE_MEMORY_WIDTH-1];
      end
      // The read data is always forwarded to both the cache and SPM controller to save some multiplexers
      spm_rdata[i]    = ram_rdata[i];
      cl_rdata[i]     = ram_rdata[i]; // TODO: only select the first CVA6Cfg.ICACHE_LINE_WIDTH bits?
      cl_tag_rdata[i] = ram_rdata[i][CVA6Cfg.ICACHE_LINE_WIDTH +: CVA6Cfg.ICACHE_TAG_WIDTH];
    end

    // Cache/SPM SRAM (tag and data combined)
    sram_cache #(
      .USER_WIDTH  ( CVA6Cfg.ICACHE_USER_LINE_WIDTH ),
      .DATA_WIDTH  ( ICACHE_MEMORY_WIDTH            ),
      .USER_EN     ( CVA6Cfg.FETCH_USER_EN          ),
      .NUM_WORDS   ( ICACHE_NUM_WORDS               ),
      .TECHNO_CUT  ( CVA6Cfg.TechnoCut              ),
      .BYTE_ACCESS ( 1                              ),
      .impl_in_t   ( impl_in_t                      )
    ) icache_sram (
      .clk_i     ( clk_i            ),
      .rst_ni    ( rst_ni           ),
      .impl_i    ( sram_impl_i[i]   ),
      .req_i     ( ram_req[i]       ),
      .we_i      ( ram_we[i]        ),
      .addr_i    ( ram_addr[i]      ),
      .wuser_i   ( mem_rtrn_i.user  ),
      .wdata_i   ( ram_wdata[i]     ),
      .be_i      ( ram_be[i]        ),
      .ruser_o   ( cl_ruser[i]      ),
      .rdata_o   ( ram_rdata[i]     )
    );
  end


  always_ff @(posedge clk_i or negedge rst_ni) begin : p_regs
    if (!rst_ni) begin
      cl_tag_q      <= '0;
      flush_cnt_q   <= '0;
      vaddr_q       <= '0;
      cmp_en_q      <= '0;
      cache_en_q    <= '0;
      flush_q       <= '0;
      state_q       <= FLUSH;
      cl_offset_q   <= '0;
      repl_way_oh_q <= '0;
      inv_q         <= '0;
    end else begin
      cl_tag_q      <= cl_tag_d;
      flush_cnt_q   <= flush_cnt_d;
      vaddr_q       <= vaddr_d;
      cmp_en_q      <= cmp_en_d;
      cache_en_q    <= cache_en_d;
      flush_q       <= flush_d;
      state_q       <= state_d;
      cl_offset_q   <= cl_offset_d;
      repl_way_oh_q <= repl_way_oh_d;
      inv_q         <= inv_d;
    end
  end

  ///////////////////////////////////////////////////////
  // assertions
  ///////////////////////////////////////////////////////

  //pragma translate_off
`ifndef VERILATOR
  repl_inval0 :
  assert property (
    @(posedge clk_i) disable iff (!rst_ni) cache_wren |-> !(mem_rtrn_i.inv.all | mem_rtrn_i.inv.vld))
  else $fatal(1, "[l1 icache] cannot replace cacheline and invalidate cacheline simultaneously");

  repl_inval1 :
  assert property (
    @(posedge clk_i) disable iff (!rst_ni) (mem_rtrn_i.inv.all | mem_rtrn_i.inv.vld) |-> !cache_wren)
  else $fatal(1, "[l1 icache] cannot replace cacheline and invalidate cacheline simultaneously");

  invalid_state :
  assert property (
    @(posedge clk_i) disable iff (!rst_ni) (state_q inside {FLUSH, IDLE, READ, MISS, KILL_ATRANS, KILL_MISS}))
  else $fatal(1, "[l1 icache] fsm reached an invalid state");

  hot1 :
  assert property (
    @(posedge clk_i) disable iff (!rst_ni) (!inv_en) |-> cache_rden |=> cmp_en_q |-> $onehot0(
      cl_hit
  ))
  else $fatal(1, "[l1 icache] cl_hit signal must be hot1");

  // this is only used for verification!
  logic vld_mirror[ICACHE_NUM_WORDS-1:0][CVA6Cfg.ICACHE_SET_ASSOC-1:0];
  logic [CVA6Cfg.ICACHE_TAG_WIDTH-1:0] tag_mirror[ICACHE_NUM_WORDS-1:0][CVA6Cfg.ICACHE_SET_ASSOC-1:0];
  logic [CVA6Cfg.ICACHE_SET_ASSOC-1:0] tag_write_duplicate_test;

  always_ff @(posedge clk_i or negedge rst_ni) begin : p_mirror
    if (!rst_ni) begin
      vld_mirror <= '{default: '0};
      tag_mirror <= '{default: '0};
    end else begin
      for (int i = 0; i < CVA6Cfg.ICACHE_SET_ASSOC; i++) begin
        if (vld_req[i] & vld_we) begin
          vld_mirror[vld_addr][i] <= vld_wdata[i];
          tag_mirror[vld_addr][i] <= cl_tag_q;
        end
      end
    end
  end

  for (genvar i = 0; i < CVA6Cfg.ICACHE_SET_ASSOC; i++) begin : gen_tag_dupl
    assign tag_write_duplicate_test[i] = (tag_mirror[vld_addr][i] == cl_tag_q) & vld_mirror[vld_addr][i] & (|vld_wdata) & ~icache_spm_ways_i[i];
  end

  tag_write_duplicate :
  assert property (
    @(posedge clk_i) disable iff (!rst_ni) |vld_req |-> vld_we |-> !(|tag_write_duplicate_test))
  else $fatal(1, "[l1 icache] cannot allocate a CL that is already present in the cache");


  initial begin
    // assert wrong parameterizations
    assert (CVA6Cfg.ICACHE_INDEX_WIDTH <= 12)
    else $fatal(1, "[l1 icache] cache index width can be maximum 12bit since VM uses 4kB pages");
  end
`endif
  //pragma translate_on

endmodule  // cva6_icache
