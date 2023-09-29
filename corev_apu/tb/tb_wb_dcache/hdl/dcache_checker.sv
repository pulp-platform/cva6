// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright 2022 PlanV GmbH

// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

`include "tb.svh"

module dcache_checker import ariane_pkg::*; import std_cache_pkg::*; import tb_pkg::*;
  #(
    parameter int unsigned MAX_ROUNDS = 1000,
    parameter int unsigned NR_CPU_PORTS = 3,
    parameter ariane_cfg_t ArianeCfg = ArianeDefaultConfig // contains cacheable regions
    )
  (
   input logic  clk_i,
   input logic  rst_ni,
   output logic check_done_o,
   input        ariane_pkg::dcache_req_i_t[NR_CPU_PORTS-1:0] req_ports_i,
   input        ariane_pkg::dcache_req_o_t[NR_CPU_PORTS-1:0] req_ports_o,
   input        ariane_ace::m2s_nosnoop_t axi_data_o,
   input        ariane_ace::s2m_nosnoop_t axi_data_i,
   input        ariane_ace::m2s_nosnoop_t axi_bypass_o,
   input        ariane_ace::s2m_nosnoop_t axi_bypass_i,
   input        ariane_ace::snoop_req_t snoop_req_i,
   input        ariane_ace::snoop_resp_t snoop_resp_o
   );

  typedef enum  int {RD_REQ, WR_REQ, SNOOP_REQ} type_t;

  typedef struct packed {
    type_t req_type;
    logic [63:0] addr;
    logic [DCACHE_INDEX_WIDTH-1:0] index;
    logic [DCACHE_TAG_WIDTH-1:0]   tag;
    logic [DCACHE_INDEX_WIDTH-DCACHE_BYTE_OFFSET-1:0] mem_idx;
    logic [$clog2(NR_CPU_PORTS)-1:0]                  active_port;
    snoop_pkg::acsnoop_t snoop_type;
  } current_req_t;

  // Cache model

  cache_line_t [DCACHE_NUM_WORDS-1:0][DCACHE_SET_ASSOC-1:0] cache_status;

  // Signals

  logic [DCACHE_SET_ASSOC-1:0] lfsr;
  current_req_t current_req;

  logic [$clog2(DCACHE_SET_ASSOC)-1:0] target_way;
  logic [DCACHE_SET_ASSOC-1:0]         valid_v, dirty_v, shared_v;

  logic aceIsShared, acePassDirty;

  // Helper functions

  function logic[7:0] nextLfsr(logic[7:0] n);
    automatic logic tmp;
    tmp = !(n[7] ^ n[3] ^ n[2] ^ n[1]);
    return {n[6:0], tmp};
  endfunction

  function bit isHit(
                     cache_line_t [DCACHE_NUM_WORDS-1:0][DCACHE_SET_ASSOC-1:0] cache_status,
                     current_req_t req
                     );
    for (int i = 0; i < DCACHE_SET_ASSOC; i++) begin
      if (cache_status[req.mem_idx][i].valid && cache_status[req.mem_idx][i].tag == req.tag)
        return 1'b1;
    end
    return 1'b0;
  endfunction

  function bit isDirty(
                       cache_line_t [DCACHE_NUM_WORDS-1:0][DCACHE_SET_ASSOC-1:0] cache_status,
                       current_req_t req
                     );
    for (int i = 0; i < DCACHE_SET_ASSOC; i++) begin
      if (|cache_status[req.mem_idx][i].dirty && cache_status[req.mem_idx][i].valid && cache_status[req.mem_idx][i].tag == req.tag)
        return 1'b1;
    end
    return 1'b0;
  endfunction

  function bit isShared(
                        cache_line_t [DCACHE_NUM_WORDS-1:0][DCACHE_SET_ASSOC-1:0] cache_status,
                        current_req_t req
                       );
    for (int i = 0; i < DCACHE_SET_ASSOC; i++) begin
      if (cache_status[req.mem_idx][i].shared && cache_status[req.mem_idx][i].valid && cache_status[req.mem_idx][i].tag == req.tag)
        return 1'b1;
    end
    return 1'b0;
  endfunction

  function int getHitWay(
                     cache_line_t [DCACHE_NUM_WORDS-1:0][DCACHE_SET_ASSOC-1:0] cache_status,
                     current_req_t req
                     );
    for (int i = 0; i < DCACHE_SET_ASSOC; i++) begin
      if (cache_status[req.mem_idx][i].valid && cache_status[req.mem_idx][i].tag == req.tag)
        return i;
    end
  endfunction

  function bit isCleanUnique(
                     ariane_ace::m2s_nosnoop_t ace_req
                     );
    if (ace_req.ar.snoop == 4'b1011 && ace_req.ar.bar[0] == 1'b0 && (ace_req.ar.domain == 2'b10 || ace_req.ar.domain == 2'b01))
      return 1'b1;
    else
      return 1'b0;
  endfunction

  function bit isReadShared(
                            ariane_ace::m2s_nosnoop_t ace_req
                            );
    if (ace_req.ar.snoop == 4'b0001 && ace_req.ar.bar[0] == 1'b0 && (ace_req.ar.domain == 2'b01 || ace_req.ar.domain == 2'b10))
      return 1'b1;
    else
      return 1'b0;
  endfunction

  function bit isReadOnce(
                            ariane_ace::m2s_nosnoop_t ace_req
                            );
    if (ace_req.ar.snoop == 4'b0000 && ace_req.ar.bar[0] == 1'b0 && (ace_req.ar.domain == 2'b01 || ace_req.ar.domain == 2'b10))
      return 1'b1;
    else
      return 1'b0;
  endfunction

  function bit isReadUnique(
                          ariane_ace::m2s_nosnoop_t ace_req
                          );
    if (ace_req.ar.snoop == 4'b0111 && ace_req.ar.bar[0] == 1'b0 && (ace_req.ar.domain == 2'b01 || ace_req.ar.domain == 2'b10))
      return 1'b1;
    else
      return 1'b0;
  endfunction

  function bit isReadNoSnoop(
                            ariane_ace::m2s_nosnoop_t ace_req
                            );
    if (ace_req.ar.snoop == 4'b0000 && ace_req.ar.bar[0] == 1'b0 && (ace_req.ar.domain == 2'b00 || ace_req.ar.domain == 2'b11))
      return 1'b1;
    else
      return 1'b0;
  endfunction

  function bit isWriteBack(
                            ariane_ace::m2s_nosnoop_t ace_req
                            );
    if (ace_req.aw.snoop == 3'b011 && ace_req.aw.bar[0] == 1'b0 && (ace_req.aw.domain == 2'b00 || ace_req.aw.domain == 2'b01 || ace_req.aw.domain == 2'b10))
      return 1'b1;
    else
      return 1'b0;
  endfunction

  function bit isWriteUnique(
                           ariane_ace::m2s_nosnoop_t ace_req
                           );
    if (ace_req.aw.snoop == 3'b000 && ace_req.aw.bar[0] == 1'b0 && (ace_req.aw.domain == 2'b01 || ace_req.aw.domain == 2'b10))
      return 1'b1;
    else
      return 1'b0;
  endfunction

  function bit isWriteNoSnoop(
                           ariane_ace::m2s_nosnoop_t ace_req
                           );
    if (ace_req.aw.snoop == 3'b000 && ace_req.aw.bar[0] == 1'b0 && (ace_req.aw.domain == 2'b00 || ace_req.aw.domain == 2'b11))
      return 1'b1;
    else
      return 1'b0;
  endfunction

  function bit mustEvict(
                         cache_line_t [DCACHE_NUM_WORDS-1:0][DCACHE_SET_ASSOC-1:0] cache_status,
                         current_req_t req
                         );
    automatic logic valid = 1'b1;
    for (int i = 0; i < DCACHE_SET_ASSOC; i++) begin
      valid = valid & cache_status[req.mem_idx][i].valid;
    end
    if (!isHit(cache_status, req) && valid == 1'b1 && cache_status[req.mem_idx][lfsr[$clog2(DCACHE_SET_ASSOC)-1:0]].dirty != '0)
      return 1'b1;
    else
      return 1'b0;
  endfunction

  // Coverage

  // Each cache block used at least once
  bit [DCACHE_NUM_WORDS-1:0][DCACHE_SET_ASSOC-1:0] cache_valid_bin;
  // Read transactions targeting every way and every combination of valid/dirty/shared
  // it is impossible to have shared or dirty flags when the cache block is invalid
  bit [DCACHE_SET_ASSOC-1:0][7:0]                 read_bin = {DCACHE_SET_ASSOC{8'b00001110}};
  // Write transactions targeting every way and every combination of valid/dirty/shared
  // it is impossible to have shared or dirty flags when the cache block is invalid
  bit [DCACHE_SET_ASSOC-1:0][7:0]                 write_bin = {DCACHE_SET_ASSOC{8'b00001110}};
  // Snoop transactions targetin every way and every combination of valid/dirty/shared
  // it is impossible to have shared or dirty flags when the cache block is invalid
  bit [DCACHE_SET_ASSOC-1:0][7:0]                 snoop_bin = {DCACHE_SET_ASSOC{8'b00001110}};

  task automatic updateBuckets();
    logic [2:0] vds; // valid/dirty/shared
    vds = {cache_status[current_req.mem_idx][target_way].valid,
           |cache_status[current_req.mem_idx][target_way].dirty,
           cache_status[current_req.mem_idx][target_way].shared};

    case (current_req.req_type)
      SNOOP_REQ: begin
        for (int i = 0; i < DCACHE_SET_ASSOC; i++) begin
          if (valid_v[i] && cache_status[current_req.mem_idx][i].tag == current_req.tag) begin
            snoop_bin[i][vds] = 1'b1;
            break;
          end
        end
      end
      RD_REQ: begin
        read_bin[target_way][vds] = 1'b1;
      end
      WR_REQ: begin
        write_bin[target_way][vds] = 1'b1;
      end
    endcase
    if (current_req.req_type == RD_REQ || current_req.req_type == WR_REQ) begin
      if (is_inside_cacheable_regions(ArianeCfg, current_req.addr)) begin
        cache_valid_bin[current_req.mem_idx][target_way] = 1'b1;
      end
    end
  endtask

  task reportCoverage(
                      output bit finish
                      );
    automatic int unsigned                 total_items;
    automatic int unsigned                 covered_items;
    automatic real coverage;

    finish = 1'b0;
    covered_items = 0;

    total_items = DCACHE_NUM_WORDS * DCACHE_SET_ASSOC + 3*DCACHE_SET_ASSOC*8;
    for (int i = 0; i < DCACHE_NUM_WORDS; i++) begin
      for (int j = 0; j < DCACHE_SET_ASSOC; j++) begin
        if (cache_valid_bin[i][j])
          covered_items = covered_items + 1;
      end
    end

    for (int i = 0; i < DCACHE_SET_ASSOC; i++) begin
      for (int j = 0; j < 8; j++) begin
        if (read_bin[i][j])
          covered_items = covered_items + 1;
        if (write_bin[i][j])
          covered_items = covered_items + 1;
        if (snoop_bin[i][j])
          covered_items = covered_items + 1;
      end
    end
    coverage = 100 * real'(covered_items)/real'(total_items);
    //$display("Coverage = %0.2f %%", coverage);
    if (coverage == 100)
      finish = 1'b1;

  endtask

  int total_requests = 0;
  int snoop_requests = 0;
  int read_requests = 0;
  int write_requests = 0;
  int cacheable_read_requests = 0;
  int cacheable_write_requests = 0;
  int shareable_read_requests = 0;
  int shareable_write_requests = 0;
  int valid_read_requests = 0;
  int valid_write_requests = 0;
  int dirty_read_requests = 0;
  int dirty_write_requests = 0;
  int shared_read_requests = 0;
  int shared_write_requests = 0;
  int cleaninvalid_snoop_requests = 0;
  int readunique_snoop_requests = 0;
  int readshared_snoop_requests = 0;
  int readonce_snoop_requests = 0;
  int valid_cleaninvalid_snoop_requests = 0;
  int valid_readunique_snoop_requests = 0;
  int valid_readshared_snoop_requests = 0;
  int valid_readonce_snoop_requests = 0;
  int dirty_cleaninvalid_snoop_requests = 0;
  int dirty_readunique_snoop_requests = 0;
  int dirty_readshared_snoop_requests = 0;
  int dirty_readonce_snoop_requests = 0;
  int shared_cleaninvalid_snoop_requests = 0;
  int shared_readunique_snoop_requests = 0;
  int shared_readshared_snoop_requests = 0;
  int shared_readonce_snoop_requests = 0;

  task updateTestStatistics();
    total_requests = total_requests + 1;
    if (current_req.req_type == RD_REQ) begin
      read_requests = read_requests + 1;
      if (is_inside_cacheable_regions(ArianeCfg, current_req.addr)) begin
        cacheable_read_requests = cacheable_read_requests + 1;
        if (isHit(cache_status, current_req))
          valid_read_requests = valid_read_requests + 1;
        if (isDirty(cache_status, current_req))
          dirty_read_requests = dirty_read_requests + 1;
        if (isShared(cache_status, current_req))
          shared_read_requests = shared_read_requests + 1;
      end
      if (is_inside_shareable_regions(ArianeCfg, current_req.addr))
        shareable_read_requests = shareable_read_requests + 1;
    end
    else if (current_req.req_type == WR_REQ) begin
      write_requests = write_requests + 1;
      if (is_inside_cacheable_regions(ArianeCfg, current_req.addr)) begin
        cacheable_write_requests = cacheable_write_requests + 1;
        if (isHit(cache_status, current_req))
          valid_write_requests = valid_write_requests + 1;
        if (isDirty(cache_status, current_req))
          dirty_write_requests = dirty_write_requests + 1;
        if (isShared(cache_status, current_req))
          shared_write_requests = shared_write_requests + 1;
      end
      if (is_inside_shareable_regions(ArianeCfg, current_req.addr))
        shareable_write_requests = shareable_write_requests + 1;
    end
    else if (current_req.req_type == SNOOP_REQ) begin
      snoop_requests = snoop_requests + 1;
      case (current_req.snoop_type)
        snoop_pkg::READ_SHARED: begin
          readshared_snoop_requests = readshared_snoop_requests + 1;
          if (isHit(cache_status, current_req))
            valid_readshared_snoop_requests = valid_readshared_snoop_requests + 1;
          if (isDirty(cache_status, current_req))
            dirty_readshared_snoop_requests = dirty_readshared_snoop_requests + 1;
          if (isShared(cache_status, current_req))
            shared_readshared_snoop_requests = shared_readshared_snoop_requests + 1;
        end
        snoop_pkg::READ_UNIQUE: begin
          readunique_snoop_requests = readunique_snoop_requests + 1;
          if (isHit(cache_status, current_req))
            valid_readunique_snoop_requests = valid_readunique_snoop_requests + 1;
          if (isDirty(cache_status, current_req))
            dirty_readunique_snoop_requests = dirty_readunique_snoop_requests + 1;
          if (isShared(cache_status, current_req))
            shared_readunique_snoop_requests = shared_readunique_snoop_requests + 1;
        end
        snoop_pkg::CLEAN_INVALID: begin
          cleaninvalid_snoop_requests = cleaninvalid_snoop_requests + 1;
          if (isHit(cache_status, current_req))
            valid_cleaninvalid_snoop_requests = valid_cleaninvalid_snoop_requests + 1;
          if (isDirty(cache_status, current_req))
            dirty_cleaninvalid_snoop_requests = dirty_cleaninvalid_snoop_requests + 1;
          if (isShared(cache_status, current_req))
            shared_cleaninvalid_snoop_requests = shared_cleaninvalid_snoop_requests + 1;
        end
        snoop_pkg::READ_ONCE: begin
          readonce_snoop_requests = readonce_snoop_requests + 1;
          if (isHit(cache_status, current_req))
            valid_readonce_snoop_requests = valid_readonce_snoop_requests + 1;
          if (isDirty(cache_status, current_req))
            dirty_readonce_snoop_requests = dirty_readonce_snoop_requests + 1;
          if (isShared(cache_status, current_req))
            shared_readonce_snoop_requests = shared_readonce_snoop_requests + 1;
        end
      endcase
    end
  endtask

  task reportStatistics();
    $display("Total generated requests: %d", total_requests);

    $display("\tSnoop requests: %d", snoop_requests);
    $display("\t\tCleanInvalid requests: %d", cleaninvalid_snoop_requests);
    $display("\t\t\tCleanInvalid requests targeting valid cache blocks: %d",   valid_cleaninvalid_snoop_requests);
    $display("\t\t\tCleanInvalid requests targeting dirty cache blocks: %d",   dirty_cleaninvalid_snoop_requests);
    $display("\t\t\tCleanInvalid requests targeting shared cache blocks: %d", shared_cleaninvalid_snoop_requests);
    $display("\t\tReadUnique requests: %d", readunique_snoop_requests);
    $display("\t\t\tReadUnique requests targeting valid cache blocks: %d",   valid_readunique_snoop_requests);
    $display("\t\t\tReadUnique requests targeting dirty cache blocks: %d",   dirty_readunique_snoop_requests);
    $display("\t\t\tReadUnique requests targeting shared cache blocks: %d", shared_readunique_snoop_requests);
    $display("\t\tReadShared requests: %d", readshared_snoop_requests);
    $display("\t\t\tReadShared requests targeting valid cache blocks: %d",   valid_readshared_snoop_requests);
    $display("\t\t\tReadShared requests targeting dirty cache blocks: %d",   dirty_readshared_snoop_requests);
    $display("\t\t\tReadShared requests targeting shared cache blocks: %d", shared_readshared_snoop_requests);
    $display("\t\tReadOnce requests: %d", readonce_snoop_requests);
    $display("\t\t\tReadOnce requests targeting valid cache blocks: %d",   valid_readonce_snoop_requests);
    $display("\t\t\tReadOnce requests targeting dirty cache blocks: %d",   dirty_readonce_snoop_requests);
    $display("\t\t\tReadOnce requests targeting shared cache blocks: %d", shared_readonce_snoop_requests);

    $display("\tRead requests: %d", read_requests);
    $display("\t\tRead requests targeting cacheable locations: %d", cacheable_read_requests);
    $display("\t\tRead requests targeting shareable locations: %d", shareable_read_requests);
    $display("\t\tRead requests targeting valid cache blocks: %d",  valid_read_requests);
    $display("\t\tRead requests targeting dirty cache blocks: %d",  dirty_read_requests);
    $display("\t\tRead requests targeting shared cache blocks: %d", shared_read_requests);
    $display("\tWrite requests: %d", write_requests);
    $display("\t\tWrite requests targeting cacheable locations: %d", cacheable_write_requests);
    $display("\t\tWrite requests targeting shareable locations: %d", shareable_write_requests);
    $display("\t\tWrite requests targeting valid cache blocks: %d",   valid_write_requests);
    $display("\t\tWrite requests targeting dirty cache blocks: %d",   dirty_write_requests);
    $display("\t\tWrite requests targeting shared cache blocks: %d", shared_write_requests);
  endtask

  // Helper tasks

  generate
    genvar                             i;
    for (i = 0; i < DCACHE_SET_ASSOC; i++) begin
      assign valid_v[i] = cache_status[current_req.mem_idx][i].valid;
      assign dirty_v[i] = |cache_status[current_req.mem_idx][i].dirty;
      assign shared_v[i] = cache_status[current_req.mem_idx][i].shared;
    end
  endgenerate

  task automatic updateCache();
    if (current_req.req_type == SNOOP_REQ) begin
      // look for the right tag
      for (int i = 0; i < DCACHE_SET_ASSOC; i++) begin
        if (valid_v[i] && cache_status[current_req.mem_idx][i].tag == current_req.tag) begin
          case (current_req.snoop_type)
            snoop_pkg::READ_SHARED: begin
              cache_status[current_req.mem_idx][i].shared = 1'b1;
            end
            snoop_pkg::READ_UNIQUE: begin
              cache_status[current_req.mem_idx][i].shared = 1'b0;
              cache_status[current_req.mem_idx][i].valid = 1'b0;
              cache_status[current_req.mem_idx][i].dirty = '0;
            end
            snoop_pkg::CLEAN_INVALID: begin
              cache_status[current_req.mem_idx][i].shared = 1'b0;
              cache_status[current_req.mem_idx][i].valid = 1'b0;
              cache_status[current_req.mem_idx][i].dirty = '0;
            end
          endcase
          break;
        end
      end
    end
    // read or write request
    else begin
      // cache miss
      if (!isHit(cache_status, current_req)) begin
        // all ways occupied
        if (&valid_v) begin
          target_way = lfsr[$clog2(DCACHE_SET_ASSOC)-1:0];
          cache_status[current_req.mem_idx][target_way].tag = current_req.tag;
          if (current_req.req_type == WR_REQ) begin
            cache_status[current_req.mem_idx][target_way].dirty = '1;
            cache_status[current_req.mem_idx][target_way].shared = 1'b0;
          end
          else begin
            cache_status[current_req.mem_idx][target_way].dirty = acePassDirty ? '1 : '0;
            cache_status[current_req.mem_idx][target_way].shared = aceIsShared;
          end
          lfsr = nextLfsr(lfsr);
        end
        // there is an empty way
        else begin
          target_way = one_hot_to_bin(get_victim_cl(~valid_v));
          cache_status[current_req.mem_idx][target_way].tag = current_req.tag;
          cache_status[current_req.mem_idx][target_way].valid = 1'b1;
          if (current_req.req_type == WR_REQ) begin
            cache_status[current_req.mem_idx][target_way].dirty = '1;
            cache_status[current_req.mem_idx][target_way].shared = 1'b0;
          end
          else begin
            cache_status[current_req.mem_idx][target_way].dirty = acePassDirty ? '1 : '0;
            cache_status[current_req.mem_idx][target_way].shared = aceIsShared;
          end
        end
      end
      // cache hit
      else begin
        target_way = getHitWay(cache_status, current_req);
        if (current_req.req_type == WR_REQ) begin
          cache_status[current_req.mem_idx][target_way].dirty = 1'b1;
          cache_status[current_req.mem_idx][target_way].shared = 1'b0;
        end
      end
    end
  endtask

  task automatic checkCache (
                             output bit OK
                             );


    typedef vldrty_t [DCACHE_SET_ASSOC-1:0]  vld_t;
    typedef vld_t                            vld_sram_t  [DCACHE_NUM_WORDS-1:0];
    vld_sram_t vld_sram;
    vld_sram = i_dut.valid_dirty_sram.i_tc_sram.sram;

    OK = 1'b1;

    // check the target_way
    // only check that any bit is set
    if (|cache_status[current_req.mem_idx][target_way].dirty != |vld_sram[current_req.mem_idx][target_way].dirty) begin
      OK = 1'b0;
      $error("Cache mismatch index %h tag %h way %h - dirty bit: expected %d, actual %d", current_req.index, current_req.tag, target_way, |cache_status[current_req.mem_idx][target_way].dirty, |vld_sram[current_req.mem_idx][target_way].dirty);
    end
    if (cache_status[current_req.mem_idx][target_way].valid != vld_sram[current_req.mem_idx][target_way].valid) begin
      OK = 1'b0;
      $error("Cache mismatch index %h tag %h way %h - valid bit: expected %d, actual %d", current_req.index, current_req.tag, target_way, cache_status[current_req.mem_idx][target_way].valid, vld_sram[current_req.mem_idx][target_way].valid);
    end
    if (cache_status[current_req.mem_idx][target_way].shared != vld_sram[current_req.mem_idx][target_way].shared) begin
      OK = 1'b0;
      $error("Cache mismatch index %h tag %h way %h - shared bit: expected %d, actual %d", current_req.index, current_req.tag, target_way, cache_status[current_req.mem_idx][target_way].shared, vld_sram[current_req.mem_idx][target_way].shared);
    end
    if (cache_status[current_req.mem_idx][0].tag != i_dut.sram_block[0].tag_sram.i_tc_sram.sram[current_req.mem_idx][47:0]) begin
      OK = 1'b0;
      $error("Cache mismatch index %h tag %h way %h - tag: expected %h, actual %h", current_req.index, current_req.tag, target_way, cache_status[current_req.mem_idx][target_way].tag, i_dut.sram_block[0].tag_sram.i_tc_sram.sram[current_req.mem_idx][47:0]);
    end
    if (cache_status[current_req.mem_idx][1].tag != i_dut.sram_block[1].tag_sram.i_tc_sram.sram[current_req.mem_idx][47:0]) begin
      OK = 1'b0;
      $error("Cache mismatch index %h tag %h way %h - tag: expected %h, actual %h", current_req.index, current_req.tag, target_way, cache_status[current_req.mem_idx][target_way].tag, i_dut.sram_block[1].tag_sram.i_tc_sram.sram[current_req.mem_idx][47:0]);
    end
    if (cache_status[current_req.mem_idx][2].tag != i_dut.sram_block[2].tag_sram.i_tc_sram.sram[current_req.mem_idx][47:0]) begin
      OK = 1'b0;
      $error("Cache mismatch index %h tag %h way %h - tag: expected %h, actual %h", current_req.index, current_req.tag, target_way, cache_status[current_req.mem_idx][target_way].tag, i_dut.sram_block[2].tag_sram.i_tc_sram.sram[current_req.mem_idx][47:0]);
    end
    if (cache_status[current_req.mem_idx][3].tag != i_dut.sram_block[3].tag_sram.i_tc_sram.sram[current_req.mem_idx][47:0]) begin
      OK = 1'b0;
      $error("Cache mismatch index %h tag %h way %h - tag: expected %h, actual %h", current_req.index, current_req.tag, target_way, cache_status[current_req.mem_idx][target_way].tag, i_dut.sram_block[3].tag_sram.i_tc_sram.sram[current_req.mem_idx][47:0]);
    end
    if (cache_status[current_req.mem_idx][4].tag != i_dut.sram_block[4].tag_sram.i_tc_sram.sram[current_req.mem_idx][47:0]) begin
      OK = 1'b0;
      $error("Cache mismatch index %h tag %h way %h - tag: expected %h, actual %h", current_req.index, current_req.tag, target_way, cache_status[current_req.mem_idx][target_way].tag, i_dut.sram_block[4].tag_sram.i_tc_sram.sram[current_req.mem_idx][47:0]);
    end
    if (cache_status[current_req.mem_idx][5].tag != i_dut.sram_block[5].tag_sram.i_tc_sram.sram[current_req.mem_idx][47:0]) begin
      OK = 1'b0;
      $error("Cache mismatch index %h tag %h way %h - tag: expected %h, actual %h", current_req.index, current_req.tag, target_way, cache_status[current_req.mem_idx][target_way].tag, i_dut.sram_block[5].tag_sram.i_tc_sram.sram[current_req.mem_idx][47:0]);
    end
    if (cache_status[current_req.mem_idx][6].tag != i_dut.sram_block[6].tag_sram.i_tc_sram.sram[current_req.mem_idx][47:0]) begin
      OK = 1'b0;
      $error("Cache mismatch index %h tag %h way %h - tag: expected %h, actual %h", current_req.index, current_req.tag, target_way, cache_status[current_req.mem_idx][target_way].tag, i_dut.sram_block[6].tag_sram.i_tc_sram.sram[current_req.mem_idx][47:0]);
    end
    if (cache_status[current_req.mem_idx][7].tag != i_dut.sram_block[7].tag_sram.i_tc_sram.sram[current_req.mem_idx][47:0]) begin
      OK = 1'b0;
      $error("Cache mismatch index %h tag %h way %h - tag: expected %h, actual %h", current_req.index, current_req.tag, target_way, cache_status[current_req.mem_idx][target_way].tag, i_dut.sram_block[7].tag_sram.i_tc_sram.sram[current_req.mem_idx][47:0]);
    end

  endtask

  task automatic checkCRResp (
                              output bit OK
                             );
    OK = 1'b1;

    if (current_req.snoop_type != snoop_pkg::CLEAN_INVALID &&
        current_req.snoop_type != snoop_pkg::READ_ONCE &&
        current_req.snoop_type != snoop_pkg::READ_UNIQUE &&
        current_req.snoop_type != snoop_pkg::READ_SHARED &&
        snoop_resp_o.cr_resp.error == 1'b0) begin
      $error("CR.resp.error expected for snoop request %s", current_req.snoop_type);
      OK = 1'b0;
    end

    if (current_req.snoop_type == snoop_pkg::CLEAN_INVALID) begin

      if (snoop_resp_o.cr_resp.isShared == 1'b1 && snoop_resp_o.cr_resp.error == 1'b0) begin
        $error("CR.resp.isShared mismatch: expected 0, actual 1");
        OK = 1'b0;
      end

      if(isDirty(cache_status, current_req) != snoop_resp_o.cr_resp.passDirty && snoop_resp_o.cr_resp.error == 1'b0) begin
        $error("CR.resp.passDirty mismatch: expected %h, actual %h", isDirty(cache_status, current_req), snoop_resp_o.cr_resp.passDirty);
        OK = 1'b0;
      end

      if(isDirty(cache_status, current_req) != snoop_resp_o.cr_resp.dataTransfer && snoop_resp_o.cr_resp.error == 1'b0) begin
        $error("CR.resp.dataTransfer mismatch: expected %h, actual %h", isDirty(cache_status, current_req), snoop_resp_o.cr_resp.dataTransfer);
        OK = 1'b0;
      end

    end else if (current_req.snoop_type == snoop_pkg::READ_SHARED) begin

      if(isHit(cache_status, current_req) != snoop_resp_o.cr_resp.isShared && snoop_resp_o.cr_resp.error == 1'b0) begin
        $error("CR.resp.isShared mismatch: expected %h, actual %h", isHit(cache_status, current_req), snoop_resp_o.cr_resp.isShared);
        OK = 1'b0;
      end

      if (snoop_resp_o.cr_resp.passDirty == 1'b1 && snoop_resp_o.cr_resp.error == 1'b0) begin
        $error("CR.resp.passDirty mismatch: expected 0, actual 1");
        OK = 1'b0;
      end

      if(isHit(cache_status, current_req) != snoop_resp_o.cr_resp.dataTransfer && snoop_resp_o.cr_resp.error == 1'b0) begin
        $error("CR.resp.transferData mismatch: expected %h, actual %h", isHit(cache_status, current_req), snoop_resp_o.cr_resp.dataTransfer);
        OK = 1'b0;
      end
    end else if (current_req.snoop_type == snoop_pkg::READ_ONCE) begin

      if (isShared(cache_status, current_req) != snoop_resp_o.cr_resp.isShared && snoop_resp_o.cr_resp.error == 1'b0) begin
        $error("CR.resp.isShared mismatch: expected %h, actual %h", isShared(cache_status, current_req), snoop_resp_o.cr_resp.isShared);
        OK = 1'b0;
      end

      if (snoop_resp_o.cr_resp.passDirty == 1'b1 && snoop_resp_o.cr_resp.error == 1'b0) begin
        $error("CR.resp.passDirty mismatch: expected 0, actual 1");
        OK = 1'b0;
      end

      if(isHit(cache_status, current_req) != snoop_resp_o.cr_resp.dataTransfer && snoop_resp_o.cr_resp.error == 1'b0) begin
        $error("CR.resp.transferData mismatch: expected %h, actual %h", isHit(cache_status, current_req), snoop_resp_o.cr_resp.dataTransfer);
        OK = 1'b0;
      end

    end else begin // READ_UNIQUE

      if (snoop_resp_o.cr_resp.isShared == 1'b1 && snoop_resp_o.cr_resp.error == 1'b0) begin
        $error("CR.resp.isShared mismatch: expected 0, actual 1");
        OK = 1'b0;
      end

      if(isDirty(cache_status, current_req) != snoop_resp_o.cr_resp.passDirty && snoop_resp_o.cr_resp.error == 1'b0) begin
        $error("CR.resp.passDirty mismatch: expected %h, actual %h", isDirty(cache_status, current_req), snoop_resp_o.cr_resp.passDirty);
        OK = 1'b0;
      end

      if(isHit(cache_status, current_req) != snoop_resp_o.cr_resp.dataTransfer && snoop_resp_o.cr_resp.error == 1'b0) begin
        $error("CR.resp.transferData mismatch: expected %h, actual %h", isHit(cache_status, current_req), snoop_resp_o.cr_resp.dataTransfer);
        OK = 1'b0;
      end
    end

  endtask

  // Main loop

  assign current_req.index = current_req.addr[DCACHE_INDEX_WIDTH-1:0];
  assign current_req.tag = current_req.addr[DCACHE_TAG_WIDTH+DCACHE_INDEX_WIDTH-1:DCACHE_INDEX_WIDTH];
  assign current_req.mem_idx = current_req.addr[DCACHE_INDEX_WIDTH-1:DCACHE_BYTE_OFFSET];

  logic start_transaction;

  always_comb begin
    start_transaction = 1'b0;
    if (snoop_req_i.ac_valid & snoop_resp_o.ac_ready)
      start_transaction = 1'b1;
    for (int i = 0; i < NR_CPU_PORTS; i++)
      if (req_ports_i[i].data_req)
        start_transaction = 1'b1;
  end

  always_latch begin
    if (start_transaction) begin
      if (snoop_req_i.ac_valid & snoop_resp_o.ac_ready) begin
        current_req.active_port = 0;
        current_req.req_type = SNOOP_REQ;
        current_req.addr = snoop_req_i.ac.addr;
        current_req.snoop_type = snoop_req_i.ac.snoop;
      end
      else begin
        for (int i = 0; i < NR_CPU_PORTS; i++) begin
          if (req_ports_i[i].data_req) begin
            current_req.active_port = i;
            if (req_ports_i[i].data_we)
              current_req.req_type = WR_REQ;
            else
              current_req.req_type = RD_REQ;
            current_req.addr = {req_ports_i[i].address_tag, req_ports_i[i].address_index};
            current_req.snoop_type = 0;
          end
        end
      end
    end
  end

  initial begin
    automatic int unsigned round = 0;
    bit checkOK;
    bit finish;

    cache_status = '0;
    lfsr = '0;

    forever begin
      check_done_o = 1'b0;
      aceIsShared = 1'b0;
      acePassDirty = 1'b0;

      `WAIT_SIG(clk_i, start_transaction)

      if (current_req.req_type == SNOOP_REQ) begin
        // wait for the response
        if(~snoop_resp_o.cr_valid) begin
          `WAIT_SIG(clk_i, snoop_resp_o.cr_valid)
          checkCRResp(checkOK);
        end
        // expect the data
        if (isHit(cache_status, current_req) && (current_req.snoop_type == snoop_pkg::READ_UNIQUE || 
                                                 current_req.snoop_type == snoop_pkg::READ_ONCE || 
                                                 current_req.snoop_type == snoop_pkg::READ_SHARED)) begin
          `WAIT_SIG(clk_i, snoop_resp_o.cd.last)
        end else if (isDirty(cache_status, current_req) && (current_req.snoop_type == snoop_pkg::CLEAN_INVALID)) begin
          `WAIT_SIG(clk_i, snoop_resp_o.cd.last)
        end
      end
      else begin
        // bypass
        if (!is_inside_cacheable_regions(ArianeCfg, current_req.addr)) begin
          if (current_req.req_type == WR_REQ) begin
            `WAIT_SIG(clk_i, axi_bypass_o.aw_valid)
            if (is_inside_shareable_regions(ArianeCfg, current_req.addr)) begin
              if (!isWriteUnique(axi_bypass_o))
                $error("Error WRITE_UNIQUE request expected");
            end
            else begin
              if (!isWriteNoSnoop(axi_bypass_o))
                $error("Error WRITE_NO_SNOOP request expected");
            end
            `WAIT_SIG(clk_i, axi_bypass_i.b_valid)
          end
          else begin
            `WAIT_SIG(clk_i, axi_bypass_o.ar_valid)
            if (is_inside_shareable_regions(ArianeCfg, current_req.addr)) begin
              if (!isReadOnce(axi_bypass_o))
                $error("Error READ_ONCE request expected");
            end
            else begin
              if (!isReadNoSnoop(axi_bypass_o))
                $error("Error READ_NO_SNOOP request expected");
            end
            `WAIT_SIG(clk_i, axi_bypass_i.r.last)
          end
        end
        // cacheable
        else begin
          // Cache miss
          if (!isHit(cache_status, current_req)) begin
            // check if eviction is necessary
            if (mustEvict(cache_status, current_req)) begin
              `WAIT_SIG(clk_i, axi_data_o.aw_valid)
              if (!isWriteBack(axi_data_o))
                $error("Error WRITEBACK request expected");
            end
            fork
              begin
                `WAIT_SIG(clk_i, axi_data_o.ar_valid)
                if (current_req.req_type == WR_REQ) begin
                  if (!isReadUnique(axi_data_o))
                    $error("Error READ_UNIQUE request expected");
                end
                else begin
                  if (!isReadShared(axi_data_o))
                    $error("Error READ_SHARED request expected");
                end
                `WAIT_SIG(clk_i, axi_data_i.r.last)
                acePassDirty = axi_data_i.r.resp[2];
                aceIsShared = axi_data_i.r.resp[3];
              end
              begin
                if (current_req.req_type == WR_REQ)
                  `WAIT_SIG(clk_i, req_ports_o[current_req.active_port].data_gnt)
                else
                  `WAIT_SIG(clk_i, req_ports_o[current_req.active_port].data_rvalid)
              end
            join
          end
          else begin
            // in case of a cache hit and write request, wait for a CleanUnique transaction
            if (current_req.req_type == WR_REQ) begin
              if (isShared(cache_status, current_req)) begin
                `WAIT_SIG(clk_i, axi_data_o.ar_valid)
                if (!isCleanUnique(axi_data_o))
                  $error("Error CLEAN_UNIQUE expected");
              end
              `WAIT_SIG(clk_i, req_ports_o[current_req.active_port].data_gnt)
            end
            // otherwise wait only for the response from the port
            else begin
              fork
                begin
                  if (~req_ports_o[current_req.active_port].data_rvalid) begin
                    `WAIT_SIG(clk_i, req_ports_o[current_req.active_port].data_rvalid)
                  end
                end
                begin
                  `WAIT_SIG(clk_i, axi_data_o.ar_valid)
                  $error("AR_VALID error, expected 0");
                end
              join_any
              disable fork;
            end
          end
        end
      end

      // wait for acePassDirty and aceIsShared to stabilize
      `WAIT_CYC(clk_i, 1)

      updateTestStatistics();

      if (is_inside_cacheable_regions(ArianeCfg, current_req.addr))
        updateCache();

      // the actual cache needs 2 more cycles to be updated
      `WAIT_CYC(clk_i, 2)

      checkCache(checkOK);

      updateBuckets();
      reportCoverage(finish);

      if (finish) begin
        $display("Simulation end");
        reportStatistics();
        $finish();
      end
      else if (round == MAX_ROUNDS-1) begin
        $warning("Simulation end - Maximum number of rounds reached");
        $finish();
      end
      else begin
        round = round + 1;
      end

      `WAIT_CYC(clk_i, 2)
      check_done_o = 1'b1;
      `WAIT_CYC(clk_i, 1)
    end
  end

endmodule
