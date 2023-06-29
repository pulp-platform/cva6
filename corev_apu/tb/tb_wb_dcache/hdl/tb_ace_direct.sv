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
`include "ace/assign.svh"

module tb_ace_direct import ariane_pkg::*; import std_cache_pkg::*; import tb_pkg::*; import snoop_pkg::*; #()();

  // leave this
  timeunit 1ps;
  timeprecision 1ps;

  localparam int timeout = 1000;

  // memory configuration (64bit words)

  // 0 ~ 1/16 MemSize   => non-cacheable, non-shareable
  // 1/16 ~ 1/8 MemSize => non-cacheable, shareable
  // 1/8 ~ 9/16 MemSize => cacheable, shareable
  // 9/16 ~ end MemSize => cacheable, non-shareable

  parameter MemBytes          = 2**DCACHE_INDEX_WIDTH * 4 * 4; //2**DCACHE_INDEX_WIDTH * 4 * 32;
  parameter MemWords          = MemBytes>>3;

  // cacheable portion
  parameter logic [63:0] CachedAddrBeg = MemBytes>>3; // 1/8 is non-cacheable
  parameter logic [63:0] CachedAddrEnd = 64'hFFFF_FFFF_FFFF_FFFF;
  // shareable portion
  parameter logic [63:0] SharedAddrBeg = MemBytes>>4; // 1/16 is the beginning of the shareable region
  parameter logic [63:0] SharedAddrEnd = 9*MemBytes>>4-1; // 9/16 is the end of the shareable region

  localparam ariane_cfg_t ArianeCfg = '{
    RASDepth: 2,
    BTBEntries: 32,
    BHTEntries: 128,
    // idempotent region
    NrNonIdempotentRules:  0,
    NonIdempotentAddrBase: {64'b0},
    NonIdempotentLength:   {64'b0},
    // executable region
    NrExecuteRegionRules:  0,
    ExecuteRegionAddrBase: {64'h0},
    ExecuteRegionLength:   {64'h0},
    // cached region
    NrCachedRegionRules:   1,
    CachedRegionAddrBase:  {CachedAddrBeg},//1/8th of the memory is NC
    CachedRegionLength:    {CachedAddrEnd-CachedAddrBeg+64'b1},
    // shared region
    NrSharedRegionRules:   1,
    SharedRegionAddrBase:  {SharedAddrBeg},
    SharedRegionLength:    {SharedAddrEnd-SharedAddrBeg+64'b1},
    // cache config
    AxiCompliant:          1'b1,
    SwapEndianess:         1'b0,
    // debug
    DmBaseAddress:         64'h0,
    NrPMPEntries:          0
  };

  // ID width of the Full AXI slave port, master port has ID `AxiIdWidthFull + 32'd1`
  parameter int unsigned AxiIdWidth   = 32'd6;
  // Address width of the full AXI bus
  parameter int unsigned AxiAddrWidth = 32'd64;
  // Data width of the full AXI bus
  parameter int unsigned AxiDataWidth = 32'd64;
  localparam int unsigned AxiUserWidth = 32'd1;

  // Functions

  function logic[DCACHE_TAG_WIDTH-1:0] addr2tag(logic[63:0] addr);
    return addr[DCACHE_TAG_WIDTH+DCACHE_INDEX_WIDTH-1:DCACHE_INDEX_WIDTH];
  endfunction

  function logic[DCACHE_INDEX_WIDTH-1:0] addr2index(logic[63:0] addr);
    return addr[DCACHE_INDEX_WIDTH-1:0];
  endfunction

  // DUT signal declarations

  logic                           enable_i;
  logic                           flush_i;
  logic                           flush_ack_o;
  logic                           miss_o;
  amo_req_t                       amo_req_i;
  amo_resp_t                      amo_resp_o;
  dcache_req_i_t [2:0]            req_ports_i;
  dcache_req_o_t [2:0]            req_ports_o;
  ariane_ace::m2s_nosnoop_t       axi_data_o;
  ariane_ace::s2m_nosnoop_t       axi_data_i;
  ariane_ace::m2s_nosnoop_t       axi_bypass_o;
  ariane_ace::s2m_nosnoop_t       axi_bypass_i;
  ariane_ace::snoop_resp_t        snoop_port_o;
  ariane_ace::snoop_req_t         snoop_port_i;

  // TB signal declarations

  logic clk_i, rst_ni;
  ACE_BUS #(
            .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
            .AXI_DATA_WIDTH ( AxiDataWidth     ),
            .AXI_ID_WIDTH   ( AxiIdWidth + 32'd1 ),
            .AXI_USER_WIDTH ( AxiUserWidth     )
            ) axi_data ();
  ACE_BUS_DV #(
               .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
               .AXI_DATA_WIDTH ( AxiDataWidth     ),
               .AXI_ID_WIDTH   ( AxiIdWidth + 32'd1 ),
               .AXI_USER_WIDTH ( AxiUserWidth     )
               ) axi_data_dv(clk_i);
  ACE_BUS_DV #(
               .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
               .AXI_DATA_WIDTH ( AxiDataWidth     ),
               .AXI_ID_WIDTH   ( AxiIdWidth + 32'd1 ),
               .AXI_USER_WIDTH ( AxiUserWidth     )
               ) axi_data_monitor_dv(clk_i);
  `ACE_ASSIGN(axi_data_dv, axi_data)
  `ACE_ASSIGN_FROM_REQ(axi_data, axi_data_o)
  `ACE_ASSIGN_TO_RESP(axi_data_i, axi_data)

  ACE_BUS #(
            .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
            .AXI_DATA_WIDTH ( AxiDataWidth     ),
            .AXI_ID_WIDTH   ( AxiIdWidth + 32'd1 ),
            .AXI_USER_WIDTH ( AxiUserWidth     )
            ) axi_bypass ();
  ACE_BUS_DV #(
               .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
               .AXI_DATA_WIDTH ( AxiDataWidth     ),
               .AXI_ID_WIDTH   ( AxiIdWidth + 32'd1 ),
               .AXI_USER_WIDTH ( AxiUserWidth     )
               ) axi_bypass_dv(clk_i);
  ACE_BUS_DV #(
               .AXI_ADDR_WIDTH ( AxiAddrWidth     ),
               .AXI_DATA_WIDTH ( AxiDataWidth     ),
               .AXI_ID_WIDTH   ( AxiIdWidth + 32'd1 ),
               .AXI_USER_WIDTH ( AxiUserWidth     )
               ) axi_bypass_monitor_dv(clk_i);
  `ACE_ASSIGN(axi_bypass_dv, axi_bypass)
  `ACE_ASSIGN_FROM_REQ(axi_bypass, axi_bypass_o)
  `ACE_ASSIGN_TO_RESP(axi_bypass_i, axi_bypass)

  localparam time ApplTime =  2ns;
  localparam time TestTime =  8ns;

  typedef ace_test::ace_rand_slave #(
                                     // AXI interface parameters
                                     .AW ( AxiAddrWidth ),
                                     .DW ( AxiDataWidth ),
                                     .IW ( AxiIdWidth + 32'd1 ),
                                     .UW ( AxiUserWidth ),
                                     // Stimuli application and test time
                                     .TA ( ApplTime         ),
                                     .TT ( TestTime         )
                                     ) axi_rand_slave_t;

  axi_rand_slave_t axi_rand_slave_data;
  axi_rand_slave_t axi_rand_slave_bypass;

  initial begin
    axi_rand_slave_data = new( axi_data_dv );
    axi_rand_slave_bypass = new( axi_bypass_dv );
    axi_rand_slave_data.reset();
    axi_rand_slave_bypass.reset();
    @(posedge rst_ni);
    fork
      axi_rand_slave_data.run();
      axi_rand_slave_bypass.run();
    join
  end

  logic                           start_rd, start_wr, start_snoop;
  logic                           check_done;
  logic                           init_cache_done;

  cache_line_t [DCACHE_NUM_WORDS-1:0][DCACHE_SET_ASSOC-1:0] cache_status;

  // DUT

  assign enable_i = 1'b1;
  assign flush_i = 1'b0;

  std_nbdcache  #(
    .ArianeCfg      ( ArianeCfg                 ),
    .AXI_ADDR_WIDTH ( AxiAddrWidth              ),
    .AXI_ID_WIDTH   ( AxiIdWidth + 32'd1        ),
    .AXI_DATA_WIDTH ( AxiDataWidth              ),
    .axi_req_t      ( ariane_ace::m2s_nosnoop_t ),
    .axi_rsp_t      ( ariane_ace::s2m_nosnoop_t )
  ) i_dut (
    .clk_i           ( clk_i           ),
    .rst_ni          ( rst_ni          ),
    .flush_i         ( flush_i         ),
    .flush_ack_o     ( flush_ack_o     ),
    .enable_i        ( enable_i        ),
    .miss_o          ( miss_o          ),
    .amo_req_i       ( '0 /*amo_req_i*/       ),
    .amo_resp_o      ( /*amo_resp_o*/      ),
    .req_ports_i     ( req_ports_i     ),
    .req_ports_o     ( req_ports_o     ),
    .axi_data_o      ( axi_data_o      ),
    .axi_data_i      ( axi_data_i      ),
    .axi_bypass_o    ( axi_bypass_o    ),
    .axi_bypass_i    ( axi_bypass_i    ),
    .snoop_port_o    ( snoop_port_o ),
    .snoop_port_i    ( snoop_port_i )
  );

  // Clock and reset

  initial
    begin
      forever begin
        clk_i = 1; #(CLK_HI);
        clk_i = 0; #(CLK_LO);
      end
    end

  logic [7:0] rst_n_v = '0;

  always_ff @(posedge clk_i) begin
    rst_n_v[6:0] <= rst_n_v[7:1];
    rst_n_v[7] <= 1'b1;
  end

  assign rst_ni = rst_n_v[0];

  // Request generator

  // Tasks

  logic [2:0] active_port;

  task automatic genRdReq();
    logic [31:0] addr;
    addr = $urandom_range(32'h8000);
    if ($urandom_range(1))
      addr = addr + ArianeCfg.CachedRegionAddrBase[0];
    active_port = 0;
    `WAIT_CYC(clk_i, 1)
    req_ports_i[active_port].data_req  = 1'b1;
    req_ports_i[active_port].data_size = 2'b11;
    req_ports_i[active_port].address_tag   = addr2tag(addr);
    req_ports_i[active_port].address_index = addr2index(addr);
    `WAIT_SIG(clk_i, req_ports_o[active_port].data_gnt)
    req_ports_i[active_port].data_req  = 1'b0;
    req_ports_i[active_port].tag_valid     = 1'b1;
    `WAIT_CYC(clk_i,1)
    req_ports_i = '0;
    `WAIT_CYC(clk_i,1)
  endtask

  task automatic genWrReq(
                          input [31:0] addr
                          );
    active_port = 0;
    `WAIT_CYC(clk_i, 1)
    req_ports_i[active_port].data_req  = 1'b1;
    req_ports_i[active_port].data_we  = 1'b1;
    req_ports_i[active_port].data_be  = '1;
    req_ports_i[active_port].data_size = 2'b11;
    req_ports_i[active_port].address_tag   = addr2tag(addr);
    req_ports_i[active_port].tag_valid     = 1'b1;
    req_ports_i[active_port].address_index = addr2index(addr);
    `WAIT_SIG(clk_i, req_ports_o[active_port].data_gnt)
    req_ports_i = '0;
    `WAIT_CYC(clk_i,1)
  endtask

  task automatic genSnoopReq(
                             input [31:0] addr
                             );
    // wait 3 cycle more than the wr requestor
    `WAIT_CYC(clk_i, 3)
    snoop_port_i.ac_valid  = 1'b1;
    snoop_port_i.ac.addr = addr;
    snoop_port_i.ac.prot = '0;
    if ($urandom_range(1))
      snoop_port_i.ac.snoop = 4'b0111;
    else
      snoop_port_i.ac.snoop = 4'b1001;
    `WAIT_SIG(clk_i, snoop_port_o.ac_ready)
    snoop_port_i.ac_valid = 1'b0;
    `WAIT_CYC(clk_i,1)
  endtask

  task automatic cacheFilling();
    for (int i = 0; i < 2048; i++) begin
      logic [31:0] addr = ArianeCfg.CachedRegionAddrBase[0] + i*16;
      `WAIT_CYC(clk_i, 2)
      req_ports_i[0].data_req  = 1'b1;
      req_ports_i[0].data_we  = 1'b0;
      req_ports_i[0].data_be  = '0;
      req_ports_i[0].data_size = 2'b11;
      req_ports_i[0].address_tag   = addr2tag(addr);
      req_ports_i[0].tag_valid     = 1'b1;
      req_ports_i[0].address_index = addr2index(addr);
      `WAIT_SIG(clk_i, req_ports_o[0].data_gnt)
      req_ports_i[0].data_req  = 1'b0;
      req_ports_i[0].tag_valid     = 1'b1;
      `WAIT_CYC(clk_i,1)
      `WAIT_SIG(clk_i, req_ports_o[0].data_rvalid)
      req_ports_i = '0;
      `WAIT_CYC(clk_i,1)
    end
  endtask

  initial begin
    req_ports_i = '0;
    snoop_port_i = '0;

    `WAIT_CYC(clk_i,1)
    `WAIT_SIG(clk_i,rst_ni)

    // fill the cache
    cacheFilling();

    `WAIT_SIG(clk_i,init_cache_done)

    forever begin

      snoop_port_i.cr_ready = 1'b1;
      snoop_port_i.cd_ready = 1'b1;

      for (int i = 0; i < 2048; i++) begin
        fork
          begin
            genWrReq(ArianeCfg.CachedRegionAddrBase[0] + i*16);

          end
          begin
            genSnoopReq(ArianeCfg.CachedRegionAddrBase[0] + i*16);
          end
        join
        `WAIT_SIG(clk_i,check_done)
      end

    end
  end

  // checker

  function bit isCleanUnique(
                             ariane_ace::m2s_nosnoop_t ace_req
                             );
    if (ace_req.ar.snoop == 4'b1011 && ace_req.ar.bar[0] == 1'b0 && (ace_req.ar.domain == 2'b10 || ace_req.ar.domain == 2'b01))
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

  function bit isWriteBack(
                           ariane_ace::m2s_nosnoop_t ace_req
                           );
    if (ace_req.aw.snoop == 3'b011 && ace_req.aw.bar[0] == 1'b0 && (ace_req.aw.domain == 2'b00 || ace_req.aw.domain == 2'b01 || ace_req.aw.domain == 2'b10))
      return 1'b1;
    else
      return 1'b0;
  endfunction

  initial begin

    automatic int way = 0;

    cache_status = '0;
    check_done = 1'b0;
    init_cache_done = 1'b0;

    `WAIT_CYC(clk_i,1)
    `WAIT_SIG(clk_i,rst_ni)

    // initialize the cache
    for (int i = 0; i < 2048; i++) begin
      automatic logic [31:0] addr = ArianeCfg.CachedRegionAddrBase[0] + i*16;
      `WAIT_SIG(clk_i, axi_data_i.r.last)
      if (cache_status[addr[DCACHE_INDEX_WIDTH-1:DCACHE_BYTE_OFFSET]][way].valid)
        way = way + 1;
      cache_status[addr[DCACHE_INDEX_WIDTH-1:DCACHE_BYTE_OFFSET]][way].valid = 1'b1;
      cache_status[addr[DCACHE_INDEX_WIDTH-1:DCACHE_BYTE_OFFSET]][way].dirty = axi_data_i.r.resp[2];
      cache_status[addr[DCACHE_INDEX_WIDTH-1:DCACHE_BYTE_OFFSET]][way].shared = axi_data_i.r.resp[3];
    end

    init_cache_done = 1'b1;

    way = 0;

    // check transactions
    for (int i = 0; i < 2048; i++) begin
      fork
        begin
          automatic logic [31:0] addr = ArianeCfg.CachedRegionAddrBase[0] + i*16;
          automatic logic [3:0]  snoop_type;
          automatic logic        dirty;
          automatic logic        shared;
          `WAIT_SIG(clk_i, snoop_port_i.ac_valid)
          snoop_type = snoop_port_i.ac.snoop;
          dirty = cache_status[addr[DCACHE_INDEX_WIDTH-1:DCACHE_BYTE_OFFSET]][way].dirty;
          shared = cache_status[addr[DCACHE_INDEX_WIDTH-1:DCACHE_BYTE_OFFSET]][way].shared;
          $display("Processing %x, addr %x, dirty=%x, shared=%x", snoop_port_i.ac.snoop, addr, dirty, shared);

          fork
            begin
              case({snoop_type, dirty, shared})
                {CLEAN_INVALID, 1'b0, 1'b0} : begin
                  // dirty bit will be set by write request before snoop is served
                  `WAIT_SIG(clk_i, snoop_port_o.cr_valid)
                  if (snoop_port_o.cr_resp.error)
                    $error("Unexpected CR.RESP.error");
                  if (!snoop_port_o.cr_resp.dataTransfer)
                    $error("Unexpected CR.RESP.dataTransfer = 0");
                  if (snoop_port_o.cr_resp.isShared)
                    $error("Unexpected CR.RESP.isShared");
                  if (!snoop_port_o.cr_resp.passDirty)
                    $error("Unexpected CR.RESP.passDirty = 0");
                end
                {CLEAN_INVALID, 1'b0, 1'b1} : begin
                  // dirty bit will NOT be set by write request before snoop is served since
                  // the write will wait for CleanUnique + ReadUnique before updating
                  fork
                    begin
                      `WAIT_SIG(clk_i, snoop_port_o.cr_valid)
                      if (snoop_port_o.cr_resp.error)
                        $error("Unexpected CR.RESP.error");
                      if (snoop_port_o.cr_resp.dataTransfer)
                        $error("Unexpected CR.RESP.dataTransfer");
                      if (snoop_port_o.cr_resp.isShared)
                        $error("Unexpected CR.RESP.isShared");
                      if (snoop_port_o.cr_resp.passDirty)
                        $error("Unexpected CR.RESP.passDirty");
                    end
                    begin
                      `WAIT_SIG(clk_i, {axi_data_o.ar_valid & axi_data_i.ar_ready})
                      if (!isCleanUnique(axi_data_o))
                        $error("CleanUnique expected");
                      `WAIT_SIG(clk_i, {axi_data_o.ar_valid & axi_data_i.ar_ready})
                      if (!isReadUnique(axi_data_o))
                        $error("ReadUnique expected");
                    end
                  join
                end
                {CLEAN_INVALID, 1'b1, 1'b0} : begin
                  `WAIT_SIG(clk_i, snoop_port_o.cr_valid)
                  if (snoop_port_o.cr_resp.error)
                    $error("Unexpected CR.RESP.error");
                  if (!snoop_port_o.cr_resp.dataTransfer)
                    $error("Unexpected CR.RESP.dataTransfer = 0");
                  if (snoop_port_o.cr_resp.isShared)
                    $error("Unexpected CR.RESP.isShared");
                  if (!snoop_port_o.cr_resp.passDirty)
                    $error("Unexpected CR.RESP.passDirty = 0");
                end
                {CLEAN_INVALID, 1'b1, 1'b1} : begin
                  fork
                    begin
                      `WAIT_SIG(clk_i, snoop_port_o.cr_valid)
                      if (snoop_port_o.cr_resp.error)
                        $error("Unexpected CR.RESP.error");
                      if (!snoop_port_o.cr_resp.dataTransfer)
                        $error("Unexpected CR.RESP.dataTransfer = 0");
                      if (snoop_port_o.cr_resp.isShared)
                        $error("Unexpected CR.RESP.isShared");
                      if (!snoop_port_o.cr_resp.passDirty)
                        $error("Unexpected CR.RESP.passDirty = 0");
                    end
                    begin
                      `WAIT_SIG(clk_i, {axi_data_o.ar_valid & axi_data_i.ar_ready})
                      if (!isCleanUnique(axi_data_o))
                        $error("CleanUnique expected");
                      `WAIT_SIG(clk_i, {axi_data_o.ar_valid & axi_data_i.ar_ready})
                      if (!isReadUnique(axi_data_o))
                        $error("ReadUnique expected");
                    end
                  join
                end
                {READ_UNIQUE, 1'b0, 1'b0} : begin
                  `WAIT_SIG(clk_i, snoop_port_o.cr_valid)
                  if (snoop_port_o.cr_resp.error)
                    $error("Unexpected CR.RESP.error");
                  if (!snoop_port_o.cr_resp.dataTransfer)
                    $error("Unexpected CR.RESP.dataTransfer = 0");
                  if (snoop_port_o.cr_resp.isShared)
                    $error("Unexpected CR.RESP.isShared");
                  if (!snoop_port_o.cr_resp.passDirty)
                    $error("Unexpected CR.RESP.passDirty = 0");
                  `WAIT_SIG(clk_i, snoop_port_o.cd_valid)
                end
                {READ_UNIQUE, 1'b0, 1'b1} : begin
                  fork
                    begin
                      `WAIT_SIG(clk_i, axi_data_o.ar_valid)
                      if (!isCleanUnique(axi_data_o))
                        $error("CleanUnique expected");
                    end
                    begin
                      `WAIT_SIG(clk_i, snoop_port_o.cr_valid)
                      if (snoop_port_o.cr_resp.error)
                        $error("Unexpected CR.RESP.error");
                      if (!snoop_port_o.cr_resp.dataTransfer)
                        $error("Unexpected CR.RESP.dataTransfer = 0");
                      if (snoop_port_o.cr_resp.isShared)
                        $error("Unexpected CR.RESP.isShared");
                      if (snoop_port_o.cr_resp.passDirty)
                        $error("Unexpected CR.RESP.passDirty");
                      `WAIT_SIG(clk_i, snoop_port_o.cd_valid)
                    end
                  join
                end
                {READ_UNIQUE, 1'b1, 1'b0} : begin
                  `WAIT_SIG(clk_i, snoop_port_o.cr_valid)
                  if (snoop_port_o.cr_resp.error)
                    $error("Unexpected CR.RESP.error");
                  if (!snoop_port_o.cr_resp.dataTransfer)
                    $error("Unexpected CR.RESP.dataTransfer = 0");
                  if (snoop_port_o.cr_resp.isShared)
                    $error("Unexpected CR.RESP.isShared");
                  if (!snoop_port_o.cr_resp.passDirty)
                    $error("Unexpected CR.RESP.passDirty = 0");
                  `WAIT_SIG(clk_i, snoop_port_o.cd_valid)
                end
                {READ_UNIQUE, 1'b1, 1'b1} : begin
                  fork
                    begin
                      `WAIT_SIG(clk_i, snoop_port_o.cr_valid)
                      if (snoop_port_o.cr_resp.error)
                        $error("Unexpected CR.RESP.error");
                      if (!snoop_port_o.cr_resp.dataTransfer)
                        $error("Unexpected CR.RESP.dataTransfer = 0");
                      if (snoop_port_o.cr_resp.isShared)
                        $error("Unexpected CR.RESP.isShared");
                      if (!snoop_port_o.cr_resp.passDirty)
                        $error("Unexpected CR.RESP.passDirty = 0");
                      `WAIT_SIG(clk_i, snoop_port_o.cd_valid)
                    end
                    begin
                      `WAIT_SIG(clk_i, axi_data_o.ar_valid)
                      if (!isCleanUnique(axi_data_o))
                        $error("CleanUnique expected");
                    end
                  join
                end
              endcase
            end
            begin
              `WAIT_SIG(clk_i, req_ports_o[0].data_gnt)
            end
          join
          `WAIT_CYC(clk_i,5)
          check_done = 1'b1;
          `WAIT_CYC(clk_i,2)
          check_done = 1'b0;

          if (i%256 == 255)
            way = way + 1;

        end
        begin
          `WAIT_CYC(clk_i, timeout)
          $error("Timeout");
          $finish();
        end
      join_any
      disable fork;

    end

    $display("End of simulation");
    $finish();
  end

endmodule
