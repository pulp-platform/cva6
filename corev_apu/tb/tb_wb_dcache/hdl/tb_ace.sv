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

module tb_ace import ariane_pkg::*; import std_cache_pkg::*; import tb_pkg::*; #()();

  localparam MaxRounds = 1000000;

  // leave this
  timeunit 1ps;
  timeprecision 1ps;

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
    // CLIC
    CLICNumInterruptSrc:    1'b1,
    CLICIntCtlBits:         1,
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

  // DUT signal declarations

  logic                           enable_i;
  logic                           flush_i;
  logic                           flush_ack_o;
  logic                           miss_o;
  amo_req_t                       amo_req_i;
  amo_resp_t                      amo_resp_o;
  dcache_req_i_t [2:0]            req_ports_i;
  dcache_req_o_t [2:0]            req_ports_o;
  ariane_ace::req_nosnoop_t       axi_data_o;
  ariane_ace::resp_nosnoop_t      axi_data_i;
  ariane_ace::req_nosnoop_t       axi_bypass_o;
  ariane_ace::resp_nosnoop_t      axi_bypass_i;
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

  // DUT

  assign enable_i = 1'b1;
  assign flush_i = 1'b0;

  std_nbdcache  #(
    .ArianeCfg         ( ArianeCfg                  ),
    .VLD_SRAM_SIM_INIT ( "zeros"                    ),
    .AXI_ADDR_WIDTH    ( AxiAddrWidth               ),
    .AXI_ID_WIDTH      ( AxiIdWidth + 32'd1         ),
    .AXI_DATA_WIDTH    ( AxiDataWidth               ),
    .axi_req_t         ( ariane_ace::req_nosnoop_t  ),
    .axi_rsp_t         ( ariane_ace::resp_nosnoop_t )
  ) i_dut (
    .clk_i           ( clk_i           ),
    .rst_ni          ( rst_ni          ),
    .flush_i         ( flush_i         ),
    .flush_ack_o     ( flush_ack_o     ),
    .enable_i        ( enable_i        ),
    .miss_o          ( miss_o          ),
    .busy_o          (                 ),
    .stall_i         ( 1'b0            ),
    .init_ni         ( 1'b1            ),
    .amo_req_i       ( '0 /*amo_req_i*/),
    .amo_resp_o      ( /*amo_resp_o*/  ),
    .req_ports_i     ( req_ports_i     ),
    .req_ports_o     ( req_ports_o     ),
    .axi_data_o      ( axi_data_o      ),
    .axi_data_i      ( axi_data_i      ),
    .axi_bypass_o    ( axi_bypass_o    ),
    .axi_bypass_i    ( axi_bypass_i    ),
    .snoop_port_o    ( snoop_port_o ),
    .snoop_port_i    ( snoop_port_i )
  );

  // Request generation

  request_scheduler
    #(
      .NR_CPU_PORTS (3),
      .ArianeCfg ( ArianeCfg ),
      .AxiAddrWidth (AxiAddrWidth),
      .AxiDataWidth (AxiDataWidth),
      .ApplTime (ApplTime),
      .TestTime (TestTime)
      )
  i_request_scheduler
    (

     .clk_i (clk_i),
     .rst_ni (rst_ni),
     .check_done_i (check_done),
     .req_ports_o(req_ports_o),
     .req_ports_i(req_ports_i),
     .snoop_req_o(snoop_port_i),
     .snoop_resp_i(snoop_port_o)
     );

  // Execution check

  dcache_checker
    #(
      .MAX_ROUNDS (MaxRounds),
      .NR_CPU_PORTS (3),
      .ArianeCfg ( ArianeCfg )
      )
  i_checker
    (
     .clk_i (clk_i),
     .rst_ni (rst_ni),
     .check_done_o (check_done),
     .req_ports_o,
     .req_ports_i,
     .axi_data_o,
     .axi_data_i,
     .axi_bypass_o,
     .axi_bypass_i,
     .snoop_req_i (snoop_port_i),
     .snoop_resp_o (snoop_port_o)
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

endmodule
