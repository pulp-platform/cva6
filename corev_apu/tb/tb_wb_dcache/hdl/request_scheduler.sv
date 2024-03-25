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

module request_scheduler import ariane_pkg::*; import std_cache_pkg::*; import tb_pkg::*;
  #(
    parameter int unsigned NR_CPU_PORTS = 3,
    parameter ariane_cfg_t ArianeCfg        = ArianeDefaultConfig, // contains cacheable regions
    parameter int unsigned AxiAddrWidth = 32'd64,
    parameter int unsigned AxiDataWidth = 32'd64,
    parameter time         ApplTime = 2ns,
    parameter time         TestTime = 8ns
    )
  (
   input logic clk_i,
   input logic rst_ni,
   input logic check_done_i,
   output      ariane_pkg::dcache_req_i_t[NR_CPU_PORTS-1:0] req_ports_i,
   input       ariane_pkg::dcache_req_o_t[NR_CPU_PORTS-1:0] req_ports_o,
   output      ariane_ace::snoop_req_t snoop_req_o,
   input       ariane_ace::snoop_resp_t snoop_resp_i
   );

  localparam int timeout = 1000;

  SNOOP_BUS #(
              .SNOOP_ADDR_WIDTH ( AxiAddrWidth     ),
              .SNOOP_DATA_WIDTH ( AxiDataWidth     )
              ) snoop ();
  SNOOP_BUS_DV #(
                 .SNOOP_ADDR_WIDTH ( AxiAddrWidth     ),
                 .SNOOP_DATA_WIDTH ( AxiDataWidth     )
                 ) snoop_dv(clk_i);
  SNOOP_BUS_DV #(
                 .SNOOP_ADDR_WIDTH ( AxiAddrWidth     ),
                 .SNOOP_DATA_WIDTH ( AxiDataWidth     )
                 ) snoop_monitor_dv(clk_i);
  `SNOOP_ASSIGN(snoop, snoop_dv)
  `SNOOP_ASSIGN_TO_REQ(snoop_req_o, snoop)
  `SNOOP_ASSIGN_FROM_RESP(snoop, snoop_resp_i)

  typedef snoop_test::snoop_rand_master #(
                                     // AXI interface parameters
                                     .AW ( AxiAddrWidth ),
                                     .DW ( AxiDataWidth ),
                                     // Stimuli application and test time
                                     .TA ( ApplTime         ),
                                     .TT ( TestTime         )
                                     ) snoop_rand_master_t;

  snoop_rand_master_t snoop_rand_master;

  // Functions

  function logic[DCACHE_TAG_WIDTH-1:0] addr2tag(logic[63:0] addr);
    return addr[DCACHE_TAG_WIDTH+DCACHE_INDEX_WIDTH-1:DCACHE_INDEX_WIDTH];
  endfunction

  function logic[DCACHE_INDEX_WIDTH-1:0] addr2index(logic[63:0] addr);
    return addr[DCACHE_INDEX_WIDTH-1:0];
  endfunction

  // Tasks

  logic [2:0] active_port;

  task automatic genRdReq();
    logic [31:0] addr;

    addr = $urandom_range(32'h8000);
    if ($urandom_range(1))
      addr = addr + ArianeCfg.CachedRegionAddrBase[0];
    active_port = $urandom_range(2);
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

  task automatic genWrReq();
    logic [31:0] addr;

    addr = $urandom_range(32'h8000);
    if ($urandom_range(1))
      addr = addr + ArianeCfg.CachedRegionAddrBase[0];
    active_port = $urandom_range(2);
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

  // Scheduler

  typedef enum int {READ, WRITE, SNOOP} state_req_t;
  state_req_t state_req;

  initial begin
    req_ports_i = '0;

    snoop_rand_master = new( snoop_dv );
    snoop_rand_master.add_memory_region(ArianeCfg.SharedRegionAddrBase[0], ArianeCfg.SharedRegionAddrBase[0]+ArianeCfg.SharedRegionLength[0], axi_pkg::DEVICE_NONBUFFERABLE);
    snoop_rand_master.reset();

    `WAIT_CYC(clk_i,1)
    `WAIT_SIG(clk_i,rst_ni)

    forever begin
      // randomly select the next transaction
      $cast(state_req, $urandom_range(SNOOP, READ));
      case (state_req)
        READ: begin
          genRdReq();
        end

        WRITE: begin
          genWrReq();
        end

        SNOOP: begin
          snoop_rand_master.run(1);
        end
      endcase

      fork
        begin
          `WAIT_SIG(clk_i, check_done_i)
        end
        begin
          `WAIT_CYC(clk_i, timeout)
          $error("Timeout");
          $finish();
        end
      join_any
      disable fork;
    end
  end

endmodule
