// Copyright 2017-2019 ETH Zurich and University of Bologna.
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
// Date: 19.03.2017
// Description: Ariane Top-level module


module ariane import ariane_pkg::*; #(
  parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty,
  parameter bit IsRVFI = bit'(0),
  parameter type rvfi_probes_t = logic,
  parameter int unsigned AxiAddrWidth = ariane_axi::AddrWidth,
  parameter int unsigned AxiDataWidth = ariane_axi::DataWidth,
  parameter int unsigned AxiIdWidth   = ariane_axi::IdWidth,
  parameter type axi_ar_chan_t = ariane_axi::ar_chan_t,
  parameter type axi_aw_chan_t = ariane_axi::aw_chan_t,
  parameter type axi_w_chan_t  = ariane_axi::w_chan_t,
  parameter type noc_req_t = ariane_axi::req_t,
  parameter type noc_resp_t = ariane_axi::resp_t
) (
  input  logic                         clk_i,
  input  logic                         rst_ni,
  // Core ID, Cluster ID and boot address are considered more or less static
  input  logic [riscv::VLEN-1:0]       boot_addr_i,  // reset boot address
  input  logic [riscv::XLEN-1:0]       hart_id_i,    // hart id in a multicore environment (reflected in a CSR)

  // Interrupt inputs
  input  logic [1:0]                   irq_i,        // level sensitive IR lines, mip & sip (async)
  input  logic                         ipi_i,        // inter-processor interrupts (async)
  // Timer facilities
  input  logic                         time_irq_i,   // timer interrupt in (async)
  input  logic                         debug_req_i,  // debug request (async)
  // RISC-V formal interface port (`rvfi`):
  // Can be left open when formal tracing is not needed.
  output rvfi_probes_t rvfi_probes_o,
  // memory side
  output noc_req_t                     noc_req_o,
  input  noc_resp_t                    noc_resp_i
);

  cvxif_pkg::cvxif_req_t  cvxif_req;
  cvxif_pkg::cvxif_resp_t cvxif_resp;

  cva6 #(
    .CVA6Cfg ( CVA6Cfg ),
    .IsRVFI ( IsRVFI ),
    .rvfi_probes_t ( rvfi_probes_t ),
    .axi_ar_chan_t (axi_ar_chan_t),
    .axi_aw_chan_t (axi_aw_chan_t),
    .axi_w_chan_t (axi_w_chan_t),
    .noc_req_t (noc_req_t),
    .noc_resp_t (noc_resp_t)
  ) i_cva6 (
    .clk_i                ( clk_i                     ),
    .rst_ni               ( rst_ni                    ),
    .clear_i              ( '0                        ),
    .boot_addr_i          ( boot_addr_i               ),
    .hart_id_i            ( hart_id_i                 ),
    .irq_i                ( irq_i                     ),
    .ipi_i                ( ipi_i                     ),
    .time_irq_i           ( time_irq_i                ),
    .debug_req_i          ( debug_req_i               ),
    .clic_irq_valid_i     ( 1'b0                      ),
    .clic_irq_id_i        ( '0                        ),
    .clic_irq_level_i     ( '0                        ),
    .clic_irq_priv_i      ( '0                        ),
    .clic_irq_shv_i       ( 1'b0                      ),
    .clic_irq_ready_o     (                           ),
    .clic_kill_req_i      ( 1'b0                      ),
    .clic_kill_ack_o      (                           ),
    .rvfi_probes_o        ( rvfi_probes_o             ),
    .cvxif_req_o          ( cvxif_req                 ),
    .cvxif_resp_i         ( cvxif_resp                ),
    .noc_req_o            ( noc_req_o                 ),
    .noc_resp_i           ( noc_resp_i                ),
    .dcache_commit_wbuffer_not_ni_i ( '0 ),
    .inval_ready_i                  ( '0 ),
    .priv_lvl_o                     (    ),
    .busy_i                         ( '0 ),
    .stall_o                        (    ),
    .init_no                        (    ),
    .icache_en_o                    (    ),
    .icache_flush_o                 (    ),
    .icache_miss_i                  ( '0 ),
    .dcache_enable_o                (    ),
    .dcache_flush_o                 (    ),
    .dcache_flush_ack_i             ( '0 ),
    .dcache_miss_i                  ( '0 ),
    .wbuffer_empty_i                ( '0 ),
    .dcache_req_ports_o             (    ),
    .dcache_req_ports_i             ( '0 ),
    .amo_req_o                      (    ),
    .amo_resp_i                     ( '0 ),
    .icache_areq_o                  (    ),
    .icache_areq_i                  ( '0 ),
    .icache_dreq_o                  (    ),
    .icache_dreq_i                  ( '0 )
  );

  if (CVA6Cfg.CvxifEn) begin : gen_example_coprocessor
    cvxif_example_coprocessor i_cvxif_coprocessor (
      .clk_i                ( clk_i                          ),
      .rst_ni               ( rst_ni                         ),
      .cvxif_req_i          ( cvxif_req                      ),
      .cvxif_resp_o         ( cvxif_resp                     )
    );
  end

endmodule // ariane
