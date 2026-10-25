// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// The implementation wrapper of the ASIC flow: CVA6 with its AXI, interrupt
// and debug ports, in the configuration its package defines, with the memory
// attributes of the testbench SoC (test/hw/cva6tb_cfg_pkg.sv).

package cva6_asic_pkg;

  // The configuration the testbench SoC builds from the same package, so that
  // the implemented core is the one the testbench runs
  localparam config_pkg::cva6_user_cfg_t CVA6UserCfg = cva6_config_pkg::cva6_cfg;
  localparam config_pkg::cva6_cfg_t          CVA6Cfg = cva6tb_cfg_pkg::gen_cva6_cfg(CVA6UserCfg);

  localparam type axi_ar_chan_t = struct packed {
    logic [CVA6Cfg.AxiIdWidth-1:0]   id;
    logic [CVA6Cfg.AxiAddrWidth-1:0] addr;
    axi_pkg::len_t                   len;
    axi_pkg::size_t                  size;
    axi_pkg::burst_t                 burst;
    logic                            lock;
    axi_pkg::cache_t                 cache;
    axi_pkg::prot_t                  prot;
    axi_pkg::qos_t                   qos;
    axi_pkg::region_t                region;
    logic [CVA6Cfg.AxiUserWidth-1:0] user;
  };

  localparam type axi_aw_chan_t = struct packed {
    logic [CVA6Cfg.AxiIdWidth-1:0]   id;
    logic [CVA6Cfg.AxiAddrWidth-1:0] addr;
    axi_pkg::len_t                   len;
    axi_pkg::size_t                  size;
    axi_pkg::burst_t                 burst;
    logic                            lock;
    axi_pkg::cache_t                 cache;
    axi_pkg::prot_t                  prot;
    axi_pkg::qos_t                   qos;
    axi_pkg::region_t                region;
    axi_pkg::atop_t                  atop;
    logic [CVA6Cfg.AxiUserWidth-1:0] user;
  };

  localparam type axi_w_chan_t = struct packed {
    logic [CVA6Cfg.AxiDataWidth-1:0]     data;
    logic [(CVA6Cfg.AxiDataWidth/8)-1:0] strb;
    logic                                last;
    logic [CVA6Cfg.AxiUserWidth-1:0]     user;
  };

  localparam type b_chan_t = struct packed {
    logic [CVA6Cfg.AxiIdWidth-1:0]   id;
    axi_pkg::resp_t                  resp;
    logic [CVA6Cfg.AxiUserWidth-1:0] user;
  };

  localparam type r_chan_t = struct packed {
    logic [CVA6Cfg.AxiIdWidth-1:0]   id;
    logic [CVA6Cfg.AxiDataWidth-1:0] data;
    axi_pkg::resp_t                  resp;
    logic                            last;
    logic [CVA6Cfg.AxiUserWidth-1:0] user;
  };

  localparam type noc_req_t = struct packed {
    axi_aw_chan_t aw;
    logic         aw_valid;
    axi_w_chan_t  w;
    logic         w_valid;
    logic         b_ready;
    axi_ar_chan_t ar;
    logic         ar_valid;
    logic         r_ready;
  };

  localparam type noc_resp_t = struct packed {
    logic    aw_ready;
    logic    ar_ready;
    logic    w_ready;
    logic    b_valid;
    b_chan_t b;
    logic    r_valid;
    r_chan_t r;
  };

endpackage

module cva6_asic_top
  import cva6_asic_pkg::*;
(
  input  logic        clk_i,
  input  logic        rst_ni,
  input  logic [63:0] boot_addr_i,
  input  logic        clic_irq_valid_i,
  input  logic [$clog2(CVA6Cfg.CLICNumInterruptSrc)-1:0] clic_irq_id_i,
  input  logic [ 7:0] clic_irq_level_i,
  input  logic [ 1:0] clic_irq_priv_i,
  input  logic [ 1:0] clic_irq_rstk_i,
  input  logic        clic_irq_v_i,
  input  logic [ 5:0] clic_irq_vsid_i,
  input  logic        clic_irq_shv_i,
  output logic        clic_irq_ready_o,
  input  logic        clic_irq_kill_req_i,
  output logic        clic_irq_kill_ack_o,
  input  logic [ 1:0] irq_i,
  input  logic        ipi_i,
  input  logic        time_irq_i,
  input  logic        debug_req_i,
  output noc_req_t    noc_req_o,
  input  noc_resp_t   noc_resp_i
);

  riscv::priv_lvl_t clic_irq_priv_struct;

  assign clic_irq_priv_struct = riscv::priv_lvl_t'(clic_irq_priv_i);

  cva6 #(
    .CVA6Cfg          ( CVA6Cfg                 )
  ) i_cva6 (
    .clk_i            ( clk_i                   ),
    .rst_ni           ( rst_ni                  ),
    .boot_addr_i      ( boot_addr_i             ),
    .hart_id_i        ( 64'h0                   ),
    .irq_i            ( irq_i                   ),
    .ipi_i            ( ipi_i                   ),
    .time_irq_i       ( time_irq_i              ),
    .debug_req_i      ( debug_req_i             ),
    .clic_irq_valid_i ( clic_irq_valid_i        ),
    .clic_irq_id_i    ( clic_irq_id_i           ),
    .clic_irq_level_i ( clic_irq_level_i        ),
    .clic_irq_priv_i  ( clic_irq_priv_struct    ),
    .clic_irq_rstk_i  ( clic_irq_rstk_i         ),
    .clic_irq_v_i     ( clic_irq_v_i            ),
    .clic_irq_vsid_i  ( clic_irq_vsid_i         ),
    .clic_irq_shv_i   ( clic_irq_shv_i          ),
    .clic_irq_ready_o ( clic_irq_ready_o        ),
    .clic_kill_req_i  ( clic_irq_kill_req_i     ),
    .clic_kill_ack_o  ( clic_irq_kill_ack_o     ),
    .rvfi_probes_o    ( /* Not connected */     ),
    .perf_probes_o    ( /* Not connected */     ),
    .cvxif_req_o      ( /* Not connected */     ),
    .cvxif_resp_i     ( '0                      ),
    .noc_req_o        ( noc_req_o               ),
    .noc_resp_i       ( noc_resp_i              )
  );

endmodule
