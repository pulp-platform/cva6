// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// Testbench SoC

module cva6tb_soc
  import ariane_pkg::*;
  import cva6tb_pkg::*;
#(
  parameter time ApplDelay = 0ps,
  parameter time AcqDelay  = 0ps
)(
  input  logic                   clk_i,
  input  logic                   rst_ni,
  input  logic [NumClicIrqs-1:0] clic_irqs_i
);

  logic                              clk;
  logic                              rst_n;
  logic                       [31:0] cva6_boot_addr;
  rvfi_probes_t                      rvfi_probes;

  axi_mst_req_t                      axi_core_req;
  axi_mst_rsp_t                      axi_core_rsp;
  axi_slv_req_t                      axi_err_req;
  axi_slv_rsp_t                      axi_err_rsp;
  axi_slv_req_t                      axi_reg_req;
  axi_slv_rsp_t                      axi_reg_rsp;
  axi_slv_req_t                      axi_amo_req;
  axi_slv_rsp_t                      axi_amo_rsp;
  axi_slv_req_t                      axi_mem_req;
  axi_slv_rsp_t                      axi_mem_rsp;
  axi_slv_req_t                      axi_rmp_req;
  axi_slv_rsp_t                      axi_rmp_rsp;
  axi_mst_req_t [AxiXbarMasters-1:0] axi_mst_reqs;
  axi_mst_rsp_t [AxiXbarMasters-1:0] axi_mst_rsps;
  axi_slv_req_t [ AxiXbarSlaves-1:0] axi_slv_reqs;
  axi_slv_rsp_t [ AxiXbarSlaves-1:0] axi_slv_rsps;

  logic                              clic_irq_valid, clic_irq_ready;
  logic                              clic_irq_kill_req, clic_irq_kill_ack;
  logic                              clic_irq_shv;
  logic                        [7:0] clic_irq_id;
  logic                        [7:0] clic_irq_level;
  logic                        [1:0] clic_irq_priv;
  logic                              clic_irq_v;
  logic                        [5:0] clic_irq_vsid;

  reg_req_t                          reg_pcr_req;
  reg_rsp_t                          reg_pcr_rsp;
  reg_req_t                          reg_console_req;
  reg_rsp_t                          reg_console_rsp;
  reg_req_t                          reg_clic_req;
  reg_rsp_t                          reg_clic_rsp;
  reg_req_t                          reg_err_req;
  reg_rsp_t                          reg_err_rsp;
  reg_req_t                          reg_in_req;
  reg_rsp_t                          reg_in_rsp;
  reg_req_t       [RegbusOutNum-1:0] reg_out_reqs;
  reg_rsp_t       [RegbusOutNum-1:0] reg_out_rsps;

  reg_idx_t                          reg_select;
  reg_idx_t                          default_reg_idx;

  function automatic void load_hex();
    string binary_path;
    if ($value$plusargs("binary=%s", binary_path)) begin
      log($sformatf("Running program \"%s\"", binary_path));
      $readmemh(binary_path, i_axi_sim_mem.mem);
    end else begin
      log("WARNING: no binary path provided");
    end
  endfunction

  assign clk   = clk_i;
  assign rst_n = rst_ni;

  assign default_reg_idx = (RegbusErrId);

  assign axi_mst_reqs[0] = axi_core_req;
  assign axi_core_rsp = axi_mst_rsps[0];

  assign axi_err_req = axi_slv_reqs[AxiSlvIdErrSlv];
  assign axi_slv_rsps[AxiSlvIdErrSlv] = axi_err_rsp;

  assign axi_reg_req = axi_slv_reqs[AxiSlvIdReg];
  assign axi_slv_rsps[AxiSlvIdReg] = axi_reg_rsp;

  assign axi_amo_req = axi_slv_reqs[AxiSlvIdDram];
  assign axi_slv_rsps[AxiSlvIdDram] = axi_amo_rsp;

  assign reg_err_req = reg_out_reqs[RegbusErrId];
  assign reg_out_rsps[RegbusErrId] = reg_err_rsp;

  assign reg_pcr_req = reg_out_reqs[RegbusPCRsId];
  assign reg_out_rsps[RegbusPCRsId] = reg_pcr_rsp;

  assign reg_console_req = reg_out_reqs[RegbusConsoleId];
  assign reg_out_rsps[RegbusConsoleId] = reg_console_rsp;

  assign reg_clic_req = reg_out_reqs[RegbusClicId];
  assign reg_out_rsps[RegbusClicId] = reg_clic_rsp;

  cva6 #(
    .CVA6Cfg             ( CVA6Cfg                           ),
    .rvfi_probes_instr_t ( rvfi_probes_instr_t               ),
    .rvfi_probes_csr_t   ( rvfi_probes_csr_t                 ),
    .rvfi_probes_t       ( rvfi_probes_t                     ),
    .axi_ar_chan_t       ( axi_mst_ar_chan_t                 ),
    .axi_aw_chan_t       ( axi_mst_aw_chan_t                 ),
    .axi_w_chan_t        ( axi_mst_w_chan_t                  ),
    .b_chan_t            ( axi_mst_b_chan_t                  ),
    .r_chan_t            ( axi_mst_r_chan_t                  ),
    .noc_req_t           ( axi_mst_req_t                     ),
    .noc_resp_t          ( axi_mst_rsp_t                     )
  ) i_cva6 (
    .clk_i               ( clk                               ),
    .rst_ni              ( rst_n                             ),
    .boot_addr_i         ( {32'h0, cva6_boot_addr}           ),
    .hart_id_i           ( 64'h0                             ),
    .irq_i               ( '0                                ),
    .ipi_i               ( '0                                ),
    .time_irq_i          ( '0                                ),
    .debug_req_i         ( '0                                ),
    .clic_irq_valid_i    ( clic_irq_valid                    ),
    .clic_irq_id_i       ( clic_irq_id                       ),
    .clic_irq_level_i    ( clic_irq_level                    ),
    .clic_irq_priv_i     ( riscv::priv_lvl_t'(clic_irq_priv) ),
    .clic_irq_v_i        ( clic_irq_v                        ),
    .clic_irq_vsid_i     ( clic_irq_vsid                     ),
    .clic_irq_shv_i      ( clic_irq_shv                      ),
    .clic_irq_ready_o    ( clic_irq_ready                    ),
    .clic_kill_req_i     ( clic_irq_kill_req                 ),
    .clic_kill_ack_o     ( clic_irq_kill_ack                 ),
    .rvfi_probes_o       ( rvfi_probes                       ),
    .cvxif_req_o         ( /* Not connected */               ),
    .cvxif_resp_i        ( '0                                ),
    .noc_req_o           ( axi_core_req                      ),
    .noc_resp_i          ( axi_core_rsp                      )
  );

  // CLIC interrupt controller
  clic #(
    .N_SOURCE       ( NumClicIrqs       ),
    .INTCTLBITS     ( 8                 ),
    .reg_req_t      ( reg_req_t         ),
    .reg_rsp_t      ( reg_rsp_t         ),
    .SSCLIC         ( 1                 ),
    .USCLIC         ( 0                 ),
    .VSCLIC         ( 1                 ),
    .N_VSCTXTS      ( 4                 ),
    .VSPRIO         ( 1                 ),
    .VSPRIO_W       ( 1                 )
  ) i_clic (
    .clk_i          ( clk               ),
    .rst_ni         ( rst_n             ),
    .reg_req_i      ( reg_clic_req      ),
    .reg_rsp_o      ( reg_clic_rsp      ),
    .intr_src_i     ( clic_irqs_i       ),
    .irq_valid_o    ( clic_irq_valid    ),
    .irq_ready_i    ( clic_irq_ready    ),
    .irq_id_o       ( clic_irq_id       ),
    .irq_level_o    ( clic_irq_level    ),
    .irq_shv_o      ( clic_irq_shv      ),
    .irq_priv_o     ( clic_irq_priv     ),
    .irq_v_o        ( clic_irq_v        ),
    .irq_vsid_o     ( clic_irq_vsid     ),
    .irq_kill_req_o ( clic_irq_kill_req ),
    .irq_kill_ack_i ( clic_irq_kill_ack )
  );

  // AXI interconnect
  axi_xbar #(
    .Cfg                   ( AxiXbarCfg             ),
    .ATOPs                 ( 1                      ),
    .Connectivity          ( '1                     ),
    .slv_aw_chan_t         ( axi_mst_aw_chan_t      ),
    .mst_aw_chan_t         ( axi_slv_aw_chan_t      ),
    .w_chan_t              ( axi_mst_w_chan_t       ),
    .slv_b_chan_t          ( axi_mst_b_chan_t       ),
    .mst_b_chan_t          ( axi_slv_b_chan_t       ),
    .slv_ar_chan_t         ( axi_mst_ar_chan_t      ),
    .mst_ar_chan_t         ( axi_slv_ar_chan_t      ),
    .slv_r_chan_t          ( axi_mst_r_chan_t       ),
    .mst_r_chan_t          ( axi_slv_r_chan_t       ),
    .slv_req_t             ( axi_mst_req_t          ),
    .slv_resp_t            ( axi_mst_rsp_t          ),
    .mst_req_t             ( axi_slv_req_t          ),
    .mst_resp_t            ( axi_slv_rsp_t          ),
    .rule_t                ( addr_rule_t            )
  ) i_axi_xbar (
    .clk_i                 ( clk                    ),
    .rst_ni                ( rst_n                  ),
    .test_i                ( '0                     ),
    .slv_ports_req_i       ( axi_mst_reqs           ),
    .slv_ports_resp_o      ( axi_mst_rsps           ),
    .mst_ports_req_o       ( axi_slv_reqs           ),
    .mst_ports_resp_i      ( axi_slv_rsps           ),
    .addr_map_i            ( AxiMap                 ),
    .en_default_mst_port_i ( {AxiXbarMasters{1'b1}} ),
    .default_mst_port_i    ( {AxiXbarMasters{2'b0}} )
  );

  // AXI error slave
  axi_err_slv #(
    .AxiIdWidth  ( AxiSlvIdWidth ),
    .axi_req_t   ( axi_slv_req_t ),
    .axi_resp_t  ( axi_slv_rsp_t )
  ) i_axi_err_slv (
    .clk_i       ( clk           ),
    .rst_ni      ( rst_n         ),
    .test_i      ( '0            ),
    .slv_req_i   ( axi_err_req   ),
    .slv_resp_o  ( axi_err_rsp   )
  );

  // Convert from AXI to reg protocol
  axi_to_reg_v2 #(
    .AxiAddrWidth ( CVA6UserCfg.AxiAddrWidth ),
    .AxiDataWidth ( CVA6UserCfg.AxiDataWidth ),
    .AxiIdWidth   ( AxiSlvIdWidth            ),
    .AxiUserWidth ( CVA6UserCfg.AxiUserWidth ),
    .RegDataWidth ( 32                       ),
    .CutMemReqs   ( 0                        ),
    .axi_req_t    ( axi_slv_req_t            ),
    .axi_rsp_t    ( axi_slv_rsp_t            ),
    .reg_req_t    ( reg_req_t                ),
    .reg_rsp_t    ( reg_rsp_t                )
  ) i_axi_to_reg_v2 (
    .clk_i        ( clk                      ),
    .rst_ni       ( rst_n                    ),
    .axi_req_i    ( axi_reg_req              ),
    .axi_rsp_o    ( axi_reg_rsp              ),
    .reg_req_o    ( reg_in_req               ),
    .reg_rsp_i    ( reg_in_rsp               ),
    .reg_id_o     (                          ),
    .busy_o       (                          )
  );

  // Non-matching addresses are directed to an error slave
  addr_decode #(
    .NoIndices        ( RegbusOutNum    ),
    .NoRules          ( RegbusRulesNum  ),
    .addr_t           ( reg_addr_t      ),
    .rule_t           ( reg_addr_rule_t )
  ) i_reg_demux_decode (
    .addr_i           ( reg_in_req.addr ),
    .addr_map_i       ( RegbusMap       ),
    .idx_o            ( reg_select      ),
    .dec_valid_o      (                 ),
    .dec_error_o      (                 ),
    .en_default_idx_i ( 1'b1            ),
    .default_idx_i    ( default_reg_idx )
  );

  reg_demux #(
    .NoPorts     ( RegbusOutNum ),
    .req_t       ( reg_req_t    ),
    .rsp_t       ( reg_rsp_t    )
  ) i_reg_demux (
    .clk_i       ( clk          ),
    .rst_ni      ( rst_n        ),
    .in_select_i ( reg_select   ),
    .in_req_i    ( reg_in_req   ),
    .in_rsp_o    ( reg_in_rsp   ),
    .out_req_o   ( reg_out_reqs ),
    .out_rsp_i   ( reg_out_rsps )
  );

  reg_err_slv #(
    .DW      ( 32           ),
    .ERR_VAL ( 32'hBADCAB1E ),
    .req_t   ( reg_req_t    ),
    .rsp_t   ( reg_rsp_t    )
  ) i_reg_err_slv (
    .req_i   ( reg_err_req  ),
    .rsp_o   ( reg_err_rsp  )
  );

  // Platform control registers
  cva6tb_regs #(
    .reg_req_t   ( reg_req_t      ),
    .reg_rsp_t   ( reg_rsp_t      )
  ) i_regs (
    .clk_i       ( clk            ),
    .rst_ni      ( rst_n          ),
    .reg_req_i   ( reg_pcr_req    ),
    .reg_rsp_o   ( reg_pcr_rsp    ),
    .boot_addr_o ( cva6_boot_addr )
  );

  // Sim console
  cva6tb_sim_console #(
    .reg_req_t   ( reg_req_t       ),
    .reg_rsp_t   ( reg_rsp_t       )
  ) i_sim_console (
    .clk_i       ( clk             ),
    .rst_ni      ( rst_n           ),
    .reg_req_i   ( reg_console_req ),
    .reg_rsp_o   ( reg_console_rsp )
  );

  // RISC-V atomics filter
  axi_riscv_atomics_structs #(
    .AxiAddrWidth     ( CVA6UserCfg.AxiAddrWidth ),
    .AxiDataWidth     ( CVA6UserCfg.AxiDataWidth ),
    .AxiIdWidth       ( AxiSlvIdWidth            ),
    .AxiUserWidth     ( CVA6UserCfg.AxiUserWidth ),
    .AxiMaxReadTxns   ( 24                       ),
    .AxiMaxWriteTxns  ( 24                       ),
    .AxiUserAsId      ( 1                        ),
    .AxiUserIdMsb     ( 0                        ),
    .AxiUserIdLsb     ( 0                        ),
    .RiscvWordWidth   ( 64                       ),
    .NAxiCuts         ( 0                        ),
    .axi_req_t        ( axi_slv_req_t            ),
    .axi_rsp_t        ( axi_slv_rsp_t            )
  ) i_riscv_atomics (
    .clk_i            ( clk                      ),
    .rst_ni           ( rst_n                    ),
    .axi_slv_req_i    ( axi_amo_req              ),
    .axi_slv_rsp_o    ( axi_amo_rsp              ),
    .axi_mst_req_o    ( axi_mem_req              ),
    .axi_mst_rsp_i    ( axi_mem_rsp              )
  );

  // Remap uncached memory accesses
  always_comb begin
    axi_rmp_req = axi_mem_req;
    axi_mem_rsp = axi_rmp_rsp;
    if ((axi_mem_req.aw.addr & ~DramUncachedMask) == DramUncachedBaseAddr) begin
      axi_rmp_req.aw.addr = DramBaseAddr | (axi_mem_req.aw.addr & DramUncachedMask);
    end
    if ((axi_mem_req.ar.addr & ~DramUncachedMask) == DramUncachedBaseAddr) begin
      axi_rmp_req.ar.addr = DramBaseAddr | (axi_mem_req.ar.addr & DramUncachedMask);
    end
  end

  // AXI simulated memory
  axi_sim_mem #(
    .AddrWidth          ( CVA6UserCfg.AxiAddrWidth ),
    .DataWidth          ( CVA6UserCfg.AxiDataWidth ),
    .IdWidth            ( AxiSlvIdWidth            ),
    .UserWidth          ( CVA6UserCfg.AxiUserWidth ),
    .axi_req_t          ( axi_slv_req_t            ),
    .axi_rsp_t          ( axi_slv_rsp_t            ),
    .WarnUninitialized  ( 0                        ),
    .ClearErrOnAccess   ( 1                        ),
  `ifdef ZERO_SIM_MEM
    .UninitializedData  ( "zeros"                  ),
  `else
    .UninitializedData  ( "undefined"              ),
  `endif
    .ApplDelay          ( ApplDelay                ),
    .AcqDelay           ( AcqDelay                 )
  ) i_axi_sim_mem (
    .clk_i              ( clk                      ),
    .rst_ni             ( rst_n                    ),
    .axi_req_i          ( axi_rmp_req              ),
    .axi_rsp_o          ( axi_rmp_rsp              ),
    .mon_w_valid_o      ( /* Not connected */      ),
    .mon_w_addr_o       ( /* Not connected */      ),
    .mon_w_data_o       ( /* Not connected */      ),
    .mon_w_id_o         ( /* Not connected */      ),
    .mon_w_user_o       ( /* Not connected */      ),
    .mon_w_beat_count_o ( /* Not connected */      ),
    .mon_w_last_o       ( /* Not connected */      ),
    .mon_r_valid_o      ( /* Not connected */      ),
    .mon_r_addr_o       ( /* Not connected */      ),
    .mon_r_data_o       ( /* Not connected */      ),
    .mon_r_id_o         ( /* Not connected */      ),
    .mon_r_user_o       ( /* Not connected */      ),
    .mon_r_beat_count_o ( /* Not connected */      ),
    .mon_r_last_o       ( /* Not connected */      )
  );

endmodule
