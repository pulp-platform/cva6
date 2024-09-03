// Copyright 2023 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Nils Wistoff <nwistoff@iis.ee.ethz.ch>

module cva6_clic_controller #(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty
) (
    input logic clk_i,
    input logic rst_ni,
    // from CSR file
    input riscv::priv_lvl_t priv_lvl_i,  // current privilege level
    input logic v_i,  // virtualization mode bit
    input ariane_pkg::irq_ctrl_t irq_ctrl_i,
    input logic [7:0] mintthresh_i,  // M-mode interrupt threshold
    input logic [7:0] sintthresh_i,  // S-mode interrupt threshold
    input logic [7:0] vsintthresh_i,  // VS-mode interrupt threshold
    input riscv::intstatus_rv_t mintstatus_i,  // interrupt status
    // from/to CLIC
    input logic clic_irq_valid_i,  // interrupt is valid
    input logic clic_irq_ready_i,  // interrupt is acknowledged
    input logic [$clog2(CVA6Cfg.CLICNumInterruptSrc)-1:0] clic_irq_id_i,  // interrupt ID
    input logic [7:0] clic_irq_level_i,  // interrupt level
    input riscv::priv_lvl_t clic_irq_priv_i,  // interrupt privilege level
    input logic clic_irq_v_i,  // interrupt virtualization bit (only for vCLIC)
    input logic [5:0] clic_irq_vsid_i,  // interrupt Virtual Supervisor ID (only for vCLIC)
    input logic clic_kill_req_i,  // kill request
    output logic clic_kill_ack_o,  // kill acknowledge
    // to ID stage
    output logic clic_irq_req_o,
    output riscv::xlen_t clic_irq_cause_o
);
  // -------
  // Trigger
  // -------
  // Check if the interrupt level of the current interrupt exceeds the current
  // irq threshold and global interrupt are enabled (otherwise it won't fire).
  // The effective interrupt threshold is the maximum of mintstatus.mil and
  // mintthresh, because interrupts with higher level have priority.
  logic [7:0] max_mthresh, max_sthresh, max_vsthresh;

  logic clic_irq_v;  // Interrupt target virtualization mode (only for vCLIC)

  logic sgeie;  // Guest external interrupts at hypervisor level enable (HIE register)

  assign max_mthresh = mintthresh_i > mintstatus_i.mil ? mintthresh_i : mintstatus_i.mil;
  assign max_sthresh = sintthresh_i > mintstatus_i.sil ? sintthresh_i : mintstatus_i.sil;
  assign max_vsthresh = vsintthresh_i > mintstatus_i.vsil ? vsintthresh_i : mintstatus_i.vsil;

  assign sgeie = (CVA6Cfg.RVH) ? irq_ctrl_i.mie[riscv::IRQ_HS_EXT] : 1'b0;

  // Determine if CLIC interrupt shall be accepted
  always_comb begin : clic_irq_accept
    clic_irq_req_o = 1'b0;
    clic_irq_v = 1'b0;
    unique case (priv_lvl_i)
      riscv::PRIV_LVL_M: begin
        // Take M-mode interrupts with higher level
        if (clic_irq_priv_i == riscv::PRIV_LVL_M) begin
          clic_irq_req_o = (clic_irq_level_i > max_mthresh) && (clic_irq_valid_i);
        end
      end
      riscv::PRIV_LVL_S: begin
        // Take all M-mode interrupts
        if (clic_irq_priv_i == riscv::PRIV_LVL_M) begin
          clic_irq_req_o = clic_irq_valid_i;
          // Take S-mode interrupts with higher level
        end else if (clic_irq_priv_i == riscv::PRIV_LVL_S) begin
          if (CVA6Cfg.RVH && v_i) begin  // Hart currently in VS-mode
            if (CVA6Cfg.RVVCLIC && clic_irq_v_i) begin  // Virtual supervisor interrrupt
              if (clic_irq_vsid_i == irq_ctrl_i.vgein) begin // VS-mode interrupt is for currently running VS
                clic_irq_req_o = (clic_irq_level_i > max_vsthresh) && (clic_irq_valid_i) && irq_ctrl_i.sie;
                clic_irq_v = 1'b1;
              end else begin  // Received interrupt is delegated to a differet VS
                // Trap to HS-mode iff HIE.sgeie is set and HGEIE[vsid] is set
                clic_irq_req_o = (clic_irq_valid_i) && sgeie && irq_ctrl_i.hgeie[clic_irq_vsid_i];
              end
            end else begin  // Hypervisor interrupt
              clic_irq_req_o = (clic_irq_level_i > max_sthresh) && (clic_irq_valid_i); // HS-mode sie is implicitly enabled in VS-mode
            end
          end else begin  // Hart currently in (H)S-mode
            if (CVA6Cfg.RVVCLIC && clic_irq_v_i) begin  // Virtual supervisor interrrupt
              // The current custom vCLIC implementation does not provide a way to the hypervisor to set interrupt levels 
              // to interrupts delegated to VS-mode. The incoming interrupt level is therefore ignored if the hart is running
              // in HS-mode and a VS-mode interrupt is taken iff both HIE.sgeie and HGEIE[vsid] bits are set (and interrupts 
              // are globally enabled at supervisor level (i.e. MSTATUS.sie bit is set)
              clic_irq_req_o = (clic_irq_valid_i) && sgeie && irq_ctrl_i.hgeie[clic_irq_vsid_i] && irq_ctrl_i.sie;
            end else begin  // (Host) Supervisor interrupt
              clic_irq_req_o = (clic_irq_level_i > max_sthresh) && (clic_irq_valid_i) && irq_ctrl_i.sie;
            end
          end
        end
      end
      riscv::PRIV_LVL_U: begin
        // Take all M-mode interrupts
        if (clic_irq_priv_i == riscv::PRIV_LVL_M) begin
          clic_irq_req_o = clic_irq_valid_i;
        end else if (clic_irq_priv_i == riscv::PRIV_LVL_S) begin
          if (CVA6Cfg.RVH && v_i) begin  // Hart currently in VU-mode
            if (CVA6Cfg.RVVCLIC && clic_irq_v_i) begin  // Virtual supervisor interrrupt
              if (clic_irq_vsid_i == irq_ctrl_i.vgein) begin // VS-mode interrupt is for currently running VS
                clic_irq_req_o = clic_irq_valid_i;
                clic_irq_v = 1'b1;
              end else begin  // Received interrupt is delegated to a differet VS
                // Trap to HS-mode iff HIE.sgeie is set and HGEIE[vsid] is set
                clic_irq_req_o = (clic_irq_valid_i) && sgeie && irq_ctrl_i.hgeie[clic_irq_vsid_i];
              end
            end else begin  // Hypervisor interrupt
              clic_irq_req_o = clic_irq_valid_i;  // MSTATUS.sie is implicitly enabled in VU-mode
            end
          end else begin  // Hart currently in U-mode
            if (CVA6Cfg.RVVCLIC && clic_irq_v_i) begin  // Virtual supervisor interrrupt
              clic_irq_req_o = (clic_irq_valid_i) && sgeie && irq_ctrl_i.hgeie[clic_irq_vsid_i]; // MSTATUS.sie is implicitly enabled in U-mode
            end else begin  // (Host) Supervisor interrupt
              clic_irq_req_o = clic_irq_valid_i;  // HS-mode sie is implicitly enabled in U-mode
            end
          end
        end
      end
      default: ;
    endcase
  end

  // ------------------
  // Interrupt Packager
  // ------------------
  // Pack interrupt cause to be inserted into the pipeline
  assign clic_irq_cause_o = {
    1'b1,  // This is an irq
    {riscv::XLEN - 28{1'b0}},  // XLEN-2...27
    clic_irq_v,  // 26: interrupt target virtualization mode
    clic_irq_priv_i,  // 25..24: interrupt target privilege level
    clic_irq_level_i,  // to mintstatus.mil
    {16 - $clog2(CVA6Cfg.CLICNumInterruptSrc) {1'b0}},  // 15...IDWidth
    clic_irq_id_i
  };  // to mcause

  // ------------
  // Kill Control
  // ------------
  // Track whether an irq was inserted into the pipeline (accepted) but not acknowlegded yet
  logic irq_inflight_d, irq_inflight_q;
  // Set when accepting irq, clear when acknowledging or killing
  assign irq_inflight_d  = irq_inflight_q ? ~(clic_irq_ready_i | clic_kill_ack_o) : clic_irq_req_o;
  // Acknowledge kill if no irq is inflight and irq is not accepted this cycle
  assign clic_kill_ack_o = clic_kill_req_i & ~irq_inflight_q & ~clic_irq_req_o;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      irq_inflight_q <= 1'b0;
    end else begin
      irq_inflight_q <= irq_inflight_d;
    end
  end
endmodule
