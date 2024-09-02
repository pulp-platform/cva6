// Copyright (c) 2022  Bruno Sá and Zero-Day Labs.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Bruno Sá
// Date: 14/08/2022
// Acknowledges: Technology Innovation Institute (TII)
//
// Description: Translation Lookaside Buffer, Sv39x4 , fully set-associative
//              This module is an adaptation of the Sv39 TLB developed
//              by Florian Zaruba and David Schaffenrath to the Sv39x4 standard.

`include "common_cells/registers.svh"

module cva6_tlb_sv39x4
  import ariane_pkg::*;
#(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty,
    parameter int unsigned TLB_ENTRIES = 4,
    parameter int unsigned ASID_WIDTH = 1,
    parameter int unsigned VMID_WIDTH = 1,
    parameter bit          EccEnable = 1'b0
) (
    input logic clk_i,  // Clock
    input logic rst_ni,  // Asynchronous reset active low
    input logic clear_i,
    input logic flush_i,  // Flush normal translations signal
    input logic flush_vvma_i,  // Flush vs stage signal
    input logic flush_gvma_i,  // Flush g stage signal
    input logic s_st_enbl_i,  // s-stage enabled
    input logic g_st_enbl_i,  // g-stage enabled
    input logic v_i,  // virtualization mode
    // Update TLB
    input tlb_update_sv39x4_t update_i,
    // Lookup signals
    input logic lu_access_i,
    input logic [ASID_WIDTH-1:0] lu_asid_i,
    input logic [VMID_WIDTH-1:0] lu_vmid_i,
    input logic [riscv::VLEN-1:0] lu_vaddr_i,
    output logic [riscv::GPLEN-1:0] lu_gpaddr_o,
    output riscv::pte_t lu_content_o,
    output riscv::pte_t lu_g_content_o,
    input logic [ASID_WIDTH-1:0] asid_to_be_flushed_i,
    input logic [VMID_WIDTH-1:0] vmid_to_be_flushed_i,
    input logic [riscv::VLEN-1:0] vaddr_to_be_flushed_i,
    input logic [riscv::GPLEN-1:0] gpaddr_to_be_flushed_i,
    output logic lu_is_2M_o,
    output logic lu_is_1G_o,
    output logic lu_hit_o
);

  // SV39 defines three levels of page tables
  typedef struct packed {
    logic [ASID_WIDTH-1:0] asid;
    logic [VMID_WIDTH-1:0] vmid;
    logic [riscv::GPPN2:0] vpn2;
    logic [8:0]            vpn1;
    logic [8:0]            vpn0;
    logic                  is_s_2M;
    logic                  is_s_1G;
    logic                  is_g_2M;
    logic                  is_g_1G;
    logic                  s_st_enbl;  // s-stage translation
    logic                  g_st_enbl;  // g-stage translation
    logic                  v;          // virtualization mode
  } partial_tags_t;

  typedef struct packed {
    partial_tags_t tag;
    logic          valid;
  } tags_t;

  localparam int unsigned PteBits = $bits(riscv::pte_t);
  localparam int unsigned TagBits = $bits(partial_tags_t);
  localparam int unsigned ValidBits = TLB_ENTRIES; // One valid per entry

  localparam int unsigned PteCorrBits = EccEnable ? $clog2(PteBits) + 2 : 0;
  localparam int unsigned PteSize = PteBits + PteCorrBits;

  localparam int unsigned TagsCorrBits = EccEnable ? $clog2(TagBits) + 2 : 0;
  localparam int unsigned TagsSize = TagBits + TagsCorrBits;

  localparam int unsigned ValidCorrBits = EccEnable ? $clog2(ValidBits) + 2 : 0;
  localparam int unsigned ValidSize = ValidBits + ValidCorrBits;

  logic [ValidBits-1:0] valid_update, valid_dec;
  logic [ValidSize-1:0] valid_n, valid_q;

  tags_t [TLB_ENTRIES-1:0] tags;
  logic [TagBits-1:0] tags_update;
  logic [TagsSize-1:0] tags_enc;
  logic [TLB_ENTRIES-1:0][TagBits-1:0] tags_dec;
  logic [TLB_ENTRIES-1:0][TagsSize-1:0] tags_n, tags_q;

  struct packed {
    logic [PteSize-1:0] pte;
    logic [PteSize-1:0] gpte;
  } tlb_content_n;

  struct packed {
    logic [PteSize-1:0] pte;
    logic [PteSize-1:0] gpte;
  } [TLB_ENTRIES-1:0]
      content_q, content_n;

  struct packed {
    logic [PteBits-1:0] pte;
    logic [PteBits-1:0] gpte;
  } [TLB_ENTRIES-1:0]
      tlb_content_dec;

  struct packed {
    riscv::pte_t pte;
    riscv::pte_t gpte;
  } [TLB_ENTRIES-1:0]
      tlb_content_q;

  logic [8:0] vpn0, vpn1;
  logic [riscv::GPPN2:0] vpn2;
  logic [TLB_ENTRIES-1:0] lu_hit;  // to replacement logic
  logic [TLB_ENTRIES-1:0] replace_en;  // replace the following entry, set by replacement strategy
  logic [TLB_ENTRIES-1:0] match_vmid;
  logic [TLB_ENTRIES-1:0] match_asid;
  logic [TLB_ENTRIES-1:0] is_1G;
  logic [TLB_ENTRIES-1:0] is_2M;
  logic [TLB_ENTRIES-1:0] match_stage;
  riscv::pte_t g_content;

  assign tags_update = {
    update_i.asid,
    update_i.vmid,
    update_i.vpn[18+riscv::GPPN2:18],
    update_i.vpn[17:9],
    update_i.vpn[8:0],
    update_i.is_s_2M,
    update_i.is_s_1G,
    update_i.is_g_2M,
    update_i.is_g_1G,
    s_st_enbl_i,
    g_st_enbl_i,
    v_i
  };

  if (EccEnable) begin: gen_tlb_ecc
    hsiao_ecc_enc #(
      .DataWidth ( PteBits ),
      .ProtWidth ( PteCorrBits )
    ) i_ecc_pte_enc (
      .in  ( update_i.content ),
      .out ( tlb_content_n.pte)
    );

    hsiao_ecc_enc #(
      .DataWidth ( PteBits ),
      .ProtWidth ( PteCorrBits )
    ) i_ecc_gpte_enc (
      .in  ( update_i.g_content ),
      .out ( tlb_content_n.gpte)
    );

    hsiao_ecc_enc #(
      .DataWidth ( TagBits ),
      .ProtWidth ( TagsCorrBits  )
    ) i_ecc_tag_enc (
      .in  ( tags_update ),
      .out ( tags_enc   )
    );

    hsiao_ecc_enc #(
      .DataWidth ( ValidBits ),
      .ProtWidth ( ValidCorrBits )
    ) i_ecc_valid_enc (
      .in  ( valid_update ),
      .out ( valid_n   )
    );

    hsiao_ecc_dec #(
      .DataWidth ( ValidBits ),
      .ProtWidth ( ValidCorrBits )
    ) i_ecc_valid_dec (
      .in  ( valid_q ),
      .out ( valid_dec ),
      .syndrome_o (),
      .err_o ()
    );

    for (genvar i = 0; i < TLB_ENTRIES; i++) begin
      hsiao_ecc_dec #(
        .DataWidth ( PteBits ),
        .ProtWidth ( PteCorrBits )
      ) i_ecc_pte_dec (
        .in  (content_q[i].pte),
        .out (tlb_content_dec[i].pte),
        .syndrome_o (),
        .err_o ()
      );

      hsiao_ecc_dec #(
        .DataWidth ( PteBits ),
        .ProtWidth ( PteCorrBits )
      ) i_ecc_gpte_dec (
        .in  (content_q[i].gpte),
        .out (tlb_content_dec[i].gpte),
        .syndrome_o (),
        .err_o ()
      );

      hsiao_ecc_dec #(
        .DataWidth ( TagBits ),
        .ProtWidth ( TagsCorrBits )
      ) i_ecc_tag_dec (
        .in  ( tags_q[i] ),
        .out ( tags_dec[i] ),
        .syndrome_o (),
        .err_o ()
      );

    end
  end else begin: gen_no_tlb_ecc
    assign tlb_content_n.pte = update_i.content;
    assign tlb_content_n.gpte = update_i.g_content;
    assign tags_enc = tags_update;
    assign valid_n = valid_update;
    assign valid_dec = valid_q;
    for (genvar i = 0; i < TLB_ENTRIES; i++) begin
      assign tlb_content_dec[i].pte = content_q[i].pte;
      assign tlb_content_dec[i].gpte = content_q[i].gpte;
      assign tags_dec[i] = tags_q[i];
    end
  end

  for (genvar i = 0; i < TLB_ENTRIES; i++) begin
    assign tlb_content_q[i].pte = riscv::pte_t'(tlb_content_dec[i].pte);
    assign tlb_content_q[i].gpte = riscv::pte_t'(tlb_content_dec[i].gpte);
    assign tags[i].tag = partial_tags_t'(tags_dec[i]);
    assign tags[i].valid = valid_dec[i];
  end

  //-------------
  // Translation
  //-------------
  always_comb begin : translation
    automatic logic [riscv::GPPN2:0] mask_pn2;
    mask_pn2       = s_st_enbl_i ? ((2 ** (riscv::VPN2 + 1)) - 1) : ((2 ** (riscv::GPPN2 + 1)) - 1);
    vpn0           = lu_vaddr_i[20:12];
    vpn1           = lu_vaddr_i[29:21];
    vpn2           = lu_vaddr_i[30+riscv::GPPN2:30] & mask_pn2;

    // default assignment
    lu_hit         = '{default: 0};
    lu_hit_o       = 1'b0;
    lu_content_o   = '{default: 0};
    lu_g_content_o = '{default: 0};
    lu_is_1G_o     = 1'b0;
    lu_is_2M_o     = 1'b0;
    match_asid     = '{default: 0};
    match_vmid     = '{default: 0};
    match_stage    = '{default: 0};
    is_1G          = '{default: 0};
    is_2M          = '{default: 0};
    g_content      = '{default: 0};
    lu_gpaddr_o    = '{default: 0};


    for (int unsigned i = 0; i < TLB_ENTRIES; i++) begin
      // first level match, this may be a giga page, check the ASID flags as well
      // if the entry is associated to a global address, don't match the ASID (ASID is don't care)
      match_asid[i] = (((lu_asid_i == tags[i].tag.asid) || tlb_content_q[i].pte.g) && s_st_enbl_i) || !s_st_enbl_i;
      match_vmid[i] = (lu_vmid_i == tags[i].tag.vmid && g_st_enbl_i) || !g_st_enbl_i;
      is_1G[i] = is_trans_1G(s_st_enbl_i, g_st_enbl_i, tags[i].tag.is_s_1G, tags[i].tag.is_g_1G);
      is_2M[i] = is_trans_2M(
        s_st_enbl_i,
        g_st_enbl_i,
        tags[i].tag.is_s_1G,
        tags[i].tag.is_s_2M,
        tags[i].tag.is_g_1G,
        tags[i].tag.is_g_2M
      );
      // check if translation is a: S-Stage and G-Stage, S-Stage only or G-Stage only translation and virtualization mode is on/off
      match_stage[i] = (tags[i].tag.v == v_i) && (tags[i].tag.g_st_enbl == g_st_enbl_i) && (tags[i].tag.s_st_enbl == s_st_enbl_i);
      if (tags[i].valid && match_asid[i] && match_vmid[i] && match_stage[i] && (vpn2 == (tags[i].tag.vpn2 & mask_pn2))) begin
        lu_gpaddr_o = make_gpaddr(s_st_enbl_i, tags[i].tag.is_s_1G, tags[i].tag.is_s_2M, lu_vaddr_i,
                                  tlb_content_q[i].pte);
        if (is_1G[i]) begin
          lu_is_1G_o     = is_1G[i];
          lu_content_o   = tlb_content_q[i].pte;
          lu_g_content_o = tlb_content_q[i].gpte;
          lu_hit_o       = 1'b1;
          lu_hit[i]      = 1'b1;
          // not a giga page hit so check further
        end else if (vpn1 == tags[i].tag.vpn1) begin
          // this could be a 2 mega page hit or a 4 kB hit
          // output accordingly
          if (is_2M[i] || vpn0 == tags[i].tag.vpn0) begin
            lu_is_2M_o = is_2M[i];
            // Compute G-Stage PPN based on the gpaddr
            g_content  = tlb_content_q[i].gpte;
            if (tags[i].tag.is_g_2M) g_content.ppn[8:0] = lu_gpaddr_o[20:12];
            if (tags[i].tag.is_g_1G) g_content.ppn[17:0] = lu_gpaddr_o[29:12];
            // Output G-stage and S-stage content
            lu_g_content_o = g_content;
            lu_content_o   = tlb_content_q[i].pte;
            lu_hit_o       = 1'b1;
            lu_hit[i]      = 1'b1;
          end
        end
      end
    end
  end



  logic asid_to_be_flushed_is0;  // indicates that the ASID provided by SFENCE.VMA (rs2)is 0, active high
  logic vaddr_to_be_flushed_is0;  // indicates that the VADDR provided by SFENCE.VMA (rs1)is 0, active high
  logic vmid_to_be_flushed_is0;  // indicates that the VMID provided is 0, active high
  logic gpaddr_to_be_flushed_is0;  // indicates that the GPADDR provided is 0, active high
  logic [TLB_ENTRIES-1:0] vaddr_vpn0_match;
  logic [TLB_ENTRIES-1:0] vaddr_vpn1_match;
  logic [TLB_ENTRIES-1:0] vaddr_vpn2_match;
  logic [TLB_ENTRIES-1:0] gpaddr_gppn0_match;
  logic [TLB_ENTRIES-1:0] gpaddr_gppn1_match;
  logic [TLB_ENTRIES-1:0] gpaddr_gppn2_match;
  logic [TLB_ENTRIES-1:0][(riscv::GPPNW-1):0] gppn;


  assign asid_to_be_flushed_is0   = ~(|asid_to_be_flushed_i);
  assign vaddr_to_be_flushed_is0  = ~(|vaddr_to_be_flushed_i);
  assign vmid_to_be_flushed_is0   = ~(|vmid_to_be_flushed_i);
  assign gpaddr_to_be_flushed_is0 = ~(|gpaddr_to_be_flushed_i);

  // ------------------
  // Update and Flush
  // ------------------
  always_comb begin : update_flush
    tags_n = tags_q;
    content_n = content_q;
    valid_update = valid_dec;

    for (int unsigned i = 0; i < TLB_ENTRIES; i++) begin

      vaddr_vpn0_match[i] = (vaddr_to_be_flushed_i[20:12] == tags[i].tag.vpn0);
      vaddr_vpn1_match[i] = (vaddr_to_be_flushed_i[29:21] == tags[i].tag.vpn1);
      vaddr_vpn2_match[i] = (vaddr_to_be_flushed_i[30+riscv::VPN2:30] == tags[i].tag.vpn2[riscv::VPN2:0]);

      gppn[i] = make_gppn(
        tags[i].tag.s_st_enbl,
        tags[i].tag.is_s_1G,
        tags[i].tag.is_s_2M,
        {
          tags[i].tag.vpn2, tags[i].tag.vpn1, tags[i].tag.vpn0
        },
        tlb_content_q[i].pte
      );
      gpaddr_gppn0_match[i] = (gpaddr_to_be_flushed_i[20:12] == gppn[i][8:0]);
      gpaddr_gppn1_match[i] = (gpaddr_to_be_flushed_i[29:21] == gppn[i][17:9]);
      gpaddr_gppn2_match[i] = (gpaddr_to_be_flushed_i[30+riscv::GPPN2:30] == gppn[i][18+riscv::GPPN2:18]);

      if (flush_i) begin
        if (!tags[i].tag.v) begin
          // invalidate logic
          // flush everything if ASID is 0 and vaddr is 0 ("SFENCE.VMA x0 x0" case)
          if (asid_to_be_flushed_is0 && vaddr_to_be_flushed_is0) valid_update[i] = 1'b0;
          // flush vaddr in all addressing space ("SFENCE.VMA vaddr x0" case), it should happen only for leaf pages
          else if (asid_to_be_flushed_is0 && ((vaddr_vpn0_match[i] && vaddr_vpn1_match[i] && vaddr_vpn2_match[i]) || (vaddr_vpn2_match[i] && tags[i].tag.is_s_1G) || (vaddr_vpn1_match[i] && vaddr_vpn2_match[i] && tags[i].tag.is_s_2M) ) && (~vaddr_to_be_flushed_is0))
            valid_update[i] = 1'b0;
          // the entry is flushed if it's not global and asid and vaddr both matches with the entry to be flushed ("SFENCE.VMA vaddr asid" case)
          else if ((!tlb_content_q[i].pte.g) && ((vaddr_vpn0_match[i] && vaddr_vpn1_match[i] && vaddr_vpn2_match[i]) || (vaddr_vpn2_match[i] && tags[i].tag.is_s_1G) || (vaddr_vpn1_match[i] && vaddr_vpn2_match[i] && tags[i].tag.is_s_2M)) && (asid_to_be_flushed_i == tags[i].tag.asid) && (!vaddr_to_be_flushed_is0) && (!asid_to_be_flushed_is0))
            valid_update[i] = 1'b0;
          // the entry is flushed if it's not global, and the asid matches and vaddr is 0. ("SFENCE.VMA 0 asid" case)
          else if ((!tlb_content_q[i].pte.g) && (vaddr_to_be_flushed_is0) && (asid_to_be_flushed_i == tags[i].tag.asid) && (!asid_to_be_flushed_is0))
            valid_update[i] = 1'b0;
        end
      end else if (flush_vvma_i) begin
        if (tags[i].tag.v && tags[i].tag.s_st_enbl) begin
          // invalidate logic
          // flush everything if current VMID matches and ASID is 0 and vaddr is 0 ("SFENCE.VMA/HFENCE.VVMA x0 x0" case)
          if (asid_to_be_flushed_is0 && vaddr_to_be_flushed_is0 && ((tags[i].tag.g_st_enbl && lu_vmid_i == tags[i].tag.vmid) || !tags[i].tag.g_st_enbl))
            valid_update[i] = 1'b0;
          // flush vaddr in all addressing space if current VMID matches ("SFENCE.VMA/HFENCE.VVMA vaddr x0" case), it should happen only for leaf pages
          else if (asid_to_be_flushed_is0 && ((vaddr_vpn0_match[i] && vaddr_vpn1_match[i] && vaddr_vpn2_match[i]) || (vaddr_vpn2_match[i] && tags[i].tag.is_s_1G) || (vaddr_vpn1_match[i] && vaddr_vpn2_match[i] && tags[i].tag.is_s_2M) ) && (~vaddr_to_be_flushed_is0) && ((tags[i].tag.g_st_enbl && lu_vmid_i == tags[i].tag.vmid) || !tags[i].tag.g_st_enbl))
            valid_update[i] = 1'b0;
          // the entry is flushed if it's not global and asid and vaddr and current VMID matches with the entry to be flushed ("SFENCE.VMA/HFENCE.VVMA vaddr asid" case)
          else if ((!tlb_content_q[i].pte.g) && ((vaddr_vpn0_match[i] && vaddr_vpn1_match[i] && vaddr_vpn2_match[i]) || (vaddr_vpn2_match[i] && tags[i].tag.is_s_1G) || (vaddr_vpn1_match[i] && vaddr_vpn2_match[i] && tags[i].tag.is_s_2M)) && (asid_to_be_flushed_i == tags[i].tag.asid && ((tags[i].tag.g_st_enbl && lu_vmid_i == tags[i].tag.vmid) || !tags[i].tag.g_st_enbl)) && (!vaddr_to_be_flushed_is0) && (!asid_to_be_flushed_is0))
            valid_update[i] = 1'b0;
          // the entry is flushed if it's not global, and the asid and the current VMID matches and vaddr is 0. ("SFENCE.VMA/HFENCE.VVMA 0 asid" case)
          else if ((!tlb_content_q[i].pte.g) && (vaddr_to_be_flushed_is0) && (asid_to_be_flushed_i == tags[i].tag.asid && ((tags[i].tag.g_st_enbl && lu_vmid_i == tags[i].tag.vmid) || !tags[i].tag.g_st_enbl)) && (!asid_to_be_flushed_is0))
            valid_update[i] = 1'b0;
        end
      end else if (flush_gvma_i) begin
        if (tags[i].tag.g_st_enbl) begin
          // invalidate logic
          // flush everything if vmid is 0 and addr is 0 ("HFENCE.GVMA x0 x0" case)
          if (vmid_to_be_flushed_is0 && gpaddr_to_be_flushed_is0) valid_update[i] = 1'b0;
          // flush gpaddr in all addressing space ("HFENCE.GVMA gpaddr x0" case), it should happen only for leaf pages
          else if (vmid_to_be_flushed_is0 && ((gpaddr_gppn0_match[i] && gpaddr_gppn1_match[i] && gpaddr_gppn2_match[i]) || (gpaddr_gppn2_match[i] && tags[i].tag.is_g_1G) || (gpaddr_gppn1_match[i] && gpaddr_gppn2_match[i] && tags[i].tag.is_g_2M) ) && (~gpaddr_to_be_flushed_is0))
            valid_update[i] = 1'b0;
          // the entry vmid and gpaddr both matches with the entry to be flushed ("HFENCE.GVMA gpaddr vmid" case)
          else if (((gpaddr_gppn0_match[i] && gpaddr_gppn1_match[i] && gpaddr_gppn2_match[i]) || (gpaddr_gppn2_match[i] && tags[i].tag.is_g_1G) || (gpaddr_gppn1_match[i] && gpaddr_gppn2_match[i] && tags[i].tag.is_g_2M)) && (vmid_to_be_flushed_i == tags[i].tag.vmid) && (~gpaddr_to_be_flushed_is0) && (~vmid_to_be_flushed_is0))
            valid_update[i] = 1'b0;
          // the entry is flushed if the vmid matches and gpaddr is 0. ("HFENCE.GVMA 0 vmid" case)
          else if ((gpaddr_to_be_flushed_is0) && (vmid_to_be_flushed_i == tags[i].tag.vmid) && (!vmid_to_be_flushed_is0))
            valid_update[i] = 1'b0;
        end
        // normal replacement
      end else if (update_i.valid & replace_en[i]) begin
        // update tag array
        tags_n[i] = tags_enc;
        valid_update[i] = 1'b1;
        // and content as well
        content_n[i].pte = tlb_content_n.pte;
        content_n[i].gpte = tlb_content_n.gpte;
      end
    end
  end

  // -----------------------------------------------
  // PLRU - Pseudo Least Recently Used Replacement
  // -----------------------------------------------
  logic [2*(TLB_ENTRIES-1)-1:0] plru_tree_q, plru_tree_n;
  always_comb begin : plru_replacement
    plru_tree_n = plru_tree_q;
    // The PLRU-tree indexing:
    // lvl0        0
    //            / \
        //           /   \
        // lvl1     1     2
        //         / \   / \
        // lvl2   3   4 5   6
        //       / \ /\/\  /\
        //      ... ... ... ...
        // Just predefine which nodes will be set/cleared
        // E.g. for a TLB with 8 entries, the for-loop is semantically
        // equivalent to the following pseudo-code:
        // unique case (1'b1)
        // lu_hit[7]: plru_tree_n[0, 2, 6] = {1, 1, 1};
        // lu_hit[6]: plru_tree_n[0, 2, 6] = {1, 1, 0};
        // lu_hit[5]: plru_tree_n[0, 2, 5] = {1, 0, 1};
        // lu_hit[4]: plru_tree_n[0, 2, 5] = {1, 0, 0};
        // lu_hit[3]: plru_tree_n[0, 1, 4] = {0, 1, 1};
        // lu_hit[2]: plru_tree_n[0, 1, 4] = {0, 1, 0};
        // lu_hit[1]: plru_tree_n[0, 1, 3] = {0, 0, 1};
        // lu_hit[0]: plru_tree_n[0, 1, 3] = {0, 0, 0};
        // default: begin /* No hit */ end
        // endcase
        for (
        int unsigned i = 0; i < TLB_ENTRIES; i++
    ) begin
      automatic int unsigned idx_base, shift, new_index;
      // we got a hit so update the pointer as it was least recently used
      if (lu_hit[i] & lu_access_i) begin
        // Set the nodes to the values we would expect
        for (int unsigned lvl = 0; lvl < $clog2(TLB_ENTRIES); lvl++) begin
          idx_base = $unsigned((2 ** lvl) - 1);
          // lvl0 <=> MSB, lvl1 <=> MSB-1, ...
          shift = $clog2(TLB_ENTRIES) - lvl;
          // to circumvent the 32 bit integer arithmetic assignment
          new_index = ~((i >> (shift - 1)) & 32'b1);
          plru_tree_n[idx_base+(i>>shift)] = new_index[0];
        end
      end
    end
    // Decode tree to write enable signals
    // Next for-loop basically creates the following logic for e.g. an 8 entry
    // TLB (note: pseudo-code obviously):
    // replace_en[7] = &plru_tree_q[ 6, 2, 0]; //plru_tree_q[0,2,6]=={1,1,1}
    // replace_en[6] = &plru_tree_q[~6, 2, 0]; //plru_tree_q[0,2,6]=={1,1,0}
    // replace_en[5] = &plru_tree_q[ 5,~2, 0]; //plru_tree_q[0,2,5]=={1,0,1}
    // replace_en[4] = &plru_tree_q[~5,~2, 0]; //plru_tree_q[0,2,5]=={1,0,0}
    // replace_en[3] = &plru_tree_q[ 4, 1,~0]; //plru_tree_q[0,1,4]=={0,1,1}
    // replace_en[2] = &plru_tree_q[~4, 1,~0]; //plru_tree_q[0,1,4]=={0,1,0}
    // replace_en[1] = &plru_tree_q[ 3,~1,~0]; //plru_tree_q[0,1,3]=={0,0,1}
    // replace_en[0] = &plru_tree_q[~3,~1,~0]; //plru_tree_q[0,1,3]=={0,0,0}
    // For each entry traverse the tree. If every tree-node matches,
    // the corresponding bit of the entry's index, this is
    // the next entry to replace.
    for (int unsigned i = 0; i < TLB_ENTRIES; i += 1) begin
      automatic logic en;
      automatic int unsigned idx_base, shift, new_index;
      en = 1'b1;
      for (int unsigned lvl = 0; lvl < $clog2(TLB_ENTRIES); lvl++) begin
        idx_base = $unsigned((2 ** lvl) - 1);
        // lvl0 <=> MSB, lvl1 <=> MSB-1, ...
        shift = $clog2(TLB_ENTRIES) - lvl;

        // en &= plru_tree_q[idx_base + (i>>shift)] == ((i >> (shift-1)) & 1'b1);
        new_index = (i >> (shift - 1)) & 32'b1;
        if (new_index[0]) begin
          en &= plru_tree_q[idx_base+(i>>shift)];
        end else begin
          en &= ~plru_tree_q[idx_base+(i>>shift)];
        end
      end
      replace_en[i] = en;
    end
  end

  // sequential process
  `FFARNC(tags_q, tags_n, clear_i, '{default: 0}, clk_i, rst_ni)
  `FFARNC(valid_q, valid_n, clear_i, '{default: 0}, clk_i, rst_ni)
  `FFARNC(content_q, content_n, clear_i, '{default: 0}, clk_i, rst_ni)
  `FFARNC(plru_tree_q, plru_tree_n, clear_i, '{default: 0}, clk_i, rst_ni)

  //--------------
  // Sanity checks
  //--------------

  //pragma translate_off
`ifndef VERILATOR

  initial begin : p_assertions
    assert ((TLB_ENTRIES % 2 == 0) && (TLB_ENTRIES > 1))
    else begin
      $error("TLB size must be a multiple of 2 and greater than 1");
      $stop();
    end
    assert (ASID_WIDTH >= 1)
    else begin
      $error("ASID width must be at least 1");
      $stop();
    end
  end

  // Just for checking
  function int countSetBits(logic [TLB_ENTRIES-1:0] vector);
    automatic int count = 0;
    foreach (vector[idx]) begin
      count += vector[idx];
    end
    return count;
  endfunction

  assert property (@(posedge clk_i) (countSetBits(lu_hit) <= 1))
  else begin
    $error("More then one hit in TLB!");
    $stop();
  end
  assert property (@(posedge clk_i) (countSetBits(replace_en) <= 1))
  else begin
    $error("More then one TLB entry selected for next replace!");
    $stop();
  end

`endif
  //pragma translate_on

endmodule
