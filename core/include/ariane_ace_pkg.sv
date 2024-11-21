/* Copyright 2018 ETH Zurich and University of Bologna.
 * Copyright 2022 PlanV GmbH
 * Copyright and related rights are licensed under the Solderpad Hardware
 * License, Version 0.51 (the “License”); you may not use this file except in
 * compliance with the License.  You may obtain a copy of the License at
 * http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
 * or agreed to in writing, software, hardware and materials distributed under
 * this License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations under the License.
 */

package ariane_ace;

  typedef logic [cva6_config_pkg::CVA6ConfigAxiIdWidth-1:0] ariane_axi_id_t;
  typedef logic [cva6_config_pkg::CVA6ConfigAxiAddrWidth-1:0] ariane_axi_addr_t;
  typedef logic [cva6_config_pkg::CVA6ConfigAxiDataWidth-1:0] ariane_axi_data_t;
  typedef logic [cva6_config_pkg::CVA6ConfigDataUserWidth-1:0] ariane_axi_user_t;
  typedef logic [(cva6_config_pkg::CVA6ConfigDataUserWidth/8)-1:0] ariane_axi_strb_t;

  typedef struct packed {
    ariane_axi_data_t data;
    ariane_axi_strb_t strb;
    logic             last;
    ariane_axi_user_t user;
  } ariane_axi_w_chan_t;

  typedef struct packed {
    ariane_axi_id_t   id;
    axi_pkg::resp_t   resp;
    ariane_axi_user_t user;
  } ariane_axi_b_chan_t;

  // AW Channel
  typedef struct packed {
    ariane_axi_id_t     id;
    ariane_axi_addr_t   addr;
    axi_pkg::len_t      len;
    axi_pkg::size_t     size;
    axi_pkg::burst_t    burst;
    logic               lock;
    axi_pkg::cache_t    cache;
    axi_pkg::prot_t     prot;
    axi_pkg::qos_t      qos;
    axi_pkg::region_t   region;
    axi_pkg::atop_t     atop;
    ariane_axi_user_t   user;
    ace_pkg::awsnoop_t  snoop;
    ace_pkg::axbar_t    bar;
    ace_pkg::axdomain_t domain;
    ace_pkg::awunique_t awunique;
  } aw_chan_t;

  // AR Channel
  typedef struct packed {
    ariane_axi_id_t     id;
    ariane_axi_addr_t   addr;
    axi_pkg::len_t      len;
    axi_pkg::size_t     size;
    axi_pkg::burst_t    burst;
    logic               lock;
    axi_pkg::cache_t    cache;
    axi_pkg::prot_t     prot;
    axi_pkg::qos_t      qos;
    axi_pkg::region_t   region;
    ariane_axi_user_t   user;
    ace_pkg::arsnoop_t  snoop;
    ace_pkg::axbar_t    bar;
    ace_pkg::axdomain_t domain;
  } ar_chan_t;

  // R Channel
  typedef struct packed {
    ariane_axi_id_t    id;
    ariane_axi_data_t  data;
    ace_pkg::rresp_t   resp;
    logic              last;
    ariane_axi_user_t  user;
  } r_chan_t;

  // AC Channel
  typedef struct packed {
    ariane_axi_addr_t  addr;
    ace_pkg::acsnoop_t snoop;
    ace_pkg::acprot_t  prot;
  } ac_chan_t;

  // CD Cannel
  typedef struct packed {
    ariane_axi_data_t  data;
    logic              last;
  } cd_chan_t;


  // W and B channel remain untouched compared to AXI

  // Request/Response structs
  typedef struct packed {
    ariane_ace::aw_chan_t aw;
    logic                 aw_valid;
    ariane_axi_w_chan_t   w;
    logic                 w_valid;
    logic                 b_ready;
    ariane_ace::ar_chan_t ar;
    logic                 ar_valid;
    logic                 r_ready;
    logic                 wack;
    logic                 rack;
    // Snoop signals are reversed w.r.t. request / response
    logic                 ac_ready;
    logic                 cr_valid;
    ace_pkg::crresp_t     cr_resp;
    logic                 cd_valid;
    cd_chan_t             cd;
  } req_t;

  typedef struct packed {
    ariane_ace::aw_chan_t aw;
    logic                 aw_valid;
    ariane_axi_w_chan_t   w;
    logic                 w_valid;
    logic                 b_ready;
    ariane_ace::ar_chan_t ar;
    logic                 ar_valid;
    logic                 r_ready;
    logic                 wack;
    logic                 rack;
  } req_nosnoop_t;

  typedef struct packed {
    ac_chan_t ac;
    logic     ac_valid;
    logic     cr_ready;
    logic     cd_ready;
  } snoop_req_t;

  typedef struct packed {
    logic                aw_ready;
    logic                ar_ready;
    logic                w_ready;
    logic                b_valid;
    ariane_axi_b_chan_t  b;
    logic                r_valid;
    ariane_ace::r_chan_t r;
    // Snoop signals are reversed w.r.t. request / response
    ac_chan_t            ac;
    logic                ac_valid;
    logic                cr_ready;
    logic                cd_ready;
  } resp_t;

  typedef struct  packed {
    logic                aw_ready;
    logic                ar_ready;
    logic                w_ready;
    logic                b_valid;
    ariane_axi_b_chan_t  b;
    logic                r_valid;
    ariane_ace::r_chan_t r;
  } resp_nosnoop_t;

  typedef struct  packed {
    logic               ac_ready;
    logic               cr_valid;
    ace_pkg::crresp_t   cr_resp;
    logic               cd_valid;
    cd_chan_t           cd;
  } snoop_resp_t;

endpackage
