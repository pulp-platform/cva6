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

    // used in axi_adapter.sv
    typedef enum logic { SINGLE_REQ, CACHE_LINE_REQ } ad_req_t;
    typedef enum logic[2:0] { READ_SHARED, READ_ONCE, READ_UNIQUE, READ_NO_SNOOP, CLEAN_UNIQUE, WRITE_UNIQUE, WRITE_NO_SNOOP, WRITEBACK } ace_req_t;

    // AW Channel
    typedef struct packed {
        ariane_axi::id_t    id;
        ariane_axi::addr_t  addr;
        axi_pkg::len_t      len;
        axi_pkg::size_t     size;
        axi_pkg::burst_t    burst;
        logic               lock;
        axi_pkg::cache_t    cache;
        axi_pkg::prot_t     prot;
        axi_pkg::qos_t      qos;
        axi_pkg::region_t   region;
        axi_pkg::atop_t     atop;
        ariane_axi::user_t  user;
        ace_pkg::awsnoop_t  snoop;
        ace_pkg::bar_t      bar;
        ace_pkg::domain_t   domain;
        ace_pkg::awunique_t awunique;
    } aw_chan_t;

    // AR Channel
    typedef struct packed {
        ariane_axi::id_t   id;
        ariane_axi::addr_t addr;
        axi_pkg::len_t     len;
        axi_pkg::size_t    size;
        axi_pkg::burst_t   burst;
        logic              lock;
        axi_pkg::cache_t   cache;
        axi_pkg::prot_t    prot;
        axi_pkg::qos_t     qos;
        axi_pkg::region_t  region;
        ariane_axi::user_t user;
        ace_pkg::arsnoop_t snoop;
        ace_pkg::bar_t     bar;
        ace_pkg::domain_t  domain;
    } ar_chan_t;

    // R Channel
    typedef struct packed {
        ariane_axi::id_t   id;
        ariane_axi::data_t data;
        ace_pkg::rresp_t   resp;
        logic              last;
        ariane_axi::user_t user;
    } r_chan_t;

    // AC Channel
    typedef struct packed {
        ariane_axi::addr_t   addr;
        snoop_pkg::acsnoop_t snoop;
        snoop_pkg::acprot_t  prot;
    } ac_chan_t;

    // CD Cannel
    typedef struct packed {
        ariane_axi::data_t data;
        logic              last;
    } cd_chan_t;


    // W and B channel remain untouched compared to AXI

    // Request/Response structs
    typedef struct packed {
        ariane_ace::aw_chan_t aw;
        logic                 aw_valid;
        ariane_axi::w_chan_t  w;
        logic                 w_valid;
        logic                 b_ready;
        ariane_ace::ar_chan_t ar;
        logic                 ar_valid;
        logic                 r_ready;
        logic                 wack;
        logic                 rack;
        logic                 ac_ready;
        logic                 cr_valid;
        snoop_pkg::crresp_t   cr_resp;
        logic                 cd_valid;
        cd_chan_t             cd;
    } m2s_t;

    typedef struct packed {
        ariane_ace::aw_chan_t aw;
        logic                 aw_valid;
        ariane_axi::w_chan_t  w;
        logic                 w_valid;
        logic                 b_ready;
        ariane_ace::ar_chan_t ar;
        logic                 ar_valid;
        logic                 r_ready;
        logic                 wack;
        logic                 rack;
    } m2s_nosnoop_t;

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
        ariane_axi::b_chan_t b;
        logic                r_valid;
        ariane_ace::r_chan_t r;
        ac_chan_t            ac;
        logic                ac_valid;
        logic                cr_ready;
        logic                cd_ready;
    } s2m_t;

    typedef struct  packed {
        logic                aw_ready;
        logic                ar_ready;
        logic                w_ready;
        logic                b_valid;
        ariane_axi::b_chan_t b;
        logic                r_valid;
        ariane_ace::r_chan_t r;
    } s2m_nosnoop_t;

    typedef struct  packed {
        logic               ac_ready;
        logic               cr_valid;
        snoop_pkg::crresp_t cr_resp;
        logic               cd_valid;
        cd_chan_t           cd;
    } snoop_resp_t;

endpackage
