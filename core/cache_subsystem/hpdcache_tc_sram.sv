// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Paul Scheffler <paulsc@iis.ee.ethz.ch>

// Wrappers mapping HPDcache SRAMs to PULP `tc_sram`.

/// SRAM with one R/W port and no write mask
module hpdcache_sram_1rw #(
    parameter int unsigned ADDR_SIZE = 0,
    parameter int unsigned DATA_SIZE = 0,
    parameter int unsigned DEPTH = 2**ADDR_SIZE
) (
    input  logic                  clk,
    input  logic                  rst_n,
    input  logic                  cs,
    input  logic                  we,
    input  logic [ADDR_SIZE-1:0]  addr,
    input  logic [DATA_SIZE-1:0]  wdata,
    output logic [DATA_SIZE-1:0]  rdata
);
    tc_sram #(
        .NumWords    ( DEPTH ),
        .DataWidth   ( DATA_SIZE ),
        .ByteWidth   ( DATA_SIZE ),
        .NumPorts    ( 1 ),
        .Latency     ( 1 )
    ) i_tc_sram (
        .clk_i   ( clk ),
        .rst_ni  ( rst_n ),
        .req_i   ( cs ),
        .we_i    ( we ),
        .addr_i  ( addr ),
        .wdata_i ( wdata ),
        .be_i    ( 1'b1 ),
        .rdata_o ( rdata )
    );
endmodule

/// SRAM with one R/W port and per-byte write mask
module hpdcache_sram_wbyteenable_1rw #(
    parameter int unsigned ADDR_SIZE = 0,
    parameter int unsigned DATA_SIZE = 0,
    parameter int unsigned DEPTH = 2**ADDR_SIZE
) (
    input  logic                   clk,
    input  logic                   rst_n,
    input  logic                   cs,
    input  logic                   we,
    input  logic [ADDR_SIZE-1:0]   addr,
    input  logic [DATA_SIZE-1:0]   wdata,
    input  logic [DATA_SIZE/8-1:0] wbyteenable,
    output logic [DATA_SIZE-1:0]   rdata
);
    tc_sram #(
        .NumWords    ( DEPTH ),
        .DataWidth   ( DATA_SIZE ),
        .ByteWidth   ( 8 ),
        .NumPorts    ( 1 ),
        .Latency     ( 1 )
    ) i_tc_sram (
        .clk_i   ( clk ),
        .rst_ni  ( rst_n ),
        .req_i   ( cs ),
        .we_i    ( we ),
        .addr_i  ( addr ),
        .wdata_i ( wdata ),
        .be_i    ( wbyteenable ),
        .rdata_o ( rdata )
    );
endmodule

/// SRAM with one R/W port and per-bit write mask
module hpdcache_sram_wmask_1rw #(
    parameter int unsigned ADDR_SIZE = 0,
    parameter int unsigned DATA_SIZE = 0,
    parameter int unsigned DEPTH = 2**ADDR_SIZE
) (
    input  logic                  clk,
    input  logic                  rst_n,
    input  logic                  cs,
    input  logic                  we,
    input  logic [ADDR_SIZE-1:0]  addr,
    input  logic [DATA_SIZE-1:0]  wdata,
    input  logic [DATA_SIZE-1:0]  wmask,
    output logic [DATA_SIZE-1:0]  rdata
);
    tc_sram #(
        .NumWords    ( DEPTH ),
        .DataWidth   ( DATA_SIZE ),
        .ByteWidth   ( 1 ),
        .NumPorts    ( 1 ),
        .Latency     ( 1 )
    ) i_tc_sram (
        .clk_i   ( clk ),
        .rst_ni  ( rst_n ),
        .req_i   ( cs ),
        .we_i    ( we ),
        .addr_i  ( addr ),
        .wdata_i ( wdata ),
        .be_i    ( wmask ),
        .rdata_o ( rdata )
    );
endmodule
