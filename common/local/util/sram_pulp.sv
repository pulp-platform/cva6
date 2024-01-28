// Copyright 2018 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.
//
// Author: Florian Zaruba    <zarubaf@iis.ee.ethz.ch>, ETH Zurich
//         Michael Schaffner <schaffner@iis.ee.ethz.ch>, ETH Zurich
//         Nils Wistoff      <nwistoff@iis.ee.ethz.ch>, ETH Zurich
// Date: 15.08.2018
// Description: generic tc_sram wrapper for CVA6
//
// Note: the wrapped module contains two different implementations for
// ALTERA and XILINX tools, since these follow different coding styles for
// inferrable RAMS with byte enable. define `FPGA_TARGET_XILINX or
// `FPGA_TARGET_ALTERA in your build environment (default is ALTERA)

module sram #(
    parameter DATA_WIDTH      = 64,
    parameter BYTE_WIDTH      = 8,
    parameter USER_WIDTH      = 1,
    parameter USER_EN         = 0,
    parameter NUM_WORDS       = 1024,
    parameter ENABLE_ECC      = 0, // Enable or disable SRAM with error correcting codes
    parameter ECC_GRANULARITY = 32,
    parameter ECC_ENCODING    = "Hsiao",
    parameter SIM_INIT        = "zeros",
    parameter OUT_REGS        = 0     // enables output registers in FPGA macro (read lat = 2)
)(
   input  logic                          clk_i,
   input  logic                          rst_ni,
   input  logic                          req_i,
   input  logic                          we_i,
   input  logic [$clog2(NUM_WORDS)-1:0]  addr_i,
   input  logic [USER_WIDTH-1:0]         wuser_i,
   input  logic [DATA_WIDTH-1:0]         wdata_i,
   input  logic [(DATA_WIDTH+BYTE_WIDTH-1)/BYTE_WIDTH-1:0] be_i,
   output logic [USER_WIDTH-1:0]         ruser_o,
   output logic [DATA_WIDTH-1:0]         rdata_o,
   output logic [1:0]                    error_o,
   output logic [1:0]                    user_error_o
);

if (ENABLE_ECC) begin: gen_ecc_sram

  function automatic int unsigned get_parity_width (input int unsigned data_width);
    int unsigned cw_width = 2;
    while (unsigned'(2**cw_width) < cw_width + data_width + 1) cw_width++;
    return cw_width;
  endfunction

  localparam int unsigned G = (ECC_ENCODING == "Hamming") ? DATA_WIDTH : ECC_GRANULARITY;
  localparam int unsigned NumBanks = DATA_WIDTH/G;

  localparam int unsigned K = get_parity_width(G) + 1;

  localparam int unsigned EccDataWidth = G + K;
  localparam int unsigned BeWidth = (G+BYTE_WIDTH-1)/BYTE_WIDTH;
  localparam int unsigned EccBeWidth = (EccDataWidth+BYTE_WIDTH-1)/BYTE_WIDTH;
  localparam int unsigned BeWidthDiff = EccBeWidth - BeWidth;

  logic [NumBanks-1:0][G-1:0] wdata, rdata, wuser, ruser;
  logic [NumBanks-1:0][EccDataWidth-1:0] ecc_wdata, ecc_rdata, ecc_wuser, ecc_ruser;
  logic [NumBanks-1:0][K-1:0] syndrome, user_syndrome;
  logic [NumBanks-1:0][1:0] ecc_error, user_ecc_error;
  logic [NumBanks-1:0] correctable_error, uncorrectable_error,
                       user_correctable_error, user_uncorrectable_error;
  logic [NumBanks-1:0][EccBeWidth-1:0] ecc_be;

  assign error_o[0] = |correctable_error;
  assign error_o[1] = |uncorrectable_error;
  assign user_error_o[0] = |user_correctable_error;
  assign user_error_o[1] = |user_uncorrectable_error;
  for (genvar i = 0; i < NumBanks; i++) begin: gen_data_split
    assign wdata[i] = wdata_i[G*i+:G];
    assign rdata_o[G*i+:G] = rdata[i];
    assign ecc_be[i] = {{BeWidthDiff{|be_i[i*BeWidth+:BeWidth]}}, be_i[i*BeWidth+:BeWidth]};
    assign correctable_error[i] = ecc_error[i][0];
    assign uncorrectable_error[i] = ecc_error[i][1];

    ecc_wrap #(
      .DataWidth   ( G            ),
      .ParityWidth ( K            ),
      .Encoding    ( ECC_ENCODING )
    ) i_ecc_wrap   (
      .wdata_i     ( wdata[i]     ),
      .wdata_o     ( ecc_wdata[i] ),
      .rdata_i     ( ecc_rdata[i] ),
      .rdata_o     ( rdata[i]     ),
      .syndrome_o  ( syndrome[i]  ),
      .error_o     ( ecc_error[i] )
    );

    tc_sram #(
      .NumWords    ( NUM_WORDS    ),
      .DataWidth   ( EccDataWidth ),
      .ByteWidth   ( BYTE_WIDTH   ),
      .NumPorts    ( 32'd1        ),
      .Latency     ( 32'd1        ),
      .SimInit     ( SIM_INIT     ),
      .PrintSimCfg ( 1'b0         )
    ) i_ecc_sram   (
      .clk_i       ( clk_i        ),
      .rst_ni      ( rst_ni       ),
      .req_i       ( req_i        ),
      .we_i        ( we_i         ),
      .be_i        ( ecc_be[i]    ),
      .wdata_i     ( ecc_wdata[i] ),
      .addr_i      ( addr_i       ),
      .rdata_o     ( ecc_rdata[i] )
    );

    if (USER_EN > 1) begin: gen_ecc_mem_user
      assign wuser[i] = wuser_i[G*i+:G];
      assign ruser_o[G*i+:G] = ruser[i];
      assign user_correctable_error[i] = user_ecc_error[i][0];
      assign user_uncorrectable_error[i] = user_ecc_error[i][1];
      
      ecc_wrap #(
        .DataWidth   ( G            ),
        .ParityWidth ( K            ),
        .Encoding    ( ECC_ENCODING )
      ) i_ecc_wrap   (
        .wdata_i     ( wuser[i]          ),
        .wdata_o     ( ecc_wuser[i]      ),
        .rdata_i     ( ecc_ruser[i]      ),
        .rdata_o     ( ruser[i]          ),
        .syndrome_o  ( user_syndrome[i]  ),
        .error_o     ( user_ecc_error[i] )
      );

      tc_sram #(
        .NumWords    ( NUM_WORDS    ),
        .DataWidth   ( EccDataWidth ),
        .ByteWidth   ( BYTE_WIDTH   ),
        .NumPorts    ( 32'd1        ),
        .Latency     ( 32'd1        ),
        .SimInit     ( SIM_INIT     ),
        .PrintSimCfg ( 1'b0         )
      ) i_ecc_sram   (
        .clk_i       ( clk_i        ),
        .rst_ni      ( rst_ni       ),
        .req_i       ( req_i        ),
        .we_i        ( we_i         ),
        .be_i        ( ecc_be[i]    ),
        .wdata_i     ( ecc_wuser[i] ),
        .addr_i      ( addr_i       ),
        .rdata_o     ( ecc_ruser[i] )
      );

      if (USER_WIDTH != DATA_WIDTH)
        $fatal(1, "sram_pulp: USER_WIDTH needs to be equal to DATA_WIDTH (if USER_EN is set).");
    end else begin
      assign user_correctable_error[i] = '0;
      assign user_uncorrectable_error[i] = '0;
    end
  end

end else begin: gen_standard_sram
  assign error_o = '0;
  assign user_error_o = '0;

  tc_sram #(
    .NumWords    ( NUM_WORDS  ),
    .DataWidth   ( DATA_WIDTH ),
    .ByteWidth   ( BYTE_WIDTH ),
    .NumPorts    ( 32'd1      ),
    .Latency     ( 32'd1      ),
    .SimInit     ( SIM_INIT   ),
    .PrintSimCfg ( 1'b0       )
  ) i_tc_sram (
    .clk_i   ( clk_i   ),
    .rst_ni  ( rst_ni  ),
    .req_i   ( req_i   ),
    .we_i    ( we_i    ),
    .be_i    ( be_i    ),
    .wdata_i ( wdata_i ),
    .addr_i  ( addr_i  ),
    .rdata_o ( rdata_o )
  );

  if (USER_EN > 0) begin : gen_mem_user
    tc_sram #(
      .NumWords    ( NUM_WORDS  ),
      .DataWidth   ( DATA_WIDTH ),
      .ByteWidth   ( BYTE_WIDTH ),
      .NumPorts    ( 32'd1      ),
      .Latency     ( 32'd1      ),
      .SimInit     ( SIM_INIT   ),
      .PrintSimCfg ( 1'b0       )
    ) i_tc_sram_user (
      .clk_i   ( clk_i   ),
      .rst_ni  ( rst_ni  ),
      .req_i   ( req_i   ),
      .we_i    ( we_i    ),
      .be_i    ( be_i    ),
      .wdata_i ( wuser_i ),
      .addr_i  ( addr_i  ),
      .rdata_o ( ruser_o )
    );

    if (USER_WIDTH != DATA_WIDTH) begin : gen_err_data_user_width
      $fatal(1, "sram_pulp: USER_WIDTH needs to be equal to DATA_WIDTH (if USER_EN is set).");
    end

  end else begin
    assign ruser_o = '0;
  end
end

endmodule : sram
