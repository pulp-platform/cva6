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
// Author: Yvan Tortorella    <yvan.tortorella@unibo.it>, University of Bologna
// Description: parametric ECC wrapper for SEC-DED in CVA6 SRAMs

module ecc_wrap #(
  parameter  int unsigned DataWidth    = 32,
  parameter  int unsigned ParityWidth  = 7 ,
  parameter               Encoding     = "Hsiao",
  localparam int unsigned EccDataWidth = DataWidth + ParityWidth
)(
  input  logic [DataWidth-1:0]    wdata_i,
  output logic [EccDataWidth-1:0] wdata_o,
  input  logic [EccDataWidth-1:0] rdata_i,
  output logic [DataWidth-1:0]    rdata_o,
  output logic [ParityWidth-1:0]  syndrome_o,
  output logic [1:0]              error_o
);

if (Encoding == "Hsiao") begin: gen_hsiao_ecc
  if (DataWidth == 8) begin: gen_13bit_secded
    secded_13_8_enc i_ecc_encoder (
      .in  ( wdata_i ),
      .out ( wdata_o )
    );

    secded_13_8_dec i_ecc_decoder (
      .in         ( rdata_i    ),
      .d_o        ( rdata_o    ),
      .syndrome_o ( syndrome_o ),
      .err_o      ( error_o    )
    );
  end else if (DataWidth == 32) begin: gen_39bit_secded
    secded_39_32_enc i_ecc_encoder (
      .in  ( wdata_i ),
      .out ( wdata_o )
    );

    secded_39_32_dec i_ecc_decoder (
      .in         ( rdata_i    ),
      .d_o        ( rdata_o    ),
      .syndrome_o ( syndrome_o ),
      .err_o      ( error_o    )
    );
  end else if (DataWidth == 64) begin: gen_72bit_secded
    secded_72_64_enc i_ecc_encoder (
      .in  ( wdata_i ),
      .out ( wdata_o )
    );

    secded_72_64_dec i_ecc_decoder (
      .in         ( rdata_i    ),
      .d_o        ( rdata_o    ),
      .syndrome_o ( syndrome_o ),
      .err_o      ( error_o    )
    );
  end
end else if (Encoding == "Hamming") begin: gen_hamming_ecc
  logic [ParityWidth-2:0] syndrome;
  ecc_encode     #(
    .DataWidth    ( DataWidth )
  ) i_ecc_encoder (
    .data_i       ( wdata_i   ),
    .data_o       ( wdata_o   )
  );

  ecc_decode       #(
    .DataWidth      ( DataWidth  )
  ) i_ecc_decoder   (
    .data_i         ( rdata_i    ),
    .data_o         ( rdata_o    ),
    .syndrome_o     ( syndrome   ),
    .single_error_o ( error_o[0] ),
    .parity_error_o (            ),
    .double_error_o ( error_o[1] )
  );

  assign syndrome_o = {rdata_i[EccDataWidth-1], syndrome};
end

endmodule: ecc_wrap
