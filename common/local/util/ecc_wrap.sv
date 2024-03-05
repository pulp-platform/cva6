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
  localparam int unsigned EccDataWidth = DataWidth + ParityWidth
)(
  input  logic [DataWidth-1:0]    wdata_i,
  output logic [EccDataWidth-1:0] wdata_o,
  input  logic [EccDataWidth-1:0] rdata_i,
  output logic [DataWidth-1:0]    rdata_o,
  output logic [ParityWidth-1:0]  syndrome_o,
  output logic [1:0]              error_o
);

hsiao_ecc_enc #(
  .DataWidth  ( DataWidth    ),
  .ProtWidth  ( ParityWidth  ),
  .TotalWidth ( EccDataWidth )
) i_ecc_encoder (
  .in  ( wdata_i ),
  .out ( wdata_o )
);

hsiao_ecc_dec #(
  .DataWidth  ( DataWidth    ),
  .ProtWidth  ( ParityWidth  ),
  .TotalWidth ( EccDataWidth )
) i_ecc_decoder (
  .in         ( rdata_i    ),
  .d_o        ( rdata_o    ),
  .syndrome_o ( syndrome_o ),
  .err_o      ( error_o    )
);

endmodule: ecc_wrap
