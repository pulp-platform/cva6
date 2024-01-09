// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// HSIAO SECDED Encoder/Decoder for 8-bit words

module secded_13_8_enc (
  input        [7:0] in,
  output logic [12:0] out
);

  always_comb begin : p_encode
    out[7:0] = in;
    out[8] = ^(in & 8'h6B);
    out[9] = ^(in & 8'hF8);
    out[10] = ^(in & 8'hD5);
    out[11] = ^(in & 8'hA7);
    out[12] = ^(in & 8'h1E);
  end

endmodule : secded_13_8_enc

module secded_13_8_dec (
  input        [12:0] in,
  output logic [7:0] d_o,
  output logic [4:0] syndrome_o,
  output logic [1:0] err_o
);

  logic single_error;

  // Syndrome calculation
  assign syndrome_o[0] = ^(in & 13'h016B);
  assign syndrome_o[1] = ^(in & 13'h02F8);
  assign syndrome_o[2] = ^(in & 13'h04D5);
  assign syndrome_o[3] = ^(in & 13'h08A7);
  assign syndrome_o[4] = ^(in & 13'h101E);

  // Corrected output calculation
  assign d_o[0] = (syndrome_o == 5'hd) ^ in[0];
  assign d_o[1] = (syndrome_o == 5'h19) ^ in[1];
  assign d_o[2] = (syndrome_o == 5'h1c) ^ in[2];
  assign d_o[3] = (syndrome_o == 5'h13) ^ in[3];
  assign d_o[4] = (syndrome_o == 5'h16) ^ in[4];
  assign d_o[5] = (syndrome_o == 5'hb) ^ in[5];
  assign d_o[6] = (syndrome_o == 5'h7) ^ in[6];
  assign d_o[7] = (syndrome_o == 5'he) ^ in[7];

  // err_o calc. bit0: single error, bit1: double error
  assign single_error = ^syndrome_o;
  assign err_o[0] = single_error;
  assign err_o[1] = ~single_error & (|syndrome_o);

endmodule : secded_13_8_dec
