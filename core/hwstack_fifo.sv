// Copyright 2026 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License. You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>

`include "common_cells/assertions.svh"

module hwstack_fifo #(
    parameter config_pkg::cva6_cfg_t CVA6Cfg   = config_pkg::cva6_cfg_empty,
    // DO NOT OVERWRITE THIS PARAMETER
    parameter int unsigned           Depth     = CVA6Cfg.HwstackFifoDepth,
    parameter type                   dtype     = logic [CVA6Cfg.XLEN-1:0],
    parameter int unsigned           AddrDepth = (CVA6Cfg.HwstackFifoDepth > 1) ? $clog2(CVA6Cfg.HwstackFifoDepth) : 1
)(
    input  logic             clk_i,
    input  logic             rst_ni,
    output logic             full_o,
    output logic             empty_o,
    input  logic             load_i,
    input  dtype [Depth-1:0] load_data_i,
    input  dtype             data_i,
    input  logic             push_i,
    output dtype             data_o,
    input  logic             pop_i
);

    // FIFO depth
    localparam int unsigned FifoDepth = (Depth > 0) ? Depth : 1;
    // clock gating control
    logic gate_clock;
    // pointer to the read and write section of the queue
    logic [AddrDepth-1:0] read_pointer_n, read_pointer_q, write_pointer_n, write_pointer_q;
    // keep a counter to keep track of the current queue status
    // this integer will be truncated by the synthesis tool
    logic [AddrDepth:0] status_cnt_n, status_cnt_q;
    // actual memory
    dtype [FifoDepth-1:0] mem_n, mem_q;

    if (Depth == 0) begin : gen_pass_through
      assign empty_o     = ~push_i;
      assign full_o      = ~pop_i;
    end else begin : gen_fifo
      assign full_o       = (status_cnt_q == FifoDepth[AddrDepth:0]);
      assign empty_o      = (status_cnt_q == 0);
    end

    always_comb begin : read_write_comb
        // default assignment
        read_pointer_n  = read_pointer_q;
        write_pointer_n = write_pointer_q;
        status_cnt_n    = status_cnt_q;
        data_o          = (Depth == 0) ? data_i : mem_q[read_pointer_q];
        mem_n           = mem_q;
        gate_clock      = 1'b1;

        // push a new element to the queue
        if (push_i && ~full_o) begin
            mem_n[write_pointer_q] = data_i;
            gate_clock = 1'b0;
            if (write_pointer_q == FifoDepth[AddrDepth-1:0] - 1)
                write_pointer_n = '0;
            else
                write_pointer_n = write_pointer_q + 1;
            status_cnt_n    = status_cnt_q + 1;
        end

        // pop an element from the queue
        if (pop_i && ~empty_o) begin
            if (read_pointer_n == FifoDepth[AddrDepth-1:0] - 1)
                read_pointer_n = '0;
            else
                read_pointer_n = read_pointer_q + 1;
            status_cnt_n   = status_cnt_q - 1;
        end

        // keep the count pointer stable if we push and pop at the same time
        if (push_i && pop_i &&  ~full_o && ~empty_o)
            status_cnt_n   = status_cnt_q;
    end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if(~rst_ni) begin
      read_pointer_q  <= '0;
      write_pointer_q <= '0;
      status_cnt_q    <= '0;
    end else if (load_i) begin
      read_pointer_q  <= '0;
      write_pointer_q <= '0;
      status_cnt_q    <= Depth;
    end else begin
      read_pointer_q  <= read_pointer_n;
      write_pointer_q <= write_pointer_n;
      status_cnt_q    <= status_cnt_n;
    end
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if(~rst_ni) begin
      mem_q <= {FifoDepth{dtype'('0)}};
    end else if (load_i) begin
      mem_q <= load_data_i;
    end else if (!gate_clock) begin
      mem_q <= mem_n;
    end
  end

`ifndef COMMON_CELLS_ASSERTS_OFF
    `ASSERT_INIT(depth_0, Depth > 0, "Depth must be greater than 0.")

    `ASSERT(full_write, full_o |-> ~push_i, clk_i, !rst_ni,
            "Trying to push new data although the FIFO is full.")

    `ASSERT(empty_read, empty_o |-> ~pop_i, clk_i, !rst_ni,
            "Trying to pop data although the FIFO is empty.")
`endif

endmodule
