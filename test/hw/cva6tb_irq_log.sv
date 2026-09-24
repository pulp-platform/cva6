// Copyright 2026 ETH Zurich and University of Bologna.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// Interrupt latency logger.
//
// Tracks one interrupt at a time through six events -- line asserted, irq_valid,
// irq_ack, first commit after the pipeline redirect, first commit at the ISR
// entry point, and eret -- and streams one CSV row per interrupt as it
// completes.
//
// Two properties this module is built around:
//
//   * ONE RECORD PER INTERRUPT. The previous implementation pushed each event
//     type into its own queue and zipped the six queues by index at `final`.
//     Any event that fired an unequal number of times -- a spurious eret, an
//     interrupt arriving during an ISR -- silently shifted one column against
//     the others, pairing timestamps from different interrupts. `safety_check()`
//     only printed a size mismatch; the misaligned rows were still written.
//     Here a row cannot be assembled from more than one interrupt.
//
//   * STREAMED, NOT BUFFERED. Rows are written and flushed as they complete, so
//     a run that is killed or times out still yields every interrupt it finished,
//     and a partial file is obviously partial rather than subtly wrong.
//
// The ISR entry address arrives at run time on `isr_addr_i` (from the +IsrAddr
// plusarg, which the harness extracts from the ELF) rather than being a
// hardcoded parameter selected by a compile-time define. A wrong address now
// fails loudly -- no rows are written and the final check says so -- instead of
// producing plausible numbers for the wrong instruction.

module irq_edge_detect (
  input  logic clk_i,
  input  logic rst_ni,
  input  logic sig_i,
  output logic rising_o,
  output logic falling_o
);

  logic sig_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      sig_q <= 1'b0;
    end else begin
      sig_q <= sig_i;
    end
  end

  assign rising_o  = sig_i & ~sig_q;
  assign falling_o = ~sig_i & sig_q;

endmodule

module cva6tb_irq_log #(
  parameter int unsigned  NumIrqs  = 256,
  // Derived parameters - DO NOT OVERRIDE
  localparam int unsigned IdWidth  = NumIrqs > 1 ? $clog2(NumIrqs) : 1
) (
  input logic               clk_i,
  input logic               rst_ni,
  input event               start_log_i,
  // ISR entry address, from the +IsrAddr plusarg (harness reads it from the ELF)
  input logic [63:0]        isr_addr_i,
  input logic [NumIrqs-1:0] irqs_i,
  input logic               commit_valid_i,
  input logic [63:0]        commit_pc_i,
  input logic [63:0]        exception_pc_i,
  input logic [IdWidth-1:0] irq_id_i,
  input logic               irq_valid_i,
  input logic               irq_ack_i,
  input logic               eret_i
);

  // Tracking state for the single in-flight interrupt.
  typedef enum int {
    S_IDLE,     // waiting for a line to assert
    S_RAISED,   // line asserted, waiting for ack
    S_ACKED,    // acked, waiting for the ISR's first instruction to commit
    S_IN_ISR    // in the handler, waiting for eret
  } state_e;

  state_e      state_q;
  int unsigned cur_num;
  realtime     t_irq, t_valid, t_ack, t_first_commit, t_isr;
  logic [63:0] cur_pc;
  bit          first_commit_seen;

  int unsigned rows_written;
  int unsigned irqs_dropped;   // lines asserting while another is in flight
  int          fd;

  logic [NumIrqs-1:0] irqs_edge;
  generate
    genvar i;
    for (i = 0; i < NumIrqs; i++) begin : gen_edge_detect
      irq_edge_detect irq_edge_detect_inst (
        .clk_i     ( clk_i        ),
        .rst_ni    ( rst_ni       ),
        .sig_i     ( irqs_i[i]    ),
        .rising_o  ( irqs_edge[i] ),
        .falling_o ()
      );
    end
  endgenerate

  logic irq_valid_edge;
  irq_edge_detect irq_valid_edge_detect (
    .clk_i     ( clk_i          ),
    .rst_ni    ( rst_ni         ),
    .sig_i     ( irq_valid_i    ),
    .rising_o  ( irq_valid_edge ),
    .falling_o ()
  );

  function automatic void write_row();
    $fdisplay(fd, "%0d,%0h,%0t,%0t,%0t,%0t,%0t,%0t",
              cur_num, cur_pc, t_irq, t_valid, t_ack,
              t_first_commit, t_isr, $realtime);
    $fflush(fd);
    rows_written++;
  endfunction

  initial begin
    state_q           = S_IDLE;
    rows_written      = 0;
    irqs_dropped      = 0;
    first_commit_seen = 1'b0;

    @(start_log_i);

    fd = $fopen("irq_latencies.csv", "w");
    // Unable to record results is fatal: a run that cannot write its
    // measurements is worse than one that does not start.
    if (fd == 0) $fatal(1, "[IRQ] could not open irq_latencies.csv for writing");
    $fdisplay(fd, "irq_num,pc,irq_time,valid_time,ack_time,first_commit_time,isr_start_time,eret_time");
    $fflush(fd);
    $display("@%t | [%s] Logging interrupts, ISR entry at 0x%0h", $realtime, "IRQ", isr_addr_i);

    forever begin
      @(posedge clk_i);

      // A line asserting while an interrupt is already in flight is counted and
      // ignored rather than corrupting the record in progress. Nested tracking
      // needs one record per in-flight interrupt; this module is single-level.
      for (int unsigned n = 0; n < NumIrqs; ++n) begin
        if (irqs_edge[n]) begin
          if (state_q == S_IDLE) begin
            cur_num           = n;
            t_irq             = $realtime;
            t_valid           = 0;
            t_ack             = 0;
            t_first_commit    = 0;
            t_isr             = 0;
            cur_pc            = '0;
            first_commit_seen = 1'b0;
            state_q           = S_RAISED;
          end else begin
            irqs_dropped++;
          end
        end
      end

      case (state_q)
        S_RAISED: begin
          if (irq_valid_edge) t_valid = $realtime;
          if (irq_ack_i) begin
            t_ack   = $realtime;
            cur_pc  = exception_pc_i;
            state_q = S_ACKED;
          end
        end

        S_ACKED: begin
          if (commit_valid_i) begin
            if (!first_commit_seen) begin
              t_first_commit    = $realtime;
              first_commit_seen = 1'b1;
            end
            if (commit_pc_i == isr_addr_i) begin
              t_isr   = $realtime;
              state_q = S_IN_ISR;
            end
          end
        end

        S_IN_ISR: begin
          if (eret_i) begin
            write_row();
            state_q = S_IDLE;
          end
        end

        default: ; // S_IDLE: nothing to do until a line asserts
      endcase
    end
  end

  final begin
    $display("====== Interrupt log summary ======");
    $display("interrupts logged : %0d", rows_written);
    if (irqs_dropped != 0)
      $display("WARNING: %0d interrupt(s) asserted while another was in flight and were not logged",
               irqs_dropped);
    if (state_q != S_IDLE)
      $display("WARNING: run ended with an interrupt in flight (state %0d); its row was not written",
               state_q);
    if (rows_written == 0)
      $display("ERROR: no interrupts logged. If interrupts were enabled, the most likely cause is a wrong +IsrAddr (0x%0h): the ISR entry was never observed to commit.",
               isr_addr_i);
    $display("===================================");
    if (fd != 0) $fclose(fd);
  end

endmodule
