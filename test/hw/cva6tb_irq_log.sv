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
// Properties this module is built around:
//
//   * ONE RECORD PER INTERRUPT. One queue per event type, paired by index at
//     the end, silently shifts a column against the others as soon as an event
//     fires an unequal number of times -- a spurious eret, an interrupt arriving
//     during an ISR -- and pairs timestamps from different interrupts. Here a row
//     cannot be assembled from more than one interrupt.
//
//   * STREAMED, NOT BUFFERED. Rows are written and flushed as they complete, so
//     a run that is killed or times out still yields every interrupt it finished,
//     and a partial file is obviously partial rather than subtly wrong.
//
//   * ONLY THE TRACKED INTERRUPT COUNTS. The CLIC presents every enabled source
//     on one valid/ready pair, so valid and ack are matched against the tracked
//     line's CLIC ID, and only the eret that unwinds to its level ends the row.
//
// The ISR entry address arrives at run time on `isr_addr_i` (the +IsrAddr
// plusarg, taken from the ELF), so one compiled image serves every software
// build. A wrong address fails loudly -- no rows are written and the final check
// says so -- instead of producing plausible numbers for the wrong instruction.

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
  // CLIC interrupt ID of irqs_i[0]: the external lines sit above the SoC's
  // internal sources, so line n is CLIC ID IdOffset + n
  parameter int unsigned  IdOffset = 0,
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
  input logic               eret_i,
  // The core is halted for a reason outside the interrupt path -- a cache flush
  // triggered by fence.i (controller fence_active_q). Recorded per interrupt
  // (fence_blocked column) so an analysis can report those arrivals apart.
  input logic               blocked_i
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
  bit          cur_blocked;    // blocked_i seen while this interrupt was pending
  int unsigned cur_id;         // CLIC ID of the interrupt being tracked
  int unsigned nest;           // interrupts taken inside our handler, not yet returned

  int unsigned rows_written;
  int unsigned irqs_dropped;   // lines asserting while another is in flight
  int unsigned isr_missed;     // handler returned without the ISR entry committing
  int unsigned rows_blocked;   // rows with fence_blocked = 1
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

  logic irq_ack_edge;
  irq_edge_detect irq_ack_edge_detect (
    .clk_i     ( clk_i        ),
    .rst_ni    ( rst_ni       ),
    .sig_i     ( irq_ack_i    ),
    .rising_o  ( irq_ack_edge ),
    .falling_o ()
  );

  logic irq_valid_edge;
  irq_edge_detect irq_valid_edge_detect (
    .clk_i     ( clk_i          ),
    .rst_ni    ( rst_ni         ),
    .sig_i     ( irq_valid_i    ),
    .rising_o  ( irq_valid_edge ),
    .falling_o ()
  );

  function automatic void write_row();
    $fdisplay(fd, "%0d,%0h,%0t,%0t,%0t,%0t,%0t,%0t,%0d",
              cur_num, cur_pc, t_irq, t_valid, t_ack,
              t_first_commit, t_isr, $realtime, cur_blocked);
    $fflush(fd);
    rows_written++;
    if (cur_blocked) rows_blocked++;
  endfunction

  initial begin
    state_q           = S_IDLE;
    rows_written      = 0;
    irqs_dropped      = 0;
    isr_missed        = 0;
    rows_blocked      = 0;
    first_commit_seen = 1'b0;
    cur_blocked       = 1'b0;

    @(start_log_i);

    fd = $fopen("irq_latencies.csv", "w");
    // Unable to record results is fatal: a run that cannot write its
    // measurements is worse than one that does not start.
    if (fd == 0) $fatal(1, "[IRQ] could not open irq_latencies.csv for writing");
    $fdisplay(fd, "irq_num,pc,irq_time,valid_time,ack_time,first_commit_time,isr_start_time,eret_time,fence_blocked");
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
            cur_id            = IdOffset + n;
            t_irq             = $realtime;
            t_valid           = 0;
            t_ack             = 0;
            t_first_commit    = 0;
            t_isr             = 0;
            cur_pc            = '0;
            first_commit_seen = 1'b0;
            cur_blocked       = 1'b0;
            state_q           = S_RAISED;
          end else begin
            irqs_dropped++;
          end
        end
      end

      // Between the line asserting and the ISR entry committing, note whether
      // the core was halted by a fence (its entry may wait for the flush).
      if ((state_q == S_RAISED || state_q == S_ACKED) && blocked_i) cur_blocked = 1'b1;

      case (state_q)
        // The CLIC presents and acknowledges every enabled source -- the timer
        // included -- on the same valid/ready pair, so both are matched against
        // the ID of the line being tracked. An unfiltered ack from another
        // source would become this record's ack, and the record would then wait
        // for an ISR entry that never comes, dropping every later interrupt.
        S_RAISED: begin
          if (irq_valid_i && int'(irq_id_i) == cur_id && t_valid == 0) t_valid = $realtime;
          if (irq_ack_i && int'(irq_id_i) == cur_id) begin
            t_ack   = $realtime;
            cur_pc  = exception_pc_i;
            nest    = 0;
            state_q = S_ACKED;
          end
        end

        S_ACKED, S_IN_ISR: begin
          if (state_q == S_ACKED && commit_valid_i) begin
            if (!first_commit_seen) begin
              t_first_commit    = $realtime;
              first_commit_seen = 1'b1;
            end
            if (commit_pc_i == isr_addr_i) begin
              t_isr   = $realtime;
              state_q = S_IN_ISR;
            end
          end

          // An interrupt taken inside our handler returns with its own eret;
          // only the eret that unwinds back to our level ends the record.
          if (irq_ack_edge) nest++;
          if (eret_i) begin
            if (nest != 0) begin
              nest--;
            end else begin
              // Our handler returned. If the ISR entry never committed (wrong
              // +IsrAddr, or a handler that skips it) the row is still written,
              // with isr_start_time 0, and counted: one bad record must not
              // hold the logger and drop every interrupt after it.
              if (state_q == S_ACKED) isr_missed++;
              write_row();
              state_q = S_IDLE;
            end
          end
        end

        default: ; // S_IDLE: nothing to do until a line asserts
      endcase
    end
  end

  final begin
    $display("====== Interrupt log summary ======");
    $display("interrupts logged : %0d", rows_written);
    $display("fence-blocked     : %0d (entry delayed by a cache-flush fence; fence_blocked = 1)", rows_blocked);
    if (irqs_dropped != 0)
      $display("WARNING: %0d interrupt(s) asserted while another was in flight and were not logged",
               irqs_dropped);
    if (isr_missed != 0)
      $display("WARNING: %0d interrupt(s) returned without committing the ISR entry 0x%0h (isr_start_time 0 in their rows)",
               isr_missed, isr_addr_i);
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
