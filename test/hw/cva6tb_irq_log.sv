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
  parameter int unsigned  NumIrqs       = 256,
  parameter bit [63:0]    FirstInstAddr = 64'h0000_0000_8000_0000,
  // Derived parameters - DO NOT OVERRIDE
  localparam int unsigned IdWidth       = NumIrqs > 1 ? $clog2(NumIrqs) : 1
) (
  input logic               clk_i,
  input logic               rst_ni,
  input event               start_log_i,
  input logic [NumIrqs-1:0] irqs_i,
  input logic               commit_valid_i,
  input logic [63:0]        commit_pc_i,
  input logic [63:0]        exception_pc_i,
  input logic [IdWidth-1:0] irq_id_i,
  input logic               irq_valid_i,
  input logic               irq_ack_i,
  input logic               eret_i
);

  typedef struct {
    int unsigned num;
    realtime     tstamp;
  } irq_t;

  irq_t        irqs               [$];
  realtime     valid_times        [$];
  logic [63:0] interrupted_pcs    [$];
  realtime     ack_times          [$];
  realtime     first_commit_times [$];
  realtime     isr_start_times    [$];
  realtime     eret_times         [$];

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

  logic irq_raised;
  logic irq_inflight;
  logic first_commit_done;

  initial begin
    irq_raised = 1'b0;
    irq_inflight = 1'b0;
    first_commit_done = 1'b0;
    @(start_log_i);
    $display("@%t | [%s] Logging interrupts", $realtime, "IRQ");
    forever begin
      @(posedge clk_i);
      for (int unsigned i = 0; i < NumIrqs; ++i) begin
        if (irqs_edge[i]) begin
          irqs.push_back('{num: i, tstamp: $realtime});
          irq_raised = 1'b1;
        end
      end
      if (irq_raised && irq_valid_edge) begin
        valid_times.push_back($realtime);
      end
      if (irq_raised && irq_ack_i) begin
        ack_times.push_back($realtime);
        interrupted_pcs.push_back(exception_pc_i);
        irq_inflight = 1'b1;
      end
      if (irq_inflight && commit_valid_i) begin
        if (!first_commit_done) begin
          first_commit_times.push_back($realtime);
          first_commit_done = 1'b1;
        end
        if (commit_pc_i == FirstInstAddr) begin
          isr_start_times.push_back($realtime);
          irq_inflight = 1'b0;
          irq_raised = 1'b0;
        end
      end
      if (first_commit_done && eret_i) begin
        eret_times.push_back($realtime);
        first_commit_done = 1'b0;
      end
    end
  end

  function automatic void safety_check();
    if (irqs.size() != valid_times.size()) begin
      $display("ERROR: Interrupt and valid times size mismatch (%d vs %d)", irqs.size(), valid_times.size());
      return;
    end

    if (irqs.size() != ack_times.size()) begin
      $display("ERROR: Interrupt and ack times size mismatch");
      return;
    end

    if (irqs.size() != interrupted_pcs.size()) begin
      $display("ERROR: Interrupt and interrupted PCs size mismatch");
      return;
    end

    if (irqs.size() != first_commit_times.size()) begin
      $display("ERROR: Interrupt and first commit times size mismatch");
      return;
    end

    if (irqs.size() != isr_start_times.size()) begin
      $display("ERROR: Interrupt and isr start times size mismatch");
      return;
    end

    if (irqs.size() != eret_times.size()) begin
      $display("ERROR: Interrupt and eret times size mismatch");
      return;
    end
  endfunction

  function automatic void display_log();

    $display("====== Interrupt log summary ======");

    foreach (irqs[i]) begin
      automatic realtime delta = isr_start_times[i] - irqs[i].tstamp;
      $display("@%t | Interrupt %0d latency: %t, interrupted PC: %0h", irqs[i].tstamp, irqs[i].num, delta, interrupted_pcs[i]);
    end

    $display("===================================");

  endfunction

  function automatic void log_csv(string filename = "irq_latencies.csv");
    int fd;
    fd = $fopen(filename, "w");
    if (fd == 0) begin
      $display("ERROR: Could not open %s for writing", filename);
      return;
    end
    $fdisplay(fd, "irq_num,pc,irq_time,valid_time,ack_time,first_commit_time,isr_start_time,eret_time");
    foreach (irqs[i]) begin
      $fdisplay(fd, "%0d,%0h,%0t,%0t,%0t,%0t,%0t,%0t", irqs[i].num, interrupted_pcs[i], irqs[i].tstamp, valid_times[i], ack_times[i], first_commit_times[i], isr_start_times[i], eret_times[i]);
    end
    $fclose(fd);
    $display("INFO: Interrupt latencies written to %s", filename);
  endfunction

  final begin
    safety_check();
    display_log();
    log_csv();
  end

endmodule
