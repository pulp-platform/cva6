package cva6tb_hwmon_pkg;

  // Internal address width; 5 bits covers offsets 0x00–0x10.
  localparam int unsigned IntAddrWidth = 5;

  // Register map (write-only):
  //   RELEASE   0x00  wdata = task_id  → record release cycle, reset per-task state,
  //                                       compute deadline = time_i + period_q[task_id]
  //   PERIOD_LO 0x04  wdata[31:0]      → stage low 32 bits of task period (init only)
  //   PERIOD_HI 0x08  wdata[31:0]      → commit {HI,LO} as period for last RELEASE'd task
  //   START     0x0C  wdata = task_id  → open measurement window; handles preemption
  //   STOP      0x10  wdata = task_id  → close window, flush CSV row
  localparam logic [IntAddrWidth-1:0] CVA6TB_HWMON_RELEASE_OFFSET   = 5'h00;
  localparam logic [IntAddrWidth-1:0] CVA6TB_HWMON_PERIOD_LO_OFFSET = 5'h04;
  localparam logic [IntAddrWidth-1:0] CVA6TB_HWMON_PERIOD_HI_OFFSET = 5'h08;
  localparam logic [IntAddrWidth-1:0] CVA6TB_HWMON_START_OFFSET     = 5'h0C;
  localparam logic [IntAddrWidth-1:0] CVA6TB_HWMON_STOP_OFFSET      = 5'h10;

  localparam int unsigned NumTasks    = 32;
  localparam int unsigned TaskIdWidth = $clog2(NumTasks);
  typedef logic [TaskIdWidth-1:0] task_id_t;

  // RISC-V user privilege level (matches riscv::PRIV_LVL_U). Miss events are only
  // attributed to a task while the core executes at this level, so scheduler /
  // ecall bookkeeping done in M/S-mode between START and the actual task run is
  // excluded.
  localparam logic [1:0] CVA6TB_HWMON_PRIV_LVL_U = 2'b00;

  function automatic void mon_log(string msg);
    $display("@%t | [MON] %s", $realtime, msg);
  endfunction

endpackage

module cva6tb_hwmon
#(
  parameter type reg_req_t = logic,
  parameter type reg_rsp_t = logic
) (
  input  logic        clk_i,
  input  logic        rst_ni,
  input  reg_req_t    reg_req_i,
  output reg_rsp_t    reg_rsp_o,
  input  logic [63:0] time_i,
  // Single-cycle event pulses tapped directly from CVA6 RTL.
  // Each signal is high for exactly one cycle per miss event.
  input  logic        l1_icache_miss_i,
  input  logic        l1_dcache_miss_i,
  input  logic        itlb_miss_i,
  input  logic        dtlb_miss_i,
  // Current architectural privilege level of the core (riscv::priv_lvl_t).
  // Misses are only counted while the core runs in U-mode.
  input  logic [1:0]  priv_lvl_i
);

  import cva6tb_hwmon_pkg::*;

  // Write-only peripheral: always ready, never errors, no read data.
  assign reg_rsp_o.ready = 1'b1;
  assign reg_rsp_o.error = 1'b0;
  assign reg_rsp_o.rdata = '0;

  // ---------------------------------------------------------------------------
  // Decode incoming bus transaction
  // ---------------------------------------------------------------------------
  logic                    req_write;
  logic [IntAddrWidth-1:0] req_addr;
  task_id_t                req_task_id;
  logic [31:0]             req_wdata32;

  assign req_write   = reg_req_i.valid & reg_req_i.write;
  assign req_addr    = reg_req_i.addr[IntAddrWidth-1:0];
  assign req_task_id = task_id_t'(reg_req_i.wdata);
  assign req_wdata32 = reg_req_i.wdata[31:0];

  // ---------------------------------------------------------------------------
  // Internal state
  // ---------------------------------------------------------------------------

  // Free-running cycle counter.
  logic [63:0] cycle_q;

  // Per-task accumulated state (indexed by task_id).
  logic [63:0] release_cycle_q    [NumTasks]; // cycle at last RELEASE
  logic [63:0] deadline_q         [NumTasks]; // absolute deadline for current job
  logic [63:0] first_start_cycle_q[NumTasks]; // cycle of first CPU grant this job
  logic        first_start_pend_q [NumTasks]; // 1 = first CPU grant not yet seen
  logic [63:0] exec_time_q        [NumTasks]; // accumulated CPU time (across preemptions)
  logic [63:0] icache_count_q     [NumTasks]; // miss pulses since last RELEASE
  logic [63:0] dcache_count_q     [NumTasks];
  logic [63:0] itlb_count_q       [NumTasks];
  logic [63:0] dtlb_count_q       [NumTasks];
  logic [31:0] preempt_count_q    [NumTasks]; // preemptions this job

  // Active window state.
  logic        active_q;        // measurement window is open
  task_id_t    current_task_q;  // task currently on CPU
  logic [63:0] window_start_q;  // cycle_q at last START write

  // Per-task period (written once at init via PERIOD_LO/HI).
  logic [63:0] period_q           [NumTasks];

  // PERIOD staging: two-phase write for 64-bit period over 32-bit bus.
  logic [31:0] period_lo_q;
  task_id_t    deadline_target_q; // task whose period is being staged

  // ---------------------------------------------------------------------------
  // Free-running cycle counter
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) cycle_q <= '0;
    else         cycle_q <= cycle_q + 1;
  end

  // ---------------------------------------------------------------------------
  // Per-task state machine
  //
  // RELEASE write : resets all per-task accumulators for the released task.
  // START write   : opens a CPU window.  If another task was active (preemption),
  //                 the outgoing task's partial exec time is committed first.
  // STOP write    : finalises the CPU window; total exec_time is committed so the
  //                 logging block can read the correct final value.
  // Active cycles : miss pulses accumulated for current_task_q, but only while
  //                 the core runs in U-mode (priv_lvl_i == U). Tasks always run
  //                 in U-mode, so misses caused by the scheduler / ecall handler
  //                 in M/S-mode between START and the task actually running are
  //                 excluded.
  //
  // Misses on the START and STOP write cycles are also excluded (scheduler/ecall
  // overhead) because req_write suppresses the else-if accumulation branch.
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      active_q          <= 1'b0;
      current_task_q    <= '0;
      window_start_q    <= '0;
      period_lo_q       <= '0;
      deadline_target_q <= '0;
      for (int i = 0; i < NumTasks; i++) begin
        release_cycle_q[i]     <= '0;
        deadline_q[i]          <= '0;
        period_q[i]            <= '0;
        first_start_cycle_q[i] <= '0;
        first_start_pend_q[i]  <= 1'b0;
        exec_time_q[i]         <= '0;
        icache_count_q[i]      <= '0;
        dcache_count_q[i]      <= '0;
        itlb_count_q[i]        <= '0;
        dtlb_count_q[i]        <= '0;
        preempt_count_q[i]     <= '0;
      end
    end else if (req_write) begin
      case (req_addr)

        // --- New job released: reset all per-task accumulators,
        //     deadline auto-computed from stored period ---
        CVA6TB_HWMON_RELEASE_OFFSET: begin
          release_cycle_q[req_task_id]    <= cycle_q;
          deadline_q[req_task_id]         <= time_i + period_q[req_task_id];
          first_start_pend_q[req_task_id] <= 1'b1;
          exec_time_q[req_task_id]        <= '0;
          icache_count_q[req_task_id]     <= '0;
          dcache_count_q[req_task_id]     <= '0;
          itlb_count_q[req_task_id]       <= '0;
          dtlb_count_q[req_task_id]       <= '0;
          preempt_count_q[req_task_id]    <= '0;
          deadline_target_q               <= req_task_id;
        end

        // --- Stage low 32 bits of 64-bit period (init only) ---
        CVA6TB_HWMON_PERIOD_LO_OFFSET: begin
          period_lo_q <= req_wdata32;
        end

        // --- Commit full 64-bit period for the task from the last RELEASE ---
        CVA6TB_HWMON_PERIOD_HI_OFFSET: begin
          period_q[deadline_target_q] <= {req_wdata32, period_lo_q};
        end

        // --- Open measurement window; handle preemption transparently ---
        CVA6TB_HWMON_START_OFFSET: begin
          if (active_q && (current_task_q != req_task_id)) begin
            // Preemption: commit partial exec time for the task being kicked off.
            exec_time_q[current_task_q]     <=
                exec_time_q[current_task_q] + (cycle_q - window_start_q);
            preempt_count_q[current_task_q] <= preempt_count_q[current_task_q] + 1;
          end
          active_q       <= 1'b1;
          current_task_q <= req_task_id;
          window_start_q <= cycle_q;
          // Record the very first CPU grant this job (used for sched_jitter).
          if (first_start_pend_q[req_task_id]) begin
            first_start_cycle_q[req_task_id] <= cycle_q;
            first_start_pend_q[req_task_id]  <= 1'b0;
          end
        end

        // --- Close measurement window ---
        CVA6TB_HWMON_STOP_OFFSET: begin
          if (active_q) begin
            exec_time_q[current_task_q] <=
                exec_time_q[current_task_q] + (cycle_q - window_start_q);
            active_q <= 1'b0;
          end
        end

        default: ;
      endcase

    end else if (active_q && (priv_lvl_i == CVA6TB_HWMON_PRIV_LVL_U)) begin
      // Accumulate miss pulses for the task currently on CPU, but only while it
      // executes in U-mode. Misses generated by the scheduler / ecall handler in
      // M/S-mode between START and the task actually running are thus excluded.
      icache_count_q[current_task_q] <=
          icache_count_q[current_task_q] + 64'(l1_icache_miss_i);
      dcache_count_q[current_task_q] <=
          dcache_count_q[current_task_q] + 64'(l1_dcache_miss_i);
      itlb_count_q[current_task_q]   <=
          itlb_count_q[current_task_q]   + 64'(itlb_miss_i);
      dtlb_count_q[current_task_q]   <=
          dtlb_count_q[current_task_q]   + 64'(dtlb_miss_i);
    end
  end

  // ---------------------------------------------------------------------------
  // Combinational signals for CSV logging
  //
  // These are evaluated in the active region using pre-NBA _q values,
  // which are the fully accumulated totals before the STOP update fires.
  // exec_cycles manually adds the current window so the log sees the final total.
  // ---------------------------------------------------------------------------
  logic [63:0] log_exec_cycles;
  logic [63:0] log_response_time;
  logic [63:0] log_sched_jitter;
  logic        log_deadline_miss;

  assign log_exec_cycles   = exec_time_q[current_task_q] + (cycle_q - window_start_q);
  assign log_response_time = cycle_q - release_cycle_q[current_task_q];
  assign log_sched_jitter  = first_start_cycle_q[current_task_q] -
                             release_cycle_q[current_task_q];
  assign log_deadline_miss = (time_i  > deadline_q[current_task_q]) ? 1'b1 : 1'b0;

  // ---------------------------------------------------------------------------
  // CSV logging
  //
  // Runs in the active region of the STOP posedge. Because the always_ff above
  // uses non-blocking assignments, all _q values (and the combinational signals
  // derived from them) reflect the pre-update totals — the correct final values
  // for the completed job.
  //
  // Columns:
  //   task_id        : task identifier
  //   exec_cycles    : total CPU time (excluding scheduler and ecall cycles)
  //   response_time  : STOP cycle − RELEASE cycle
  //   sched_jitter   : first START cycle − RELEASE cycle
  //   deadline_miss  : 1 if time_i at STOP > absolute deadline (in time_i ticks)
  //   preemptions    : number of times the task was preempted this job
  //   icache_misses  : I-cache miss pulses attributed to this task (U-mode only)
  //   dcache_misses  : D-cache miss pulses (U-mode only)
  //   itlb_misses    : ITLB miss pulses (U-mode only)
  //   dtlb_misses    : DTLB miss pulses (U-mode only)
  // ---------------------------------------------------------------------------
  localparam string FileName = "hwmon_log.csv";
  integer log_file;

  function automatic void setup();
    log_file = $fopen(FileName, "w");
    if (log_file == 0) begin
      $display("ERROR: Could not open log file %s", FileName);
      $finish;
    end
    $fwrite(log_file, "task_id,exec_cycles,response_time,sched_jitter,deadline_miss,preemptions,icache_misses,dcache_misses,itlb_misses,dtlb_misses\n");
  endfunction

  function automatic void cleanup();
    $fclose(log_file);
  endfunction

  initial setup();

  always @(posedge clk_i) begin
    if (req_write) begin
      case (req_addr)
        CVA6TB_HWMON_RELEASE_OFFSET:
          mon_log($sformatf("RELEASE task %0d @time=%0d, deadline=%0d",
                   req_task_id, time_i, time_i + period_q[req_task_id]));
        CVA6TB_HWMON_START_OFFSET: begin
          if (active_q && current_task_q != req_task_id)
            mon_log($sformatf("START task %0d @time=%0d (preempts task=%0d)",
                     req_task_id, time_i, current_task_q));
          else
            mon_log($sformatf("START task %0d @time=%0d",
                     req_task_id, time_i));
        end
        CVA6TB_HWMON_STOP_OFFSET: begin
          if (active_q)
            mon_log($sformatf("STOP task %0d @time=%0d, %s",
                     current_task_q, time_i, log_deadline_miss ? "MISS" : "ok"));
        end
        default: ;
      endcase
    end
    if (req_write && req_addr == CVA6TB_HWMON_STOP_OFFSET && active_q) begin
      $fwrite(log_file, "%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d\n",
              current_task_q,
              log_exec_cycles,
              log_response_time,
              log_sched_jitter,
              log_deadline_miss,
              preempt_count_q[current_task_q],
              icache_count_q[current_task_q],
              dcache_count_q[current_task_q],
              itlb_count_q[current_task_q],
              dtlb_count_q[current_task_q]);
      $fflush(log_file);
    end
  end

  final cleanup();

endmodule
