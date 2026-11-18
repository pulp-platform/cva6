// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// Basic testbench pre-loading a binary and polling the EOC register for success.
// Times out after a specified number of cycles.

module test;

  import ariane_pkg::*;
  import cva6tb_pkg::*;
  import cva6tb_regs_pkg::EOCRegOffset;

  // localparam time         ClkPeriod = 1ns;
  // localparam time         RtcPeriod = 1us;
  localparam int unsigned ClkPeriodPs = 1000;
  localparam int unsigned RtcPeriodPs = 1000000;
  localparam time         ApplDelay = (ClkPeriodPs * 1ps) * 0.1;
  localparam int unsigned RstCycles = 10;
  localparam time         ClkPeriod = ClkPeriodPs * 1ps;

  int unsigned RetCodeSuccess;
  int unsigned MaxCycles;
  int unsigned MemDelay;
  int unsigned IrqPeriod;
  int unsigned IrqJitter;
  int unsigned NumInterrupts;
  int unsigned IrqSeed;
  int unsigned IrqStartDelay;
  bit          IrqStartOnReady;
  longint unsigned IsrAddr;
  bit          InterruptsEnabled;

  logic bootmode;

  // empty -> no waveform dump
  string fstfile;

  function automatic void parse_args();
    if (!$value$plusargs("RetCodeSuccess=%d",    RetCodeSuccess))       RetCodeSuccess = 0;
    if (!$value$plusargs("MaxCycles=%d",         MaxCycles))                 MaxCycles = 10_000;
    if (!$value$plusargs("MemDelay=%d",          MemDelay))                   MemDelay = 10;
    if (!$value$plusargs("IrqPeriod=%d",         IrqPeriod))                 IrqPeriod = 100_000;
    // Randomised arrival: each inter-arrival is IrqPeriod + U(0, IrqJitter),
    // drawn from Seed, so the interrupt lands at a uniformly distributed phase
    // of the background task's interference loop. A fixed period samples one
    // alignment, which makes an observed maximum an artefact of that phase.
    if (!$value$plusargs("IrqJitter=%d",         IrqJitter))                 IrqJitter = 0;
    if (!$value$plusargs("Seed=%d",              IrqSeed))                     IrqSeed = 1;
    if (!$value$plusargs("IrqStartDelay=%d",     IrqStartDelay))         IrqStartDelay = 0;
    if (!$value$plusargs("IrqStartOnReady=%b",   IrqStartOnReady)) IrqStartOnReady = 1'b0;
    // Number of interrupts to raise; the run ends 100k cycles after the last one.
    if (!$value$plusargs("NumInterrupts=%d",     NumInterrupts))         NumInterrupts = 50;
    // ISR entry address for the interrupt logger, taken from the ELF (e.g.
    // fast_irq_handler): a run parameter, so one compiled image serves every
    // software build.
    if (!$value$plusargs("IsrAddr=%h",           IsrAddr))                     IsrAddr = 64'h0;
    if (!$value$plusargs("InterruptsEnabled=%b", InterruptsEnabled)) InterruptsEnabled = 1'b0;
    if (!$value$plusargs("fst=%s", fstfile)) begin
      if ($test$plusargs("fst")) fstfile = "cva6tb.fst";
    end
    // No default ISR address: it is build-specific, and a default that is right
    // for one build measures the wrong instruction in every other. Refuse instead.
    if (InterruptsEnabled && IsrAddr == 64'h0)
      $fatal(1, "+InterruptsEnabled=1 requires +IsrAddr=<hex>, the ISR entry address from the ELF");
  endfunction

  logic        clk;
  logic        rst_n;
  logic        rtc;
  logic        eoc;
  logic [30:0] retcode;

  // Parameters for interrupt generation
  localparam int unsigned IrqId          = 1;
  localparam int unsigned IrqHighCycles  = 10;

  assign eoc     = i_dut.i_regs.control_regs[EOCRegOffset][0];
  assign retcode = i_dut.i_regs.control_regs[EOCRegOffset][31:1];

  logic [NumClicExtIrqs-1:0] clic_ext_irqs;
  logic                [7:0] clic_irq_id;
  logic                      clic_irq_valid;
  logic                      clic_irq_ready;
  logic                      eret;
  logic                      commit_valid;
  logic               [63:0] commit_pc;
  logic               [63:0] exception_pc;
  logic                      fence_flush;
  logic                      clic_irq_req;
  event                      irq_start_log;
  event                      irq_start_gen;

  assign clic_irq_id    = i_dut.clic_irq_id;
  assign clic_irq_valid = i_dut.clic_irq_valid;
  assign clic_irq_ready = i_dut.clic_irq_ready;
  assign eret           = i_dut.i_cva6.eret;
  assign commit_valid   = i_dut.i_cva6.commit_instr_id_commit[0].valid & i_dut.i_cva6.commit_ack_commit_id[0];
  assign commit_pc      = i_dut.i_cva6.pc_commit;
  // The PC the trap saves as its return address. Taken from the commit stage's
  // output rather than the head of the scoreboard: a CLIC interrupt taken with an
  // empty pipeline has no head entry, and its return address comes from elsewhere.
  assign exception_pc   = i_dut.i_cva6.commit_stage_i.pc_o;
  // Core halted by a fence.i cache flush (CVA6 controller); see cva6tb_irq_log.
  assign fence_flush    = i_dut.i_cva6.controller_i.fence_active_q;
  // The core's CLIC controller request (after SIE / threshold gating); see cva6tb_irq_log.
  assign clic_irq_req   = i_dut.i_cva6.clic_irq_req_id;

  // Interrupt generation
  initial begin : irq_gen
    automatic int count = 1;
    automatic int unsigned gap;
    automatic int unsigned seed_state;
    clic_ext_irqs = '0;
    wait(irq_start_gen.triggered);
    // Read the seed only now: irq_start_gen fires after parse_args(), whereas
    // at time 0 this block can run first and copy the unset value.
    seed_state = IrqSeed;
    if (IrqStartOnReady) begin
      // Start once the scheduler is up (the monitor's RUN_READY), then wait
      // IrqStartDelay cycles: a fixed 1M-cycle delay is arbitrary and, with
      // many short parallel runs, more expensive than the measurement itself.
      wait(i_dut.i_perf_mon.run_ready === 1'b1);
      #(ClkPeriod * IrqStartDelay);
    end else begin
      // Software that never drives the monitor (bare-metal tests) gets the
      // fixed initial delay.
      #(ClkPeriod * (1000000 - IrqPeriod));
    end
    forever begin
      gap = IrqPeriod;
      // Explicit LCG (Numerical Recipes constants), upper bits for the draw.
      // Not $urandom(seed_state): its seed argument is an input, so the state
      // never advances, and simulators differ in what they do with it -- it is
      // ignored under Verilator, which made every +Seed give the same arrivals.
      seed_state = seed_state * 32'd1664525 + 32'd1013904223;
      if (IrqJitter != 0) gap += ((seed_state >> 8) % IrqJitter);
      #(ClkPeriod * (gap - IrqHighCycles));
      log($sformatf("Generating interrupt %0d", IrqId));
      #ApplDelay;
      clic_ext_irqs[IrqId] = 1'b1;
      #(ClkPeriod - ApplDelay);
      #(ClkPeriod * (IrqHighCycles - 1));
      clic_ext_irqs[IrqId] = 1'b0;
      if (count == NumInterrupts) break;
      count += 1;
    end
    #(ClkPeriod * 100000);
    log("**Interrupt generation finished**");
    cleanup;
  end

  cva6tb_irq_log #(
    .NumIrqs        ( NumClicExtIrqs ),
    .IdOffset       ( NumClicIntIrqs )
  ) i_irq_log (
    .clk_i          ( clk            ),
    .rst_ni         ( rst_n          ),
    .start_log_i    ( irq_start_log  ),
    .isr_addr_i     ( IsrAddr        ),
    .irqs_i         ( clic_ext_irqs  ),
    .commit_valid_i ( commit_valid   ),
    .commit_pc_i    ( commit_pc      ),
    .exception_pc_i ( exception_pc   ),
    .irq_id_i       ( clic_irq_id    ),
    .irq_valid_i    ( clic_irq_valid ),
    .irq_ack_i      ( clic_irq_ready ),
    .eret_i         ( eret           ),
    .blocked_i      ( fence_flush    ),
    .irq_req_i      ( clic_irq_req   )
  );

  cva6tb_clk_rst_gen #(
    .ClkPeriodPs  ( ClkPeriodPs ),
    .RstClkCycles ( RstCycles   )
  ) i_clk_rst_gen (
    .clk_o        ( clk         ),
    .rst_no       ( rst_n       )
  );

  cva6tb_clk_rst_gen #(
    .ClkPeriodPs  ( RtcPeriodPs ),
    .RstClkCycles ( 1           )
  ) i_rtc_gen (
    .clk_o        ( rtc         ),
    .rst_no       (             )
  );

  cva6tb_soc i_dut (
    .clk_i           ( clk           ),
    .rst_ni          ( rst_n         ),
    .rtc_i           ( rtc           ),
    .boot_mode_i     ( bootmode      ),
    .clic_ext_irqs_i ( clic_ext_irqs ),
    .plic_ext_irqs_i ( '0            )
  );

  task automatic preload_hex();
    string       binary_path;
    int          fd;
    string       line;
    logic [31:0] first_addr;

    if ($value$plusargs("binary=%s", binary_path)) begin
      log($sformatf("Running program \"%s\"", binary_path));

      fd = $fopen(binary_path, "r");
      if (fd == 0) begin
        log($sformatf("ERROR: cannot open \"%s\"", binary_path));
        return;
      end
      void'($fgets(line, fd));
      $fclose(fd);

      if (line.len() > 0 && line[0] == "@") begin
        void'($sscanf(line, "@%h", first_addr));
      end else begin
        first_addr = '0;
      end

      if (first_addr == 32'h2000_0000) begin
        bootmode = 1'b1;
        i_dut.load_sim_spm(binary_path);
      end else begin
        bootmode = 1'b0;
        i_dut.load_sim_mem(binary_path);
      end
    end else begin
      log("WARNING: no binary path provided");
    end
  endtask

  initial begin
    bootmode = 1'b0;
    // Parse command line arguments
    parse_args();
    setup(fstfile);
    // Preload binary into simulation memory and set bootmode
    preload_hex();
    // Main memory latency
    i_dut.set_mem_delay(MemDelay);
    // Wait for reset de-assertion
    wait(rst_n);
    // Start the interrupt logger and generator
    if (InterruptsEnabled) -> irq_start_log;
    if (InterruptsEnabled) -> irq_start_gen;
    // Poll EOC and the monitor's run-complete flag.
    //
    // +NumJobs (read by the monitor) ends the run once the measured task has
    // completed that many jobs, so every cell of a campaign collects exactly
    // the same number of samples and no job is cut mid-execution. MaxCycles is
    // then a safety net rather than the thing that decides how much data a run
    // produces -- which is what it used to be, with the side effect that
    // different configurations yielded different sample counts.
    for (int unsigned i = 0; i < MaxCycles; ++i) begin
      @(posedge clk);
      if (eoc == 1'b1) begin
        log($sformatf("EOC register was set to one - return code: %0d", retcode));
        if (retcode == RetCodeSuccess) pass();
        else fail();
      end
      if (i_dut.i_perf_mon.run_done === 1'b1) begin
        log("Requested job count reached");
        pass();
      end
    end
    log($sformatf("Simluation timed out after %d cycles", MaxCycles));
    fail();
  end

`ifdef CVA6TB_AMO_MON
  // Per-AMO latency log of the HPDcache configurations, amo_lat.csv: the cycles
  // from the AMO's arrival at the head of the scoreboard to its request, from
  // the request to its acceptance by the HPDcache (and the requests of other
  // ports accepted meanwhile), from the acceptance to the start of the uncached
  // handler (and the cache work in progress meanwhile), and from there to the
  // response.
  localparam int unsigned AmoPort = 3;  // HPDcache requester of the store/AMO port
  typedef enum logic [1:0] {AMO_IDLE, AMO_RAISED, AMO_ACCEPTED, AMO_IN_UC} amo_state_e;
  amo_state_e      amo_state;
  int              amo_fd;
  longint unsigned amo_cycle, amo_c_head, amo_c_raise, amo_c_accept, amo_c_uc;
  int unsigned     amo_gnt[3], amo_mshr_cycles, amo_flush_cycles, amo_rtab_cycles;
  logic [63:0]     amo_addr;
  logic            amo_head_q;
  logic            amo_head, amo_req, amo_ack, amo_accept, uc_idle;
  logic [2:0]      other_accept;

  assign amo_head   = i_dut.i_cva6.commit_stage_i.commit_instr_i[0].valid &&
                      ariane_pkg::is_amo(i_dut.i_cva6.commit_stage_i.commit_instr_i[0].op);
  assign amo_req    = i_dut.i_cva6.amo_req.req;
  assign amo_ack    = i_dut.i_cva6.amo_resp.ack;
  assign amo_accept = i_dut.i_cva6.gen_cache_hpd.i_cache_subsystem.i_dcache.i_hpdcache.core_req_arbiter_i.core_req_valid_i[AmoPort] &&
                      i_dut.i_cva6.gen_cache_hpd.i_cache_subsystem.i_dcache.i_hpdcache.core_req_arbiter_i.core_req_ready_o[AmoPort];
  for (genvar p = 0; p < 3; p++) begin : gen_other_accept
    assign other_accept[p] = i_dut.i_cva6.gen_cache_hpd.i_cache_subsystem.i_dcache.i_hpdcache.core_req_arbiter_i.core_req_valid_i[p] &&
                             i_dut.i_cva6.gen_cache_hpd.i_cache_subsystem.i_dcache.i_hpdcache.core_req_arbiter_i.core_req_ready_o[p];
  end
  assign uc_idle = i_dut.i_cva6.gen_cache_hpd.i_cache_subsystem.i_dcache.i_hpdcache.uc_ready;

  initial begin
    amo_fd = $fopen("amo_lat.csv", "w");
    $fwrite(amo_fd, "addr,head_to_req,req_to_accept,accept_to_uc,uc_to_ack,req_to_ack,");
    $fwrite(amo_fd, "acc_ptw,acc_load,acc_ctrl,mshr_cycles,flush_cycles,rtab_cycles\n");
  end

  always_ff @(posedge clk) begin
    amo_cycle  <= amo_cycle + 1;
    amo_head_q <= amo_head;
    if (amo_head && !amo_head_q) amo_c_head <= amo_cycle;
    unique case (amo_state)
      AMO_IDLE: begin
        if (amo_req) begin
          amo_c_raise      <= amo_cycle;
          amo_addr         <= i_dut.i_cva6.amo_req.operand_a;
          amo_mshr_cycles  <= 0;
          amo_flush_cycles <= 0;
          amo_rtab_cycles  <= 0;
          for (int p = 0; p < 3; p++) amo_gnt[p] <= other_accept[p];
          if (amo_accept) begin
            amo_c_accept <= amo_cycle;
            amo_state    <= AMO_ACCEPTED;
          end else begin
            amo_state <= AMO_RAISED;
          end
        end
      end
      AMO_RAISED: begin
        for (int p = 0; p < 3; p++) amo_gnt[p] <= amo_gnt[p] + other_accept[p];
        if (amo_accept) begin
          amo_c_accept <= amo_cycle;
          amo_state    <= AMO_ACCEPTED;
        end
      end
      AMO_ACCEPTED: begin
        amo_mshr_cycles  <= amo_mshr_cycles  + !i_dut.i_cva6.gen_cache_hpd.i_cache_subsystem.i_dcache.i_hpdcache.miss_mshr_empty;
        amo_flush_cycles <= amo_flush_cycles + !i_dut.i_cva6.gen_cache_hpd.i_cache_subsystem.i_dcache.i_hpdcache.flush_empty;
        amo_rtab_cycles  <= amo_rtab_cycles  + !i_dut.i_cva6.gen_cache_hpd.i_cache_subsystem.i_dcache.i_hpdcache.rtab_empty;
        if (!uc_idle) begin
          amo_c_uc  <= amo_cycle;
          amo_state <= AMO_IN_UC;
        end
      end
      AMO_IN_UC: begin
        if (amo_ack) begin
          $fwrite(amo_fd, "%h,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d,%0d\n", amo_addr,
                  amo_c_raise - amo_c_head, amo_c_accept - amo_c_raise, amo_c_uc - amo_c_accept,
                  amo_cycle - amo_c_uc, amo_cycle - amo_c_raise, amo_gnt[0], amo_gnt[1], amo_gnt[2],
                  amo_mshr_cycles, amo_flush_cycles, amo_rtab_cycles);
          $fflush(amo_fd);
          amo_state <= AMO_IDLE;
        end
      end
      default: amo_state <= AMO_IDLE;
    endcase
  end
`endif

endmodule
