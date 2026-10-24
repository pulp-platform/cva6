// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Cycle accounting for benchmarks
//
// Within a program's region of interest, delimited by two marker instructions
// retiring (see test/sw/bench/include/bench.h), every issue slot of every
// cycle is put in exactly one class, so the classes of each slot add up to the
// region's cycles:
//
//   issued      an instruction issued from the slot
//   recovery    no instruction, the frontend refilling after a redirect
//   frontend    no instruction, otherwise
//   sb_full     an instruction, but the scoreboard is full
//   blocked     an instruction, but an older slot did not issue (in order)
//   stall       an instruction, but the issue stage is stalled from outside
//   fu_busy     an instruction, but its functional unit is busy
//   raw         an instruction, but one of its operands is not ready
//   other       an instruction, not issued for another reason
//
// fu_busy and raw are further broken down by functional unit: of the waiting
// instruction for fu_busy, of the instruction producing the operand for raw.
// Within the classes, also counted are:
//   - frontend and recovery slots during an instruction cache miss: from the
//     miss, through the refill, until decode has an instruction again
//   - fu_busy slots whose unit is taken by the older instruction of the group
//   - blocked slots, by the class of the older slot
//   - stalls waiting for a load, as episodes by length: an episode lasts as
//     long as one slot waits for one load, short for a data cache hit, long
//     for a miss
// and the cache misses themselves. The region's retired instructions come
// from RVFI. When the region closes, the counters are written to perf.json in
// the working directory, and one line, "[PERF] roi cycles=<n> retired=<n>",
// to the console.

module cva6tb_perf #(
  parameter config_pkg::cva6_cfg_t CVA6Cfg       = config_pkg::cva6_cfg_empty,
  parameter type                   rvfi_instr_t  = logic,
  parameter type                   perf_probes_t = logic
) (
  input logic                                            clk_i,
  input logic                                            rst_ni,
  input rvfi_instr_t  [CVA6Cfg.NrCommitPorts-1:0]        rvfi_i,
  input perf_probes_t                                    perf_i
);

  // The markers bench.h retires: slti x0, x0, 1 and slti x0, x0, 2
  localparam logic [31:0] RoiStart = 32'h0010_2013;
  localparam logic [31:0] RoiStop  = 32'h0020_2013;

  localparam int unsigned NrPorts = CVA6Cfg.NrIssuePorts;

  typedef enum int unsigned {
    Issued, Recovery, Frontend, SbFull, Blocked, Stall, FuBusy, Raw, Other, NumClasses
  } slot_class_e;

  localparam int unsigned NumFus      = 12;  // ariane_pkg::fu_t: NONE .. AES
  localparam int unsigned HistLength  = 32;  // redirect penalty histogram, last bin open-ended
  localparam int unsigned LoadHistLen = 64;  // load stall episodes of 1 .. 63 cycles, then 64 or more

  function automatic string class_name(int unsigned c);
    case (c)
      Issued:   return "issued";
      Recovery: return "recovery";
      Frontend: return "frontend";
      SbFull:   return "sb_full";
      Blocked:  return "blocked";
      Stall:    return "stall";
      FuBusy:   return "fu_busy";
      Raw:      return "raw";
      default:  return "other";
    endcase
  endfunction

  function automatic string fu_name(int unsigned fu);
    case (fu)
      0:       return "none";
      1:       return "load";
      2:       return "store";
      3:       return "alu";
      4:       return "ctrl_flow";
      5:       return "mult";
      6:       return "csr";
      7:       return "fpu";
      8:       return "fpu_vec";
      9:       return "cvxif";
      10:      return "accel";
      default: return "aes";
    endcase
  endfunction

  logic roi_q = 1'b0;
  logic recovery_q = 1'b0;
  int unsigned recovery_len = 0;

  // Instruction cache miss episode: from the miss to the refill's response,
  // then until decode has an instruction again.
  typedef enum logic [1:0] { IcIdle, IcRefill, IcDeliver } ic_state_e;
  ic_state_e ic_q = IcIdle;

  // Length of the current load stall episode of each slot
  int unsigned load_stall_len[NrPorts];

  longint unsigned cycles = 0;
  longint unsigned retired = 0;
  longint unsigned mispredicts = 0;
  longint unsigned redirects = 0;
  longint unsigned slots[NrPorts][NumClasses];
  longint unsigned fu_busy_by_fu[NumFus];
  longint unsigned raw_by_fu[NumFus];
  longint unsigned raw_in_group = 0;
  longint unsigned penalty_hist[HistLength];
  longint unsigned icache_misses = 0;
  longint unsigned dcache_misses = 0;
  longint unsigned frontend_icache_miss = 0;
  longint unsigned recovery_icache_miss = 0;
  longint unsigned fu_busy_in_group = 0;
  longint unsigned blocked_by[NumClasses];
  longint unsigned load_stall_hist[LoadHistLen];
  longint unsigned load_stall_long_cycles = 0;

  initial begin
    foreach (slots[i, c]) slots[i][c] = 0;
    foreach (fu_busy_by_fu[f]) fu_busy_by_fu[f] = 0;
    foreach (raw_by_fu[f]) raw_by_fu[f] = 0;
    foreach (penalty_hist[l]) penalty_hist[l] = 0;
    foreach (blocked_by[c]) blocked_by[c] = 0;
    foreach (load_stall_hist[l]) load_stall_hist[l] = 0;
    foreach (load_stall_len[i]) load_stall_len[i] = 0;
  end

  function automatic void close_load_stall(int unsigned i);
    if (load_stall_len[i] == 0) return;
    if (load_stall_len[i] >= LoadHistLen) begin
      load_stall_hist[LoadHistLen-1]++;
      load_stall_long_cycles += load_stall_len[i];
    end else begin
      load_stall_hist[load_stall_len[i]-1]++;
    end
    load_stall_len[i] = 0;
  endfunction

  function automatic slot_class_e classify(int unsigned i);
    if (perf_i.issued[i]) return Issued;
    if (!perf_i.decoded_valid[i]) return recovery_q ? Recovery : Frontend;
    if (!perf_i.issue_valid[i]) return SbFull;
    if (i > 0 && !perf_i.issued[i-1]) return Blocked;
    if (perf_i.issue_stall) return Stall;
    if (perf_i.fu_busy[i]) return FuBusy;
    if (perf_i.raw[i]) return Raw;
    return Other;
  endfunction

  function automatic void write_report();
    int fd;
    $display("[PERF] roi cycles=%0d retired=%0d", cycles, retired);
    fd = $fopen("perf.json", "w");
    if (fd == 0) begin
      $display("[PERF] cannot open perf.json");
      return;
    end
    $fwrite(fd, "{\n");
    $fwrite(fd, "  \"cycles\": %0d,\n", cycles);
    $fwrite(fd, "  \"retired\": %0d,\n", retired);
    $fwrite(fd, "  \"issue_ports\": %0d,\n", NrPorts);
    $fwrite(fd, "  \"mispredicts\": %0d,\n", mispredicts);
    $fwrite(fd, "  \"redirects\": %0d,\n", redirects);
    $fwrite(fd, "  \"slots\": [\n");
    for (int unsigned i = 0; i < NrPorts; i++) begin
      $fwrite(fd, "    {");
      for (int unsigned c = 0; c < NumClasses; c++)
        $fwrite(fd, "\"%s\": %0d%s", class_name(c), slots[i][c], c == NumClasses - 1 ? "" : ", ");
      $fwrite(fd, "}%s\n", i == NrPorts - 1 ? "" : ",");
    end
    $fwrite(fd, "  ],\n");
    $fwrite(fd, "  \"fu_busy_by_fu\": {");
    for (int unsigned f = 0; f < NumFus; f++)
      $fwrite(fd, "\"%s\": %0d%s", fu_name(f), fu_busy_by_fu[f], f == NumFus - 1 ? "" : ", ");
    $fwrite(fd, "},\n");
    $fwrite(fd, "  \"raw_by_producer_fu\": {");
    for (int unsigned f = 0; f < NumFus; f++)
      $fwrite(fd, "\"%s\": %0d%s", fu_name(f), raw_by_fu[f], f == NumFus - 1 ? "" : ", ");
    $fwrite(fd, "},\n");
    $fwrite(fd, "  \"raw_in_group\": %0d,\n", raw_in_group);
    $fwrite(fd, "  \"fu_busy_in_group\": %0d,\n", fu_busy_in_group);
    $fwrite(fd, "  \"blocked_by\": {");
    for (int unsigned c = 0; c < NumClasses; c++)
      $fwrite(fd, "\"%s\": %0d%s", class_name(c), blocked_by[c], c == NumClasses - 1 ? "" : ", ");
    $fwrite(fd, "},\n");
    $fwrite(fd, "  \"icache_misses\": %0d,\n", icache_misses);
    $fwrite(fd, "  \"dcache_misses\": %0d,\n", dcache_misses);
    $fwrite(fd, "  \"frontend_icache_miss\": %0d,\n", frontend_icache_miss);
    $fwrite(fd, "  \"recovery_icache_miss\": %0d,\n", recovery_icache_miss);
    $fwrite(fd, "  \"load_stall_hist\": [");
    for (int unsigned l = 0; l < LoadHistLen; l++)
      $fwrite(fd, "%0d%s", load_stall_hist[l], l == LoadHistLen - 1 ? "" : ", ");
    $fwrite(fd, "],\n");
    $fwrite(fd, "  \"load_stall_long_cycles\": %0d,\n", load_stall_long_cycles);
    $fwrite(fd, "  \"redirect_penalty_hist\": [");
    for (int unsigned l = 0; l < HistLength; l++)
      $fwrite(fd, "%0d%s", penalty_hist[l], l == HistLength - 1 ? "" : ", ");
    $fwrite(fd, "]\n}\n");
    $fclose(fd);
  endfunction

  always @(posedge clk_i) begin
    if (rst_ni) begin
      automatic logic roi_start = 1'b0;
      automatic logic roi_stop  = 1'b0;

      if (roi_q) begin
        automatic slot_class_e older = Issued;
        cycles++;
        for (int unsigned i = 0; i < NrPorts; i++) begin
          automatic slot_class_e c = classify(i);
          slots[i][c]++;
          if (c == FuBusy) begin
            fu_busy_by_fu[perf_i.fu[i]]++;
            if (perf_i.fu_busy_in_group[i]) fu_busy_in_group++;
          end
          if (c == Raw) begin
            raw_by_fu[perf_i.raw_fu[i]]++;
            if (perf_i.raw_in_group[i]) raw_in_group++;
          end
          if (c == Frontend && ic_q != IcIdle) frontend_icache_miss++;
          if (c == Recovery && ic_q != IcIdle) recovery_icache_miss++;
          if (c == Blocked) blocked_by[older]++;
          if (c == Raw && perf_i.raw_fu[i] == ariane_pkg::LOAD) load_stall_len[i]++;
          else close_load_stall(i);
          older = c;
        end
        if (perf_i.mispredict) mispredicts++;
        if (perf_i.flush_frontend) redirects++;
        if (perf_i.icache_miss) icache_misses++;
        if (perf_i.dcache_miss) dcache_misses++;
        if (recovery_q) recovery_len++;
      end

      if (perf_i.icache_miss) ic_q <= IcRefill;
      else if (ic_q == IcRefill && perf_i.icache_valid) ic_q <= IcDeliver;
      else if (ic_q == IcDeliver && perf_i.decoded_valid[0]) ic_q <= IcIdle;

      // Redirect recovery: from a frontend flush to the first instruction
      // offered again, whose length goes to the penalty histogram.
      if (perf_i.flush_frontend) begin
        recovery_q   <= 1'b1;
        recovery_len = 0;
      end else if (recovery_q && perf_i.decoded_valid[0]) begin
        recovery_q <= 1'b0;
        if (roi_q) penalty_hist[recovery_len < HistLength ? recovery_len : HistLength - 1]++;
      end

      for (int unsigned k = 0; k < CVA6Cfg.NrCommitPorts; k++) begin
        if (rvfi_i[k].valid) begin
          if (roi_q) retired++;
          if (rvfi_i[k].insn == RoiStart) roi_start = 1'b1;
          if (rvfi_i[k].insn == RoiStop) roi_stop = 1'b1;
        end
      end
      if (roi_start && !roi_q) roi_q <= 1'b1;
      if (roi_stop && roi_q) begin
        roi_q <= 1'b0;
        write_report();
      end
    end
  end

endmodule
