// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// Instruction tracer for CVA6. Samples the RVFI interface at commit and hands
// every retirement to a DPI-C backend (cva6tb_tracer.cc), which disassembles it
// and writes one line per instruction in the format of the legacy
// common/local/util/instr_tracer.sv.
//
// This module contains no strings, classes or queues, so it elaborates equally
// well under Verilator and under event driven simulators.
//
// Plusargs:
//   +notrace                    disable tracing entirely
//   +trace_file=<path>          output file      (default cva6tb_trace_hart_<id>.txt)
//   +trace_verbose=<n>          0 legacy format, 1 + memory/CSR data, 2 + port/order
//   +trace_format=<name>        legacy, spike, rvfi or binary         (default legacy)
//   +trace_stats=<path>         write an end of run summary here     (default: none)
//   +trace_start=<trigger>      open the trace window   (default: open from the start)
//   +trace_stop=<trigger>       close it                     (default: never closes)
//   +trace_symbols=<elf>[,...]  annotate with symbol names       (default: no names)
//   +trace_symbol_mode=<mode>   header, inline or both                (default header)
//   +trace_symbol_start=<trig>  open the annotation window
//   +trace_symbol_stop=<trig>   close it
//
// A trigger is a cycle count, `cycle:<n>`, `pc:<hex>` or `sym:<name>`; a symbol
// source is an ELF path, optionally `<path>@<bias>` when the image was placed
// somewhere other than its link address. See rv_trigger.h and rv_symbols.h.

module cva6tb_tracer #(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty,
    parameter type                   rvfi_instr_t = logic,
    parameter int unsigned           HartId = 0
) (
    input logic                                    clk_i,
    input logic                                    rst_ni,
    input rvfi_instr_t [CVA6Cfg.NrCommitPorts-1:0] rvfi_i,
    // Trap value of the instruction that traps this cycle. RVFI itself does not
    // carry tval; cva6_rvfi exposes it on rvfi_to_iti_o.tval, registered in the
    // same cycle as the RVFI outputs. Tie to '0 if not available.
    input logic [           CVA6Cfg.XLEN-1:0]      trap_tval_i,
    // Branch resolution, likewise not part of RVFI and likewise taken from
    // rvfi_to_iti_o. Tie both to '0 if not available: the mispredict column
    // then reads 0 everywhere, as it did before the probe existed.
    input logic [CVA6Cfg.NrCommitPorts-1:0]        branch_valid_i,
    input logic [CVA6Cfg.NrCommitPorts-1:0]        branch_taken_i,
    input logic [CVA6Cfg.NrCommitPorts-1:0]        branch_mispredict_i
);

  // The legacy tracer stamped its lines in nanoseconds. Declaring a timeunit
  // here would make every module without one warn, so instead the time literal
  // is resolved into the surrounding time unit and used as a divisor.
  localparam longint unsigned TimeUnitsPerNs = longint'(1ns);

  import "DPI-C" function int cva6tb_tracer_open(
    input string filename,
    input int    xlen,
    input int    vlen,
    input int    plen,
    input int    verbosity,
    input int    extensions
  );

  import "DPI-C" function void cva6tb_tracer_retire(
    input int               handle,
    input longint unsigned  cycle,
    input longint unsigned  sim_time_ns,
    input byte unsigned     commit_port,
    input longint unsigned  order,
    input byte unsigned     priv,
    input byte unsigned     flags,
    input longint unsigned  pc,
    input int unsigned      insn,
    input byte unsigned     rd_addr,
    input longint unsigned  rd_wdata,
    input byte unsigned     rs1_addr,
    input longint unsigned  rs1_rdata,
    input byte unsigned     rs2_addr,
    input longint unsigned  rs2_rdata,
    input longint unsigned  mem_vaddr,
    input longint unsigned  mem_paddr,
    input int unsigned      mem_rmask,
    input int unsigned      mem_wmask,
    input longint unsigned  mem_wdata,
    input longint unsigned  mem_rdata
  );

  import "DPI-C" function void cva6tb_tracer_trap(
    input int              handle,
    input longint unsigned cycle,
    input longint unsigned sim_time_ns,
    input byte unsigned    priv,
    input longint unsigned pc,
    input int unsigned     insn,
    input longint unsigned cause,
    input longint unsigned tval,
    input byte unsigned    is_interrupt
  );

  import "DPI-C" function int cva6tb_tracer_config(
    input int    handle,
    input string key,
    input string value
  );

  import "DPI-C" function void cva6tb_tracer_close(input int handle);

  // -1 marks "tracing disabled": every backend call then returns immediately
  int                  handle = -1;
  int                  verbosity = 0;
  longint unsigned     cycle = 0;
  longint unsigned     order = 0;

  // Forwards one option to the backend, which owns all of the optional
  // behaviour. A rejected value is a configuration mistake that would otherwise
  // silently produce the wrong trace, so it stops the simulation.
  function automatic void set_option(string key, string value);
    if (cva6tb_tracer_config(handle, key, value) != 0)
      $fatal(1, "[cva6tb_tracer] bad value '%s' for option '%s'", value, key);
  endfunction

  initial begin : open_trace
    string fname;
    string val;
    if (!$test$plusargs("notrace")) begin
      if (!$value$plusargs("trace_file=%s", fname))
        fname = $sformatf("cva6tb_trace_hart_%0d.txt", HartId);
      if (!$value$plusargs("trace_verbose=%d", verbosity)) verbosity = 0;
      // bit 0 marks Zcmt, whose cm.jt/cm.jalt share the c.fsdsp encoding
      handle = cva6tb_tracer_open(fname, CVA6Cfg.XLEN, CVA6Cfg.VLEN, CVA6Cfg.PLEN, verbosity,
                                  CVA6Cfg.RVZCMT ? 32'h1 : 32'h0);

      // Everything optional is forwarded by name. Adding an option later means
      // one more row here and one more key in the backend. Symbols are loaded
      // first so that triggers can name them.
      if (handle >= 0) begin
        set_option("hart_id", $sformatf("%0d", HartId));
        if ($value$plusargs("trace_format=%s", val)) set_option("format", val);
        if ($value$plusargs("trace_stats=%s", val)) set_option("stats", val);
        if ($value$plusargs("trace_symbols=%s", val)) set_option("symbols", val);
        if ($value$plusargs("trace_symbol_mode=%s", val)) set_option("symbol_mode", val);
        if ($value$plusargs("trace_start=%s", val)) set_option("start", val);
        if ($value$plusargs("trace_stop=%s", val)) set_option("stop", val);
        if ($value$plusargs("trace_symbol_start=%s", val)) set_option("symbol_start", val);
        if ($value$plusargs("trace_symbol_stop=%s", val)) set_option("symbol_stop", val);
      end
    end
  end

  // bookkeeping is testbench local state, so plain blocking updates are used
  always @(posedge clk_i) begin : trace
    byte unsigned flags;
    if (!rst_ni) begin
      cycle = 0;
    end else begin
      cycle = cycle + 1;
      if (handle >= 0) begin
        for (int unsigned i = 0; i < CVA6Cfg.NrCommitPorts; i++) begin
          if (rvfi_i[i].valid) begin
            flags = 8'b0;
            if (branch_mispredict_i[i]) flags[0] = 1'b1;
            if (rvfi_i[i].intr[0]) flags[1] = 1'b1;
            if (rvfi_i[i].intr[2]) flags[2] = 1'b1;
            if (branch_valid_i[i]) flags[3] = 1'b1;
            if (branch_taken_i[i]) flags[4] = 1'b1;
            cva6tb_tracer_retire(
                handle,
                cycle,
                $time / TimeUnitsPerNs,
                8'(i),
                order,
                8'(rvfi_i[i].mode),
                flags,
                64'(rvfi_i[i].pc_rdata[CVA6Cfg.VLEN-1:0]),
                32'(rvfi_i[i].insn),
                8'(rvfi_i[i].rd_addr),
                64'(rvfi_i[i].rd_wdata),
                8'(rvfi_i[i].rs1_addr),
                64'(rvfi_i[i].rs1_rdata),
                8'(rvfi_i[i].rs2_addr),
                64'(rvfi_i[i].rs2_rdata),
                64'(rvfi_i[i].mem_addr),
                64'(rvfi_i[i].mem_paddr),
                32'(rvfi_i[i].mem_rmask),
                32'(rvfi_i[i].mem_wmask),
                64'(rvfi_i[i].mem_wdata),
                64'(rvfi_i[i].mem_rdata)
            );
            order = order + 1;
          end else if (rvfi_i[i].trap) begin
            cva6tb_tracer_trap(
                handle,
                cycle,
                $time / TimeUnitsPerNs,
                8'(rvfi_i[i].mode),
                64'(rvfi_i[i].pc_rdata[CVA6Cfg.VLEN-1:0]),
                32'(rvfi_i[i].insn),
                64'(rvfi_i[i].cause),
                64'(trap_tval_i),
                8'(rvfi_i[i].cause[CVA6Cfg.XLEN-1])
            );
          end
        end
      end
    end
  end

  final cva6tb_tracer_close(handle);

endmodule
