// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Author: Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
//
// Spike co-simulation for CVA6. Samples the RVFI interface at commit and hands
// every retirement to a DPI-C backend (cva6tb_cosim.cc), which steps spike by
// one instruction and compares what it did with what the DUT did.
//
// This is the same sampling block as cva6tb_tracer.sv, in a module of its own:
// one observes, the other checks, and each is enabled on its own. Like the
// tracer it contains no strings, classes or queues, so it elaborates equally
// well under Verilator and under event driven simulators.
//
// Plusargs:
//   +nocosim               disable co-simulation in a model that has it compiled in
//   +cosim_image=<path>    the hex image the testbench loaded; defaults to +binary
//   +cosim_bootrom=<path>  raw boot ROM image the reference starts in
//   +cosim_spike_log=<p>   write spike's own commit log here
//   +cosim_max_reports=<n> stop printing mismatches after n of them (default 10)
//   +cosim_inject_fault=<n> corrupt retirement n, to prove the checker fires
//   +cosim_rules=<a>,<b>   enable named rule sets: misa-ro, lrsc, clic, cache-alias
//   +cosim_rule=<spec>     one or more ad-hoc rules, ';' separated, each a list of
//                          key=value: name, why, field, action, scope, csr, mask,
//                          budget, addr_lo, addr_hi, priv_mask, insn_mask,
//                          insn_match. A rule needs a name, a field, an action
//                          and a reason.
//
// At this milestone the program counter and the instruction encoding are
// compared, strictly. A mismatch stops the simulation at the instruction that
// diverged, which is the whole point: the alternative is a failure reported
// thousands of instructions downstream of its cause.

module cva6tb_cosim #(
    parameter config_pkg::cva6_cfg_t CVA6Cfg = config_pkg::cva6_cfg_empty,
    parameter type                   rvfi_instr_t = logic,
    parameter type                   rvfi_csr_t = logic,
    parameter int unsigned           HartId = 0
) (
    input logic                                    clk_i,
    input logic                                    rst_ni,
    input rvfi_instr_t [CVA6Cfg.NrCommitPorts-1:0] rvfi_i,
    // CSR state, which RVFI proper does not carry per instruction. Only the
    // registers a trap writes are looked at, and only where the write mask says
    // they were written: without this the architectural effect of taking a trap
    // -- mepc, mcause, mtval, mstatus -- would go entirely unchecked.
    input rvfi_csr_t                               rvfi_csr_i,
    // Trap value of the instruction that traps this cycle. RVFI itself does not
    // carry tval; cva6_rvfi exposes it on rvfi_to_iti_o.tval, registered in the
    // same cycle as the RVFI outputs. Tie to '0 if not available.
    input logic [           CVA6Cfg.XLEN-1:0]      trap_tval_i
);

  // Matches the tracer: a time literal resolved into the surrounding time unit
  // and used as a divisor, so this module needs no timeunit of its own.
  localparam longint unsigned TimeUnitsPerNs = longint'(1ns);

  import "DPI-C" function int cva6tb_cosim_open(
    input int xlen,
    input int vlen,
    input int plen,
    input int extensions
  );

  import "DPI-C" function int cva6tb_cosim_config(
    input int    handle,
    input string key,
    input string value
  );

  import "DPI-C" function void cva6tb_cosim_retire(
    input int              handle,
    input longint unsigned cycle,
    input longint unsigned sim_time_ns,
    input byte unsigned    commit_port,
    input longint unsigned order,
    input byte unsigned    priv,
    input byte unsigned    virt,
    input longint unsigned pc,
    input int unsigned     insn,
    input byte unsigned    rd_addr,
    input longint unsigned rd_wdata,
    input byte unsigned    rs1_addr,
    input longint unsigned rs1_rdata,
    input byte unsigned    rs2_addr,
    input longint unsigned rs2_rdata,
    input longint unsigned mem_vaddr,
    input longint unsigned mem_paddr,
    input int unsigned     mem_rmask,
    input int unsigned     mem_wmask,
    input longint unsigned mem_wdata,
    input longint unsigned mem_rdata
  );

  import "DPI-C" function void cva6tb_cosim_trap(
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

  import "DPI-C" function void cva6tb_cosim_interrupt(
    input int              handle,
    input longint unsigned cycle,
    input longint unsigned cause
  );

  import "DPI-C" function void cva6tb_cosim_csr(
    input int              handle,
    input int unsigned     addr,
    input longint unsigned rdata,
    input longint unsigned wdata,
    input longint unsigned wmask
  );

  import "DPI-C" function longint unsigned cva6tb_cosim_mismatches(input int handle);

  import "DPI-C" function byte unsigned cva6tb_cosim_budget_spent(input int handle);

  import "DPI-C" function void cva6tb_cosim_close(input int handle);

  // -1 marks "co-simulation disabled": every backend call then returns immediately
  int              handle = -1;
  longint unsigned cycle = 0;
  // RVFI's own order field is never driven, so the counter is kept here. It is
  // the identity a mismatch report names, so it counts every retirement across
  // both commit ports, in commit order.
  longint unsigned order = 0;

  // Forwards one option to the backend, which owns all of the optional
  // behaviour. A rejected value is a configuration mistake, so it stops the
  // simulation.
  function automatic void set_option(string key, string value);
    if (cva6tb_cosim_config(handle, key, value) != 0)
      $fatal(1, "[cva6tb_cosim] bad value '%s' for option '%s'", value, key);
  endfunction

  // Bit positions of the extension mask, mirroring rv_isa_bits in rv_cosim.h.
  // The two lists are meant to be diffed by eye, which is why both are flat.
  localparam int unsigned IsaA = 0, IsaB = 1, IsaC = 2, IsaD = 3, IsaF = 4, IsaH = 5;
  localparam int unsigned IsaM = 6, IsaS = 7, IsaU = 8, IsaV = 9;
  localparam int unsigned IsaZcb = 10, IsaZcmp = 11, IsaZcmt = 12, IsaZicond = 13;
  localparam int unsigned IsaZicntr = 14, IsaZihpm = 15, IsaZkn = 16, IsaZicbom = 17;
  localparam int unsigned IsaSvnapot = 18;
  localparam int unsigned IsaClic = 24, IsaCvxif = 25, IsaCustomFp = 26;

  // The DUT's extension set, handed over as a bitmask so that the ISA string the
  // reference is built with is derived in one place, from the configuration that
  // is actually elaborated.
  //
  // Every ISA-bearing field of config_pkg::cva6_cfg_t is accounted for here,
  // either mapped to a spike extension or named in the second group as one the
  // reference cannot model. The fields left out are those that do not change
  // what instruction set the reference should implement: the
  // microarchitectural ones (caches, TLBs, SuperscalarEn, bypasses, FPGA and
  // technology options), the dimensions forwarded separately through "dims"
  // (NrPMPEntries, Sdtrig*, DCACHE_LINE_WIDTH), the platform ones with no
  // bearing on decode (SoftwareInterruptEn, PerfCounterEn, TvalEn,
  // DirectVecOnly, PMPEntryReadOnly, PMPNapotEn), MmuPresent and the Sv mode,
  // which arrive as the vaddr width, and DebugEn, whose registers the reference
  // has but the testbench never drives. Adding an extension field to
  // cva6_cfg_t means adding it to this function; an extension the reference
  // does not implement makes it trap where the DUT does not, and the reverse.
  function automatic int unsigned isa_bits();
    int unsigned b = 0;
    // M is not a separate flag in cva6_cfg_t: CVA6 always implements it.
    b |= (1 << IsaM);
    if (CVA6Cfg.RVA) b |= (1 << IsaA);
    if (CVA6Cfg.RVB) b |= (1 << IsaB);
    if (CVA6Cfg.RVC) b |= (1 << IsaC);
    if (CVA6Cfg.RVD) b |= (1 << IsaD);
    if (CVA6Cfg.RVF) b |= (1 << IsaF);
    if (CVA6Cfg.RVH) b |= (1 << IsaH);
    if (CVA6Cfg.RVS) b |= (1 << IsaS);
    if (CVA6Cfg.RVU) b |= (1 << IsaU);
    if (CVA6Cfg.RVV) b |= (1 << IsaV);
    if (CVA6Cfg.RVZCB) b |= (1 << IsaZcb);
    if (CVA6Cfg.RVZCMP) b |= (1 << IsaZcmp);
    if (CVA6Cfg.RVZCMT) b |= (1 << IsaZcmt);
    if (CVA6Cfg.RVZiCond) b |= (1 << IsaZicond);
    if (CVA6Cfg.RVZicntr) b |= (1 << IsaZicntr);
    if (CVA6Cfg.RVZihpm) b |= (1 << IsaZihpm);
    if (CVA6Cfg.ZKN) b |= (1 << IsaZkn);
    if (CVA6Cfg.RVZiCbom) b |= (1 << IsaZicbom);
    if (CVA6Cfg.SvnapotEn) b |= (1 << IsaSvnapot);

    // Not extensions the reference can be built with. Reported at startup, so
    // enabling one is stated in the run's output.
    if (CVA6Cfg.RVSCLIC || CVA6Cfg.RVXHCLIC) b |= (1 << IsaClic);
    if (CVA6Cfg.CvxifEn) b |= (1 << IsaCvxif);
    if (CVA6Cfg.XF16 || CVA6Cfg.XF16ALT || CVA6Cfg.XF8 || CVA6Cfg.XFVec)
      b |= (1 << IsaCustomFp);
    return b;
  endfunction

  initial begin : open_cosim
    string val;
    if (!$test$plusargs("nocosim")) begin
      handle = cva6tb_cosim_open(CVA6Cfg.XLEN, CVA6Cfg.VLEN, CVA6Cfg.PLEN, int'(isa_bits()));
      if (handle < 0) $fatal(1, "[cva6tb_cosim] could not start co-simulation");

      // The reference loads the same image the testbench does, so that both
      // start from identical memory.
      if ($value$plusargs("cosim_image=%s", val)) set_option("image", val);
      else if ($value$plusargs("binary=%s", val)) set_option("image", val);
      else $fatal(1, "[cva6tb_cosim] no image: neither +cosim_image nor +binary was given");

      // The DUT's own dimensions, so the reference is configured as this core.
      // The PMP granule is eight bytes: CVA6
      // implements G=1, stated in core/pmp/src/pmp_entry.sv and not a
      // configuration parameter. Sdtrig off means no triggers at all, and the
      // cache block is the data cache line in bytes.
      set_option("dims", $sformatf("%0d,%0d,%0d,%0d", CVA6Cfg.NrPMPEntries, 8,
                                   CVA6Cfg.Sdtrig ? CVA6Cfg.SdtrigNrTriggers : 0,
                                   CVA6Cfg.DCACHE_LINE_WIDTH / 8));
      if ($value$plusargs("cosim_bootrom=%s", val)) set_option("bootrom", val);
      if ($value$plusargs("cosim_spike_log=%s", val)) set_option("spike_log", val);
      if ($value$plusargs("cosim_max_reports=%s", val)) set_option("max_reports", val);
      // Rules first, so that one named on the command line is tried before the
      // built-ins and can narrow them.
      if ($value$plusargs("cosim_rules=%s", val)) set_option("rules", val);
      if ($value$plusargs("cosim_rule=%s", val)) set_option("rule", val);
      // Self test: corrupt one retirement, to show that the checker fires.
      if ($value$plusargs("cosim_stop_after=%s", val)) set_option("stop_after", val);
      if ($value$plusargs("cosim_inject_fault=%s", val)) set_option("inject_fault", val);
    end
  end

  // Reports one CSR if it was written this cycle. A zero mask means untouched.
  function automatic void report_csr(int unsigned addr, logic [CVA6Cfg.XLEN-1:0] rdata,
                                     logic [CVA6Cfg.XLEN-1:0] wdata,
                                     logic [CVA6Cfg.XLEN-1:0] wmask);
    if (wmask != '0) cva6tb_cosim_csr(handle, addr, 64'(rdata), 64'(wdata), 64'(wmask));
  endfunction

  // Every register in the snapshot that the reference also has, reported where
  // it changed this cycle. The snapshot detects a *change*, so it covers the
  // registers whose changes both models derive from the program. Three classes
  // are outside that:
  //
  //   mip        raised by the interrupt lines and, in the reference, by this
  //              co-simulation itself, at an arrival time that legitimately
  //              differs.
  //   counters   mcycle, minstret and their unprivileged aliases advance at
  //              each model's own rate; the counter rules copy their reads.
  //   fflags     accrued by floating point execution, so a change can appear a
  //              cycle either side of the retirement that caused it.
  //
  // What is left is the architectural state software writes: the trap registers
  // -- which check trap semantics beyond the cause -- plus the delegation, protection and translation registers, where a WARL
  // field that accepts the wrong bits is a real divergence. Registers the
  // reference does not implement are dropped on its side and counted, so a
  // configuration that has CLIC or the CVA6 custom registers costs nothing here.
  function automatic void report_csrs();
    report_csr(32'(riscv::CSR_MSTATUS), rvfi_csr_i.mstatus.rdata, rvfi_csr_i.mstatus.wdata,
               rvfi_csr_i.mstatus.wmask);
    report_csr(32'(riscv::CSR_MEPC), rvfi_csr_i.mepc.rdata, rvfi_csr_i.mepc.wdata,
               rvfi_csr_i.mepc.wmask);
    report_csr(32'(riscv::CSR_MCAUSE), rvfi_csr_i.mcause.rdata, rvfi_csr_i.mcause.wdata,
               rvfi_csr_i.mcause.wmask);
    report_csr(32'(riscv::CSR_MTVAL), rvfi_csr_i.mtval.rdata, rvfi_csr_i.mtval.wdata,
               rvfi_csr_i.mtval.wmask);
    report_csr(32'(riscv::CSR_MTVEC), rvfi_csr_i.mtvec.rdata, rvfi_csr_i.mtvec.wdata,
               rvfi_csr_i.mtvec.wmask);
    report_csr(32'(riscv::CSR_MIE), rvfi_csr_i.mie.rdata, rvfi_csr_i.mie.wdata,
               rvfi_csr_i.mie.wmask);
    report_csr(32'(riscv::CSR_MSCRATCH), rvfi_csr_i.mscratch.rdata, rvfi_csr_i.mscratch.wdata,
               rvfi_csr_i.mscratch.wmask);
    report_csr(32'(riscv::CSR_MCOUNTINHIBIT), rvfi_csr_i.mcountinhibit.rdata,
               rvfi_csr_i.mcountinhibit.wdata, rvfi_csr_i.mcountinhibit.wmask);
    report_csr(32'(riscv::CSR_PMPCFG0), rvfi_csr_i.pmpcfg0.rdata, rvfi_csr_i.pmpcfg0.wdata,
               rvfi_csr_i.pmpcfg0.wmask);
    report_csr(32'(riscv::CSR_PMPCFG1), rvfi_csr_i.pmpcfg1.rdata, rvfi_csr_i.pmpcfg1.wdata,
               rvfi_csr_i.pmpcfg1.wmask);
    report_csr(32'(riscv::CSR_PMPCFG2), rvfi_csr_i.pmpcfg2.rdata, rvfi_csr_i.pmpcfg2.wdata,
               rvfi_csr_i.pmpcfg2.wmask);
    report_csr(32'(riscv::CSR_PMPCFG3), rvfi_csr_i.pmpcfg3.rdata, rvfi_csr_i.pmpcfg3.wdata,
               rvfi_csr_i.pmpcfg3.wmask);
    if (CVA6Cfg.RVU) begin
      report_csr(32'(riscv::CSR_MCOUNTEREN), rvfi_csr_i.mcounteren.rdata,
                 rvfi_csr_i.mcounteren.wdata, rvfi_csr_i.mcounteren.wmask);
    end
    if (CVA6Cfg.RVS) begin
      report_csr(32'(riscv::CSR_SSTATUS), rvfi_csr_i.sstatus.rdata, rvfi_csr_i.sstatus.wdata,
                 rvfi_csr_i.sstatus.wmask);
      report_csr(32'(riscv::CSR_SEPC), rvfi_csr_i.sepc.rdata, rvfi_csr_i.sepc.wdata,
                 rvfi_csr_i.sepc.wmask);
      report_csr(32'(riscv::CSR_SCAUSE), rvfi_csr_i.scause.rdata, rvfi_csr_i.scause.wdata,
                 rvfi_csr_i.scause.wmask);
      report_csr(32'(riscv::CSR_STVAL), rvfi_csr_i.stval.rdata, rvfi_csr_i.stval.wdata,
                 rvfi_csr_i.stval.wmask);
      report_csr(32'(riscv::CSR_STVEC), rvfi_csr_i.stvec.rdata, rvfi_csr_i.stvec.wdata,
                 rvfi_csr_i.stvec.wmask);
      report_csr(32'(riscv::CSR_SIE), rvfi_csr_i.sie.rdata, rvfi_csr_i.sie.wdata,
                 rvfi_csr_i.sie.wmask);
      report_csr(32'(riscv::CSR_SSCRATCH), rvfi_csr_i.sscratch.rdata, rvfi_csr_i.sscratch.wdata,
                 rvfi_csr_i.sscratch.wmask);
      report_csr(32'(riscv::CSR_SCOUNTEREN), rvfi_csr_i.scounteren.rdata,
                 rvfi_csr_i.scounteren.wdata, rvfi_csr_i.scounteren.wmask);
      report_csr(32'(riscv::CSR_SATP), rvfi_csr_i.satp.rdata, rvfi_csr_i.satp.wdata,
                 rvfi_csr_i.satp.wmask);
      report_csr(32'(riscv::CSR_MEDELEG), rvfi_csr_i.medeleg.rdata, rvfi_csr_i.medeleg.wdata,
                 rvfi_csr_i.medeleg.wmask);
      report_csr(32'(riscv::CSR_MIDELEG), rvfi_csr_i.mideleg.rdata, rvfi_csr_i.mideleg.wdata,
                 rvfi_csr_i.mideleg.wmask);
    end
  endfunction

  // bookkeeping is testbench local state, so plain blocking updates are used
  always @(posedge clk_i) begin : cosim
    if (!rst_ni) begin
      cycle = 0;
    end else begin
      cycle = cycle + 1;
      if (handle >= 0) begin
        // Before the retirements and traps of this cycle, so that whatever they
        // wrote is on hand when the one that caused it is compared.
        report_csrs();
        for (int unsigned i = 0; i < CVA6Cfg.NrCommitPorts; i++) begin
          if (rvfi_i[i].valid) begin
            // An interrupt is not a trap as far as RVFI is concerned: trap is
            // gated on the cause's interrupt bit being clear (cva6_rvfi.sv), so
            // an interrupt only shows up as intr on the first instruction of the
            // handler. Report it before that instruction, which is where it
            // happened.
            if (rvfi_i[i].intr[2]) begin
              cva6tb_cosim_interrupt(handle, cycle, 64'(rvfi_i[i].cause));
            end
            cva6tb_cosim_retire(
                handle,
                cycle,
                $time / TimeUnitsPerNs,
                8'(i),
                order,
                8'(rvfi_i[i].mode),
                8'(rvfi_i[i].virt),
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
            // cause is latched every cycle whether or not a trap is taken, so
            // it is only meaningful here, under rvfi_i[i].trap.
            cva6tb_cosim_trap(
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
        // Checked once per cycle: both ports have been handed over by now, and
        // the report names the one that diverged. Stopping here makes the
        // failure point to its cause.
        if (cva6tb_cosim_mismatches(handle) != 0) begin
          cva6tb_cosim_close(handle);
          $fatal(1, "[cva6tb_cosim] reference and DUT diverged");
        end
        // A program with no end of its own never writes the testbench's EOC
        // register. With +cosim_stop_after the comparison itself is the end
        // condition -- the requested instructions were compared and none
        // diverged -- so such a program stops there and does not spin until
        // the cycle budget runs out.
        if (cva6tb_cosim_budget_spent(handle) != 0) begin
          $display("*** [cva6tb_cosim] INFO: retirement budget spent, stopping");
          cva6tb_cosim_close(handle);
          $finish;
        end
      end
    end
  end

  final cva6tb_cosim_close(handle);

endmodule
