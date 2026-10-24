// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Performance probes: what the issue stage does with each issue slot, every
// cycle, for a testbench to account where the core's cycles go. Like the RVFI
// probes, they cost nothing when the output is left unconnected.

`ifndef PERF_TYPES_SVH
`define PERF_TYPES_SVH

`define PERF_PROBES_T(Cfg) struct packed { \
  /* a decoded instruction is available to the slot */ \
  logic [Cfg.NrIssuePorts-1:0] decoded_valid; \
  /* ... and the scoreboard has room for it */ \
  logic [Cfg.NrIssuePorts-1:0] issue_valid; \
  /* ... and it is issued */ \
  logic [Cfg.NrIssuePorts-1:0] issued; \
  /* its functional unit */ \
  ariane_pkg::fu_t [Cfg.NrIssuePorts-1:0] fu; \
  /* not issued because the functional unit is busy */ \
  logic [Cfg.NrIssuePorts-1:0] fu_busy; \
  /* not issued because an operand is not ready */ \
  logic [Cfg.NrIssuePorts-1:0] raw; \
  /* ... produced by the older instruction of the same issue group */ \
  logic [Cfg.NrIssuePorts-1:0] raw_in_group; \
  /* ... and the functional unit producing it */ \
  ariane_pkg::fu_t [Cfg.NrIssuePorts-1:0] raw_fu; \
  /* busy only because the older instruction of the same issue group takes it */ \
  logic [Cfg.NrIssuePorts-1:0] fu_busy_in_group; \
  /* the issue stage is stalled from outside */ \
  logic issue_stall; \
  /* the frontend is flushed and redirected */ \
  logic flush_frontend; \
  /* a branch resolved as mispredicted */ \
  logic mispredict; \
  /* the instruction cache missed (one cycle per miss) */ \
  logic icache_miss; \
  /* the instruction cache returns data to the frontend */ \
  logic icache_valid; \
  /* the data cache missed (one cycle per miss) */ \
  logic dcache_miss; \
}

`endif  // PERF_TYPES_SVH
