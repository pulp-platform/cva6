# Copyright 2026 ETH Zurich and University of Bologna.
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51
#
# Parameters of the timing intent of synth_top (cva6.sdc). A flow sets the
# period and the margin, and the technology its constants and the object
# lookups the SDC uses (asic_inputs_except), before it sources this file and
# the SDC:
#   ASIC_TCK               clock period [ps]
#   ASIC_SYNTH_MARGIN      extra setup uncertainty in synthesis, a fraction of
#                          the period; 0 in place and route
#   ASIC_PS_PER_UNIT       picoseconds per time unit of the libraries
#   ASIC_CLK_TRANSITION    clock transition before clock tree synthesis [ps]
#   ASIC_HOLD_UNCERTAINTY  hold uncertainty [ps]
# The parameters below have defaults; fractions are of the period.
#   ASIC_SETUP_UNCERTAINTY setup uncertainty before clock tree synthesis
#   ASIC_IO_DELAY          input and output delay outside the block
#   ASIC_IO_MULTICYCLE     cycles an I/O path may take: the testbench side is
#                          registered
if {![info exists ASIC_SETUP_UNCERTAINTY]} { set ASIC_SETUP_UNCERTAINTY 0.05 }
if {![info exists ASIC_IO_DELAY]}          { set ASIC_IO_DELAY 0.05 }
if {![info exists ASIC_IO_MULTICYCLE]}     { set ASIC_IO_MULTICYCLE 3 }

# The clock and the reset ports of synth_top
set ASIC_CLOCK_PORTS {clk_i}
set ASIC_RESET_PORTS {rst_ni}
