# Copyright 2026 ETH Zurich and University of Bologna.
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51
#
# Parameters of the timing intent of cva6_asic_top (cva6.sdc). A flow sets the
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
#   ASIC_IO_MULTICYCLE     cycles an I/O path may take: one, so that the core
#                          meets single-cycle ports with its delays, as the
#                          testbench's netlist simulation with an SDF has
#   ASIC_CLIC_MULTICYCLE   cycles a path from the CLIC's interrupt request
#                          (ASIC_CLIC_PORTS) may take: the CLIC holds a request
#                          until the core accepts it
if {![info exists ASIC_SETUP_UNCERTAINTY]} { set ASIC_SETUP_UNCERTAINTY 0.05 }
if {![info exists ASIC_IO_DELAY]}          { set ASIC_IO_DELAY 0.05 }
if {![info exists ASIC_IO_MULTICYCLE]}     { set ASIC_IO_MULTICYCLE 1 }
if {![info exists ASIC_CLIC_MULTICYCLE]}   { set ASIC_CLIC_MULTICYCLE 2 }

# The clock and the reset ports of cva6_asic_top
set ASIC_CLOCK_PORTS {clk_i}
set ASIC_RESET_PORTS {rst_ni}
# The inputs of the CLIC's interrupt request, which the CLIC holds while it
# requests
set ASIC_CLIC_PORTS  {clic_irq_valid_i clic_irq_id_i clic_irq_level_i clic_irq_priv_i
                      clic_irq_v_i clic_irq_vsid_i clic_irq_shv_i clic_irq_rstk_i
                      clic_irq_kill_req_i}
