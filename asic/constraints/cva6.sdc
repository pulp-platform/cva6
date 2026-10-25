# Copyright 2026 ETH Zurich and University of Bologna.
# Solderpad Hardware License, Version 0.51, see LICENSE for details.
# SPDX-License-Identifier: SHL-0.51
#
# Timing intent of cva6_asic_top. Synthesis and place and route both read it, each
# at its own period; it names ports only, so it holds for the RTL and for the
# netlist alike. Parameters and their defaults: cva6.tcl. Times are given in ps
# and scaled to the libraries' time unit.

set cva6_t [expr {double($ASIC_TCK) / $ASIC_PS_PER_UNIT}]
create_clock -name CVA6_CLK -period $cva6_t [get_ports $ASIC_CLOCK_PORTS]

# Setup uncertainty: ASIC_SETUP_UNCERTAINTY of the period, plus the synthesis
# margin ASIC_SYNTH_MARGIN. Synthesis sets cva6_setup_uncertainty alone again
# before it writes its outputs and reports.
set cva6_setup_uncertainty [expr {$ASIC_SETUP_UNCERTAINTY * $cva6_t}]
set_clock_uncertainty -setup [expr {$cva6_setup_uncertainty + $ASIC_SYNTH_MARGIN * $cva6_t}] [get_clocks CVA6_CLK]
set_clock_uncertainty -hold  [expr {double($ASIC_HOLD_UNCERTAINTY) / $ASIC_PS_PER_UNIT}] [get_clocks CVA6_CLK]
set_clock_transition         [expr {double($ASIC_CLK_TRANSITION) / $ASIC_PS_PER_UNIT}] [get_clocks CVA6_CLK]

# I/O: ASIC_IO_DELAY of the period outside the block, ASIC_IO_MULTICYCLE cycles
# for every I/O path, and ASIC_CLIC_MULTICYCLE for the paths from the CLIC's
# interrupt request. The exceptions are written from the ports, so that no
# register is an exception end point. The hold false paths take precedence on
# input-to-output paths.
set cva6_clic_inputs [get_ports $ASIC_CLIC_PORTS]
set cva6_inputs [asic_inputs_except [concat $ASIC_CLOCK_PORTS $ASIC_RESET_PORTS $ASIC_CLIC_PORTS]]
set_input_delay  [expr {$ASIC_IO_DELAY * $cva6_t}] -clock CVA6_CLK $cva6_inputs
set_input_delay  [expr {$ASIC_IO_DELAY * $cva6_t}] -clock CVA6_CLK $cva6_clic_inputs
set_output_delay [expr {$ASIC_IO_DELAY * $cva6_t}] -clock CVA6_CLK [all_outputs]
set_multicycle_path $ASIC_IO_MULTICYCLE                  -setup -to [all_outputs]
set_multicycle_path [expr {$ASIC_IO_MULTICYCLE - 1}]     -hold  -to [all_outputs]
set_multicycle_path $ASIC_IO_MULTICYCLE                  -setup -from $cva6_inputs
set_multicycle_path [expr {$ASIC_IO_MULTICYCLE - 1}]     -hold  -from $cva6_inputs
set_multicycle_path $ASIC_CLIC_MULTICYCLE                -setup -from $cva6_clic_inputs
set_multicycle_path [expr {$ASIC_CLIC_MULTICYCLE - 1}]   -hold  -from $cva6_clic_inputs
set_false_path -hold -through $cva6_inputs -through [all_outputs]
set_false_path -hold -through $cva6_clic_inputs -through [all_outputs]

# Reset: asserted asynchronously, released within one period
set_max_delay $cva6_t -from [get_ports $ASIC_RESET_PORTS]
set_false_path -hold -from [get_ports $ASIC_RESET_PORTS]

# boot_addr_i is quasi-static: the core samples it when leaving reset
set_false_path -from [get_ports boot_addr_i]
