# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
#
# Records the core's switching activity in each workload phase into
# saif/phase_<marker>.saif (make vsim-run SAIF=1): a phase lasts while the
# program holds a nonzero marker in the testbench's marker register
# (cva6tb_regs, /test/i_dut/marker). Sourced before the simulation runs.
#
# Each change of the marker ends the running phase (writing its activity) and,
# if the new marker is nonzero, starts the next one. Questa counts from the
# first `power add` (the first phase) and restarts the counts with
# `power reset -all`, but always writes the time since that first add as the
# SAIF's DURATION: each file gets its phase's length instead.

proc cva6tb_saif_end { } {
    set file saif/phase_${::cva6tb_saif_phase}.saif
    power report -all -bsaif ${file}
    set fh [open ${file}]
    set saif [read ${fh}]
    close ${fh}
    regsub {\(DURATION [0-9]+\)} ${saif} "(DURATION [expr { $::now - $::cva6tb_saif_start }])" saif
    set fh [open ${file} w]
    puts -nonewline ${fh} ${saif}
    close ${fh}
    echo "\[SAIF\] Phase ${::cva6tb_saif_phase} ends at $::now ps: ${file}"
    set ::cva6tb_saif_phase ""
}

file mkdir saif
set ::cva6tb_saif_phase ""
set ::cva6tb_saif_added 0
when -label cva6tb_saif {/test/i_dut/marker'event} {
    if { $::cva6tb_saif_phase ne "" } { cva6tb_saif_end }
    set marker [examine -radix hex /test/i_dut/marker]
    if { [regexp {^0*([1-9a-fA-F][0-9a-fA-F]*)$} $marker -> phase] } {
        if { $::cva6tb_saif_added } {
            power reset -all
        } else {
            power add -nocellnet -r -in -out -inout -internal /test/i_dut/i_cva6/*
            set ::cva6tb_saif_added 1
        }
        set ::cva6tb_saif_phase ${phase}
        set ::cva6tb_saif_start $::now
        echo "\[SAIF\] Phase ${phase} starts at $::now ps"
    }
}
