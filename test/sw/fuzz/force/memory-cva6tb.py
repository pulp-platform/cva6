# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
#
# Physical memory available to FORCE-RISCV on the cva6tb testbench.
#
# DRAM only, and only the 256 MiB the testbench actually instantiates -- see
# test/hw/cva6tb_pkg.sv. Anything above 0x9000_0000 reads back as an AXI error.
#
# The SPM at 0x2000_0000 is left out. The testbench decides where
# to load an image from the first address in its hex file, so a program spanning
# both regions could not be loaded at all.


class FpixReporter(object):
    def __init__(self):
        pass


def configure_memory(interface):
    interface.addMemoryRange(0, 0x000080000000, 0x00008FFFFFFF)
