# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
#
# FORCE-RISCV for the co-simulation's generated stimulus, included by
# ../../Makefile. See README.md.

# An instruction sequence generator, used as a second source of stimulus for
# co-simulation. It is built with the host's system compiler, which its vendored
# pybind11 needs (it predates GCC 13's stricter <cstdint> includes), against the
# host's CPython 3.6 development headers, the version FORCE's own README uses. It
# therefore runs on the host as well; see util/run-force.sh.
FORCE_DIR        ?= $(TEST_DIR)/tools/force-riscv
FORCE_URL        ?= https://github.com/openhwgroup/force-riscv.git
FORCE_CC         ?= /usr/bin/g++
FORCE_PYTHON_VER ?= 3.6
FORCE_PYTHON_INC ?= /usr/include/python$(FORCE_PYTHON_VER)m
FORCE_PYTHON_LIB ?= /usr/lib64
FORCE_JOBS       ?= 8

$(FORCE_DIR)/Makefile:
	git clone $(FORCE_URL) $(FORCE_DIR)

$(FORCE_DIR)/bin/friscv: $(FORCE_DIR)/Makefile
	cd $(FORCE_DIR) && FORCE_CC=$(FORCE_CC) FORCE_PYTHON_VER=$(FORCE_PYTHON_VER) \
	    FORCE_PYTHON_INC=$(FORCE_PYTHON_INC) FORCE_PYTHON_LIB=$(FORCE_PYTHON_LIB) \
	    $(MAKE) -j$(FORCE_JOBS)

# The port: the memory description and the program counter layout go into
# FORCE's own python tree, because that is where it keeps them, and the
# configuration and the template go under config/cores/ beside its own examples.
.PHONY: force
force: $(FORCE_DIR)/bin/friscv
	mkdir -p $(FORCE_DIR)/config/cores/cva6tb
	cp $(TEST_DIR)/sw/fuzz/force/riscv-cva6tb.config \
	   $(TEST_DIR)/sw/fuzz/force/cva6tb_random_force.py \
	   $(TEST_DIR)/sw/fuzz/force/register_field_choices-cva6tb.xml \
	   $(FORCE_DIR)/config/cores/cva6tb/
	cp $(TEST_DIR)/sw/fuzz/force/memory-cva6tb.py $(FORCE_DIR)/py/riscv/
	cp $(TEST_DIR)/sw/fuzz/force/PcConfig.py $(FORCE_DIR)/py/riscv/PcConfig.py

.PHONY: force-clean
force-clean:
	rm -rf $(FORCE_DIR) $(VERILATOR_DIR)/force
