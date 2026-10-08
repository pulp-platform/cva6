# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
#
# Spike co-simulation, included by ../../Makefile in its Verilator section:
# spike and the standalone reference, and with COSIM=1 the co-simulation's part
# of the Verilator build. Co-simulation runs on Verilator only. See README.md.

COSIM         ?= 0
COSIM_DIR     := $(TEST_DIR)/hw/cosim
# The model and the reference. cva6tb_cosim.cc is the DPI half and is only
# compiled into the simulation; everything else is shared with the standalone
# tool, which is the same code driven without any RTL.
COSIM_LIB_SRC := $(COSIM_DIR)/rv_memif.cc $(COSIM_DIR)/rv_ref_spike.cc
COSIM_SRC     := $(COSIM_DIR)/cva6tb_cosim.cc $(COSIM_DIR)/rv_cosim.cc \
                 $(COSIM_DIR)/rv_policy.cc $(COSIM_LIB_SRC)
COSIM_HDR     := $(COSIM_DIR)/cva6tb_cosim_dpi.h $(COSIM_DIR)/rv_cosim.h \
                 $(COSIM_DIR)/rv_policy.h $(COSIM_DIR)/rv_memif.h $(COSIM_DIR)/rv_ref.h \
                 $(COSIM_DIR)/rv_ref_spike.h
COSIM_STANDALONE := $(COSIM_DIR)/rv_cosim_standalone

# Spike is built from source into a prefix inside test/, which pins the version
# and keeps test/ self-contained.
SPIKE_DIR     ?= $(TEST_DIR)/tools/spike
SPIKE_SRC_DIR ?= $(TEST_DIR)/tools/riscv-isa-sim
SPIKE_URL     ?= https://github.com/riscv-software-src/riscv-isa-sim.git
# Spike is pinned to a master commit: its last release predates the H, Zcb and
# Zicbom support these configurations need. The criterion for bumping it: the newest
# upstream master that builds here and still satisfies the API this backend
# depends on (take_trap private, mip::backdoor_write_with_mask, the seven
# simif_t pure virtuals, the processor_t constructor, mem_t, and the
# log_reg_write type nibble in decode_macros.h). Checked at this commit.
SPIKE_COMMIT  ?= 0bff12123b1fd510e19e19634dd997dbade70e54  # master, 2026-09-28
SPIKE_JOBS    ?= 8
# Command prefix of the environment spike and the standalone reference are built
# and run in; empty for the host's own, and util/iis-env.sh sets it. The model
# links spike, so this environment and Verilator's must agree on the C++ runtime:
# the same libstdc++ ABI, and a libstdc++ and glibc where the model runs at least
# as new as spike was built against. A mismatch stops the model's link or its
# start with undefined references or a missing GLIBCXX/GLIBC version.
SPIKE_ENV     ?=
# That environment's system compiler, named explicitly since the environment
# inherits the caller's PATH
SPIKE_CC      ?= /usr/bin/gcc
SPIKE_CXX     ?= /usr/bin/g++
SPIKE_CFLAGS  = -std=c++20 -I$(SPIKE_DIR)/include
SPIKE_LDFLAGS = -L$(SPIKE_DIR)/lib -lriscv -lsoftfloat -Wl,-rpath,$(SPIKE_DIR)/lib

ifeq ($(COSIM),1)
ifneq ($(filter vsim-build vsim-run,$(MAKECMDGOALS)),)
$(error Co-simulation runs on Verilator only: use verilator-build and verilator-run with COSIM=1)
endif
BENDER_FLAGS += -t cosim
# The reference's memory is zero filled, so the DUT's is too: memory that was
# never written reads as zero on both sides.
BENDER_FLAGS += -D ZERO_SIM_MEM
# The reference starts in the boot ROM exactly as the DUT does, so the image is
# always needed; passing it here means no invocation has to remember to.
COSIM_SIM_ARGS := +cosim_bootrom=$(TEST_DIR)/hw/bootrom/cva6tb_bootrom.bin
COSIM_RUN_DEPS := $(TEST_DIR)/hw/bootrom/cva6tb_bootrom.bin
VERILATOR_FLAGS += $(COSIM_SRC)
VERILATOR_FLAGS += -CFLAGS "$(SPIKE_CFLAGS)"
VERILATOR_FLAGS += -LDFLAGS "$(SPIKE_LDFLAGS)"
# The DPI sources and spike's headers, so editing the C++ or building spike
# rebuilds the model
VERILATOR_DEPS += $(COSIM_SRC) $(COSIM_HDR) $(SPIKE_DIR)/include/riscv/processor.h
endif

# Built only while the install is missing, so SPIKE_DIR can also name an install
# made elsewhere, without its sources
$(SPIKE_DIR)/include/riscv/processor.h:
	[ -f $(SPIKE_SRC_DIR)/configure ] || git clone $(SPIKE_URL) $(SPIKE_SRC_DIR)
	cd $(SPIKE_SRC_DIR) && git checkout $(SPIKE_COMMIT)
	mkdir -p $(SPIKE_SRC_DIR)/build
	cd $(SPIKE_SRC_DIR)/build && $(SPIKE_ENV) ../configure --prefix=$(SPIKE_DIR) \
	    CC=$(SPIKE_CC) CXX=$(SPIKE_CXX)
	cd $(SPIKE_SRC_DIR)/build && $(SPIKE_ENV) make -j$(SPIKE_JOBS)
	cd $(SPIKE_SRC_DIR)/build && $(SPIKE_ENV) make install

.PHONY: spike
spike: $(SPIKE_DIR)/include/riscv/processor.h

.PHONY: spike-clean
spike-clean:
	rm -rf $(SPIKE_DIR) $(SPIKE_SRC_DIR)/build

# The reference model without any RTL: the cheapest way to tell a co-simulation
# bug from an RTL bug
$(COSIM_STANDALONE): $(COSIM_DIR)/rv_cosim_standalone.cc $(COSIM_LIB_SRC) $(COSIM_HDR) \
                     $(SPIKE_DIR)/include/riscv/processor.h
	$(SPIKE_ENV) $(SPIKE_CXX) $(SPIKE_CFLAGS) -O2 -Wall -Wextra -o $@ \
	    $(COSIM_DIR)/rv_cosim_standalone.cc $(COSIM_LIB_SRC) $(SPIKE_LDFLAGS)

.PHONY: cosim-standalone
cosim-standalone: $(COSIM_STANDALONE)

# Runs it on the default image, with the boot ROM the testbench uses
.PHONY: cosim-standalone-run
cosim-standalone-run: $(COSIM_STANDALONE) $(TEST_DIR)/hw/bootrom/cva6tb_bootrom.bin
	$(SPIKE_ENV) $(COSIM_STANDALONE) $(abspath $(HEXFILE)) \
	    --bootrom $(TEST_DIR)/hw/bootrom/cva6tb_bootrom.bin

.PHONY: cosim-clean
cosim-clean:
	rm -f $(COSIM_STANDALONE)
