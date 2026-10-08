# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
#
# Enrico Zelioli <ezelioli@iis.ee.ethz.ch>

# Variables
MAKEFILE_DIR  := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
ROOT_DIR      := $(abspath $(MAKEFILE_DIR)/..)
TEST_DIR      := $(ROOT_DIR)/test
VSIM_DIR      := $(TEST_DIR)/vsim
VERILATOR_DIR := $(TEST_DIR)/verilator

# Tools
BENDER    ?= bender --suppress E31 -d $(ROOT_DIR)
QUESTA    ?=
VERILATOR ?= verilator
# Prefix for the steps that use the toolchain the Verilator model is built with:
# building spike and the standalone reference. Empty when that is the host's own;
# util/iis-env.sh sets it.
OSEDA     ?=

# Co-simulation. Declared here because it picks the default CVA6 configuration.
COSIM         ?= 0

# Target CVA6 configuration. Spike models neither CLIC nor the testbench's
# peripherals, so co-simulation brings up on the smallest 64-bit config and
# reaches H and CLIC through divergence rules later.
ifeq ($(COSIM),1)
CVA6_CONFIG   ?= cv64a6_imafdc_sv39_wb
endif
CVA6_CONFIG   ?= cv64a6_imafdchsclic_sv39_wb
HEXFILE       ?= $(TEST_DIR)/sw/out/hello.hex
MAX_CYCLES    ?= 10000
# Main memory delay per AXI channel, 0 to 16, passed as +MemDelay when set
MEM_DELAY     ?=
# Extra plusargs forwarded to the simulation, e.g. SIM_ARGS="+fst"
SIM_ARGS      ?=
TEST          ?= test_simple
ALLOWED_TESTS := test_simple
DEBUG         ?= 0
WAVES         ?= 0
# Verilator threads for the simulation model, a BUILD time choice. The default
# suits one simulation at a time, which is the usual case.
# For running many simulations at once, build with THREADS=1 instead.
THREADS       ?= 2
# CVA6's legacy tracer
LEGACY_TRACE  ?= 0
# C++ shared by every RVFI consumer: the decoder, the symbol table and the
# start/stop triggers. Namespace cva6tb_rv, no dependency on any one consumer.
RV_COMMON_DIR := $(TEST_DIR)/hw/common
RV_COMMON_SRC := $(RV_COMMON_DIR)/rv_disasm.cc $(RV_COMMON_DIR)/rv_disasm_spike.cc \
                 $(RV_COMMON_DIR)/rv_symbols.cc $(RV_COMMON_DIR)/rv_trigger.cc
RV_COMMON_HDR := $(RV_COMMON_DIR)/rv_disasm.h $(RV_COMMON_DIR)/rv_symbols.h \
                 $(RV_COMMON_DIR)/rv_trigger.h
# Instruction tracer
TRACER        ?= 0
TRACER_DIR    := $(TEST_DIR)/hw/tracer
TRACER_SRC    := $(TRACER_DIR)/cva6tb_tracer.cc $(TRACER_DIR)/rv_format.cc \
                 $(TRACER_DIR)/rv_stats.cc
TRACER_HDR    := $(TRACER_DIR)/cva6tb_tracer_dpi.h $(TRACER_DIR)/rv_format.h \
                 $(TRACER_DIR)/rv_stats.h
# Instruction tracer settings, each passed as the plusarg of the same name when
# set; see hw/tracer/. Files are made absolute, since the simulation runs in its
# build directory. Symbols default to the program's own ELF beside HEXFILE and
# the boot ROM's, where they exist, outside the legacy format, which stays byte
# compatible with instr_tracer's logs; TRACE_SYMBOLS= turns annotation off.
TRACE_FORMAT  ?=
TRACE_VERBOSE ?=
TRACE_FILE    ?=
TRACE_START   ?=
TRACE_STOP    ?=
TRACE_STATS   ?=
TRACE_SYMBOLS ?= $(if $(filter legacy,$(TRACE_FORMAT)),, \
                   $(wildcard $(basename $(abspath $(HEXFILE))).elf \
                              $(TEST_DIR)/hw/bootrom/cva6tb_bootrom.elf))
# Spike co-simulation. COSIM is declared further up, with the configuration it picks.
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
SPIKE_CFLAGS  = -std=c++20 -I$(SPIKE_DIR)/include
SPIKE_LDFLAGS = -L$(SPIKE_DIR)/lib -lriscv -lsoftfloat -Wl,-rpath,$(SPIKE_DIR)/lib
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

# Goals that must work without a resolved dependency tree.
NODEPS_GOALS := info default %clean
NEEDS_DEPS   := $(filter-out $(NODEPS_GOALS),$(MAKECMDGOALS))

ifneq ($(NEEDS_DEPS),)
ifeq ($(filter $(TEST),$(ALLOWED_TESTS)),)
$(error TEST must be one of: $(ALLOWED_TESTS) (got '$(TEST)'))
endif
endif

.PHONY: default
default: info

.PHONY: clean
clean: vsim-clean verilator-clean cosim-clean

.PHONY: info
info:
	@echo "########################## INFO ##########################"
	@echo "MAKEFILE_DIR  : $(MAKEFILE_DIR)"
	@echo "ROOT_DIR      : $(ROOT_DIR)"
	@echo "TEST_DIR      : $(TEST_DIR)"
	@echo "VSIM_DIR      : $(VSIM_DIR)"
	@echo "VERILATOR_DIR : $(VERILATOR_DIR)"
	@echo "##########################################################"

##################################
############ Bender ##############
##################################

BENDER_FLAGS := -t rtl -t simulation -t cva6 -t $(CVA6_CONFIG) -t cva6tb -t $(TEST)

ifeq ($(ZERO_SIM_MEM),1)
BENDER_FLAGS += -D ZERO_SIM_MEM
endif

ifeq ($(TRACER),1)
BENDER_FLAGS += -t tracer
endif

# Plusargs set through variables, the testbench's and the tracer's. They follow
# SIM_ARGS on the command line, and the first occurrence of a plusarg is the one
# that counts, so SIM_ARGS overrides them.
comma := ,
space := $(subst ,, )
TB_ARGS = $(if $(MEM_DELAY),+MemDelay=$(MEM_DELAY))
ifeq ($(TRACER),1)
TRACER_ARGS = $(if $(TRACE_FORMAT),+trace_format=$(TRACE_FORMAT)) \
              $(if $(TRACE_VERBOSE),+trace_verbose=$(TRACE_VERBOSE)) \
              $(if $(TRACE_FILE),+trace_file=$(abspath $(TRACE_FILE))) \
              $(if $(TRACE_START),+trace_start=$(TRACE_START)) \
              $(if $(TRACE_STOP),+trace_stop=$(TRACE_STOP)) \
              $(if $(TRACE_STATS),+trace_stats=$(abspath $(TRACE_STATS))) \
              $(if $(strip $(TRACE_SYMBOLS)),+trace_symbols=$(subst $(space),$(comma),$(strip $(abspath $(TRACE_SYMBOLS)))))
endif

ifeq ($(COSIM),1)
BENDER_FLAGS += -t cosim
# The reference's memory is zero filled, so the DUT's is too: memory that was
# never written reads as zero on both sides.
BENDER_FLAGS += -D ZERO_SIM_MEM
# The reference starts in the boot ROM exactly as the DUT does, so the image is
# always needed; passing it here means no invocation has to remember to.
COSIM_SIM_ARGS := +cosim_bootrom=$(TEST_DIR)/hw/bootrom/cva6tb_bootrom.bin
COSIM_RUN_DEPS := $(TEST_DIR)/hw/bootrom/cva6tb_bootrom.bin
endif

ifeq ($(WAVES),1)
BENDER_FLAGS += -D CVA6TB_WAVES
endif

ifneq ($(LEGACY_TRACE),1)
BENDER_FLAGS += -D CVA6_NO_LEGACY_TRACE
endif

##################################
##### Hardware dependencies ######
##################################

ifeq ($(shell test -d $(ROOT_DIR)/.bender || echo 1),)

CLINTROOT := $(shell $(BENDER) path clint)
REGTOOL   := $(shell $(BENDER) path register_interface)/vendor/lowrisc_opentitan/util/regtool.py
OTPROOT   := $(shell $(BENDER) path opentitan_peripherals)

CLINTCORES := 1
include $(CLINTROOT)/clint.mk
$(CLINTROOT)/.generated:
	flock -x $@ $(MAKE) clint && touch $@ || (rm -f $@; false)

include $(OTPROOT)/otp.mk
$(OTPROOT)/.generated: $(TEST_DIR)/hw/rv_plic.cfg.hjson
	flock -x $@ sh -c "cp $< $(dir $@)/src/rv_plic/; $(MAKE) -j1 otp" && touch $@ || (rm -f $@; false)

HW_DEPS := $(CLINTROOT)/.generated $(OTPROOT)/.generated

else ifneq ($(NEEDS_DEPS),)
$(error Bender dependencies are not initialized. Run 'bender checkout' in $(ROOT_DIR))
endif

##################################
########### Bootrom ##############
##################################

ifndef RV_BINROOT
RV_BINROOT := $(dir $(shell which riscv64-unknown-elf-gcc 2>/dev/null))
endif
RV_AR      := $(RV_BINROOT)/riscv64-unknown-elf-ar
RV_CC      := $(RV_BINROOT)/riscv64-unknown-elf-gcc
RV_OBJCOPY := $(RV_BINROOT)/riscv64-unknown-elf-objcopy
RV_OBJDUMP := $(RV_BINROOT)/riscv64-unknown-elf-objdump

BROM_CC_FLAGS = -DOT_PLATFORM_RV32 -march=rv64gc_zifencei -mabi=lp64d \
                -mstrict-align -O2 -Wall -Wextra -static -ffunction-sections \
                -fdata-sections -frandom-seed=cheshire -fuse-linker-plugin \
                -nostartfiles -Os -fno-zero-initialized-in-bss \
                -fwhole-program -flto -Wl,-flto -Wl,--gc-sections

$(TEST_DIR)/hw/bootrom/cva6tb_bootrom.bin: $(TEST_DIR)/hw/bootrom/cva6tb_bootrom.ld $(TEST_DIR)/hw/bootrom/cva6tb_bootrom.S $(RV_CC)
	$(RV_CC) -T$< $(BROM_CC_FLAGS) -o $(TEST_DIR)/hw/bootrom/cva6tb_bootrom.elf $(TEST_DIR)/hw/bootrom/cva6tb_bootrom.S
	cd $(@D) && $(RV_OBJDUMP) -d cva6tb_bootrom.elf > cva6tb_bootrom.dump
	$(RV_OBJCOPY) -O binary $(TEST_DIR)/hw/bootrom/cva6tb_bootrom.elf $@

$(TEST_DIR)/hw/bootrom/cva6tb_bootrom.sv: $(TEST_DIR)/hw/bootrom/cva6tb_bootrom.bin $(TEST_DIR)/util/gen_bootrom.py
	$(TEST_DIR)/util/gen_bootrom.py --sv-module cva6tb_bootrom $< > $@

.PHONY: bootrom
bootrom: $(TEST_DIR)/hw/bootrom/cva6tb_bootrom.sv

.PHONY: bootrom-clean
bootrom-clean:
	cd $(TEST_DIR)/hw/bootrom && rm -f *.sv *.elf *.bin *.dump


##################################
############ Spike ###############
##################################

# Configured and built through $(OSEDA), with the toolchain that compiles the
# model, so the two agree on the libstdc++ ABI. CC/CXX name that environment's
# system compiler explicitly, since it inherits the caller's PATH.
SPIKE_CC      ?= /usr/bin/gcc
SPIKE_CXX     ?= /usr/bin/g++

$(SPIKE_SRC_DIR)/configure:
	git clone $(SPIKE_URL) $(SPIKE_SRC_DIR)
	cd $(SPIKE_SRC_DIR) && git checkout $(SPIKE_COMMIT)

$(SPIKE_DIR)/include/riscv/processor.h: $(SPIKE_SRC_DIR)/configure
	mkdir -p $(SPIKE_SRC_DIR)/build
	cd $(SPIKE_SRC_DIR)/build && $(OSEDA) ../configure --prefix=$(SPIKE_DIR) \
	    CC=$(SPIKE_CC) CXX=$(SPIKE_CXX)
	cd $(SPIKE_SRC_DIR)/build && $(OSEDA) make -j$(SPIKE_JOBS)
	cd $(SPIKE_SRC_DIR)/build && $(OSEDA) make install

.PHONY: spike
spike: $(SPIKE_DIR)/include/riscv/processor.h

.PHONY: spike-clean
spike-clean:
	rm -rf $(SPIKE_DIR) $(SPIKE_SRC_DIR)/build

##################################
######## Co-simulation ###########
##################################

# The reference model without any RTL: the cheapest way to tell a co-simulation
# bug from an RTL bug. Built through $(OSEDA), like everything that links against
# spike, and run where the model runs.

$(COSIM_STANDALONE): $(COSIM_DIR)/rv_cosim_standalone.cc $(COSIM_LIB_SRC) $(COSIM_HDR) \
                     $(SPIKE_DIR)/include/riscv/processor.h
	$(OSEDA) $(SPIKE_CXX) $(SPIKE_CFLAGS) -O2 -Wall -Wextra -o $@ \
	    $(COSIM_DIR)/rv_cosim_standalone.cc $(COSIM_LIB_SRC) $(SPIKE_LDFLAGS)

.PHONY: cosim-standalone
cosim-standalone: $(COSIM_STANDALONE)

# Runs it on the default image, with the boot ROM the testbench uses.
.PHONY: cosim-standalone-run
cosim-standalone-run: $(COSIM_STANDALONE) $(TEST_DIR)/hw/bootrom/cva6tb_bootrom.bin
	$(COSIM_STANDALONE) $(abspath $(HEXFILE)) \
	    --bootrom $(TEST_DIR)/hw/bootrom/cva6tb_bootrom.bin

.PHONY: cosim-clean
cosim-clean:
	rm -f $(COSIM_STANDALONE)

##################################
########## QuestaSim #############
##################################

VSIM_BUILD_DIR := $(VSIM_DIR)/build

# C++ compiler path
# Caution: Questasim requires this to point to the *actual* compiler install path
CXX_PATH := $(shell which $(CXX))

# Silence warning 13314 on input ports defaulting to 'wire' due to -svinputport=relaxed
# Silence warning 2583 on extra checking for conflicts between processes is done at vopt time
VLOG_FLAGS := -timescale 1ns/1ps -suppress 13314 -suppress 2583
VOPT_FLAGS := -suppress 3009 -suppress 8386 -suppress 7061 -error 7 -cpppath $(CXX_PATH)
VSIM_FLAGS :=

ifeq ($(DEBUG),0)
VSIM_FLAGS += -c -do "run -all; quit"
else
VOPT_FLAGS += +acc
VSIM_FLAGS += -wlf cva6tb.wlf -do "log -r /*; run -all"
endif

$(VSIM_BUILD_DIR):
	@mkdir -p $(VSIM_BUILD_DIR)

$(VSIM_BUILD_DIR)/vsim_compile.tcl: $(ROOT_DIR)/Bender.lock $(HW_DEPS) | $(VSIM_BUILD_DIR)
	$(BENDER) script vsim $(BENDER_FLAGS) --vlog-arg="$(VLOG_FLAGS)" > $@
ifeq ($(TRACER),1)
	@echo 'vlog -ccflags "-std=c++17 -O2" $(RV_COMMON_SRC) $(TRACER_SRC)' >> $@
endif
	@echo 'vopt $(VOPT_FLAGS) test -o test_opt' >> $@

$(VSIM_BUILD_DIR)/vsim_compile.log: $(VSIM_BUILD_DIR)/vsim_compile.tcl
	cd $(VSIM_BUILD_DIR) && $(QUESTA) vsim -c -do "quit -code [source $<]" | tee $@

$(VSIM_BUILD_DIR)/vsim_compile.err: $(VSIM_BUILD_DIR)/vsim_compile.log
	@echo "--- QuestaSim compilation report ---" > $@
	@echo "Errors:" >> $@
	@grep "Error:" $< >> $@ || true
	@echo "" >> $@
	@echo "Warnings:" >> $@
	@grep "Warning:" $< >> $@ || true
	@cat $@

.PHONY: vsim-build
vsim-build: $(VSIM_BUILD_DIR)/vsim_compile.err

.PHONY: vsim-run
vsim-run: $(VSIM_BUILD_DIR)/vsim_compile.err
	cd $(VSIM_BUILD_DIR) && $(QUESTA) vsim $(VSIM_FLAGS) test_opt +binary=$(abspath $(HEXFILE)) +MaxCycles=$(MAX_CYCLES) $(strip $(SIM_ARGS) $(TB_ARGS) $(TRACER_ARGS))

.PHONY: vsim-clean
vsim-clean:
	rm -rf $(VSIM_BUILD_DIR)

##################################
########## Verilator #############
##################################

ifeq ($(WAVES),1)
VERILATOR_TRACE := --trace-fst --trace-structs --trace-threads 2
else
VERILATOR_TRACE :=
endif

ifneq ($(THREADS),1)
VERILATOR_THREADS := --threads $(THREADS)
else
VERILATOR_THREADS :=
endif

VERILATOR_JOBS      ?= 0
VERILATOR_BUILD_DIR := $(VERILATOR_DIR)/build
VERILATOR_LOG_DIR   := $(VERILATOR_DIR)/logs
VERILATOR_FLIST     := $(VERILATOR_DIR)/cva6tb.flist
VERILATOR_WARNINGS  := -Wall -Wno-fatal -Wno-style -Wno-BLKANDNBLK -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-WIDTHCONCAT -Wno-ASCRANGE -Wno-IMPLICITSTATIC
VERILATOR_FLAGS     := --binary -j $(VERILATOR_JOBS) --Mdir $(VERILATOR_BUILD_DIR) $(VERILATOR_WARNINGS) -O3 -f $(VERILATOR_FLIST) --assert --timing \
	$(VERILATOR_TRACE) $(VERILATOR_THREADS) --x-assign fast --x-initial fast

# Shared by both consumers, so listed once: naming it twice duplicates symbols.
ifneq ($(filter 1,$(TRACER) $(COSIM)),)
VERILATOR_FLAGS += $(RV_COMMON_SRC)
endif

ifeq ($(TRACER),1)
VERILATOR_FLAGS += $(TRACER_SRC)
endif

ifeq ($(COSIM),1)
VERILATOR_FLAGS += $(COSIM_SRC)
VERILATOR_FLAGS += -CFLAGS "$(SPIKE_CFLAGS)"
VERILATOR_FLAGS += -LDFLAGS "$(SPIKE_LDFLAGS)"
endif

$(VERILATOR_LOG_DIR):
	@mkdir -p $(VERILATOR_LOG_DIR)

$(VERILATOR_FLIST): $(ROOT_DIR)/Bender.lock $(HW_DEPS)
	$(BENDER) script verilator $(BENDER_FLAGS) -D ASSERTS_OFF -t verilator > $@

# The DPI sources are handed to verilator as positional arguments, which leaves
# them out of Vtest's prerequisites; these list them, so editing the C++ backend
# rebuilds the model.
ifneq ($(filter 1,$(TRACER) $(COSIM)),)
VERILATOR_DEPS += $(RV_COMMON_SRC) $(RV_COMMON_HDR)
endif
ifeq ($(TRACER),1)
VERILATOR_DEPS += $(TRACER_SRC) $(TRACER_HDR)
endif
ifeq ($(COSIM),1)
VERILATOR_DEPS += $(COSIM_SRC) $(COSIM_HDR) $(SPIKE_DIR)/include/riscv/processor.h
endif

$(VERILATOR_BUILD_DIR)/Vtest: $(VERILATOR_FLIST) $(VERILATOR_DEPS) | $(VERILATOR_LOG_DIR)
	cd $(VERILATOR_DIR) && $(VERILATOR) $(VERILATOR_FLAGS) --top-module test 2>&1 | tee $(VERILATOR_LOG_DIR)/build.log

.PHONY: verilator-build
verilator-build: $(VERILATOR_BUILD_DIR)/Vtest

.PHONY: verilator-run
verilator-run: $(VERILATOR_BUILD_DIR)/Vtest $(COSIM_RUN_DEPS) | $(VERILATOR_LOG_DIR)
	cd $(VERILATOR_DIR) && ./$(notdir $(VERILATOR_BUILD_DIR))/Vtest +binary=$(abspath $(HEXFILE)) +MaxCycles=$(MAX_CYCLES) $(strip $(SIM_ARGS) $(TB_ARGS) $(TRACER_ARGS) $(COSIM_SIM_ARGS)) 2>&1 | tee $(VERILATOR_LOG_DIR)/$(basename $(notdir $(HEXFILE))).log

.PHONY: verilator-clean
verilator-clean:
	rm -rf $(VERILATOR_BUILD_DIR) $(VERILATOR_LOG_DIR)
	rm -f $(VERILATOR_FLIST)
	cd $(VERILATOR_DIR) && rm -f *.fst *.fst.hier *.flist *.json *.dasm *.csv *.log cva6tb_trace_hart_*
