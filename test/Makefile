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

# Target CVA6 configuration
CVA6_CONFIG   ?= cv64a6_imafdchsclic_sv39_wb
# A configuration package to build instead of CVA6_CONFIG's own, such as a
# variant of it with other parameters, without editing core/include
CVA6_CONFIG_PKG ?=
HEXFILE       ?= $(TEST_DIR)/sw/out/hello.hex
MAX_CYCLES    ?= 10000
# Main memory delay per AXI channel, 0 to 100, passed as +MemDelay when set
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
# Cycle accounting of benchmarks (RVFI and the performance probes)
PERF          ?= 0
# The core: 0 the RTL core, 1 the ASIC flow's implementation wrapper
# (asic/src/cva6_asic_top.sv), which has neither RVFI nor the performance
# probes. On QuestaSim, NETLIST gives a netlist of it instead of its RTL,
# simulated without delays, NETLIST_MODELS the script that compiles the
# technology's simulation models of the netlist into the library `gate`, and
# NETLIST_VOPT a file of further vopt arguments the models need, if any (the
# ASIC flow runs this: make -C asic sim).
NETLIST        ?=
NETLIST_MODELS ?=
NETLIST_VOPT   ?=
# QuestaSim: an SDF of NETLIST, whose typical delays the netlist then simulates
# with; empty simulates it without delays
SDF            ?=
ASIC_TOP       ?= $(if $(NETLIST),1,0)
# The clock period [ps]; empty keeps the testbench's 1000
CLK_PERIOD_PS  ?=
# QuestaSim: 1 records the core's switching activity in each workload phase a
# program marks (SAIF, vsim/saif.tcl), into saif/ of the build directory
SAIF           ?= 0

# Goals that must work without a resolved dependency tree.
NODEPS_GOALS := info default %clean bench-check bench-check-update
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

ifeq ($(WAVES),1)
BENDER_FLAGS += -D CVA6TB_WAVES
endif

ifneq ($(LEGACY_TRACE),1)
BENDER_FLAGS += -D CVA6_NO_LEGACY_TRACE
endif

ifeq ($(PERF),1)
BENDER_FLAGS += -t perf
endif

ifeq ($(ASIC_TOP),1)
ifneq ($(filter 1,$(TRACER) $(COSIM) $(PERF)),)
$(error TRACER, COSIM and PERF need RVFI, which the ASIC implementation wrapper has not (ASIC_TOP=1))
endif
BENDER_FLAGS += -t cva6_asic
endif
ifneq ($(and $(NETLIST),$(filter-out 1,$(ASIC_TOP))),)
$(error NETLIST is a netlist of the ASIC implementation wrapper: it needs ASIC_TOP=1)
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

# The generated sources alone, in the uv environment, before builds on other
# hosts that lack it (util/sweep.py --runner)
.PHONY: hw-deps
hw-deps: $(HW_DEPS)

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
########## QuestaSim #############
##################################

VSIM_BUILD_DIR ?= $(VSIM_DIR)/build

# C++ compiler path
# Caution: Questasim requires this to point to the *actual* compiler install path
CXX_PATH := $(shell which $(CXX))

# Silence warning 13314 on input ports defaulting to 'wire' due to -svinputport=relaxed
# Silence warning 2583 on extra checking for conflicts between processes is done at vopt time
VLOG_FLAGS := -timescale 1ns/1ps -suppress 13314 -suppress 2583
VOPT_FLAGS := -suppress 3009 -suppress 8386 -suppress 7061 -error 7 -cpppath $(CXX_PATH)
VSIM_FLAGS :=

# A netlist and the technology's models form their own library, `gate`. The
# RTL top leaves `work`, so the core binds to the netlist, and every module
# below it binds within `gate`, its own library. Elaborating the netlist's top
# in this library alone proves the netlist complete, so every module of the
# core comes from the netlist or the models.
ifneq ($(NETLIST),)
VOPT_FLAGS += -L gate +notimingchecks $(if $(NETLIST_VOPT),-f $(abspath $(NETLIST_VOPT))) -G/test/i_dut/CoreRstDelayPs=1
# With an SDF, cells and wires take its typical delays; each interconnect delay
# applies at its load pin, and as every net of a netlist has a single driver,
# each load keeps its own delay. The SDF's timing checks have no counterpart
# in the simulation (vsim-SDF-3262, -12090), and neither have the interconnects
# between two ports of the netlist's top (feedthroughs, vsim-SDF-3259), whose
# delay is the SoC's at the core's ports. The SoC changes the core's inputs
# at 0.3 of the period (TAppl): after the clock edge has reached the core's
# flip-flops through its clock tree at the typical corner, and early enough for
# the input paths, which the constraints time from the start of the period at
# the slow corner, to meet setup.
ifneq ($(SDF),)
VOPT_FLAGS += -sdftyp /test/i_dut/i_cva6=$(abspath $(SDF)) -suppress 3259,3262,12090 -G/test/i_dut/TAppl=0.3
VSIM_FLAGS += -suppress 3259,3262,12090
else
VOPT_FLAGS += +nospecify
endif
endif
# Netlists and activity files count time in ps
ifneq ($(NETLIST)$(filter 1,$(SAIF)),)
VSIM_FLAGS += -t 1ps
endif
ifneq ($(CLK_PERIOD_PS),)
VOPT_FLAGS += -G/test/ClkPeriodPs=$(CLK_PERIOD_PS)
endif
# Activity recording reads every net of the core and the testbench's marker
ifeq ($(SAIF),1)
VOPT_FLAGS += +acc=npr+$(if $(filter 1,$(ASIC_TOP)),cva6_asic_top,cva6). +acc=r+cva6tb_soc
VSIM_DO    := source $(VSIM_DIR)/saif.tcl; run -all; quit
else
VSIM_DO    := run -all; quit
endif

ifeq ($(DEBUG),0)
VSIM_FLAGS += -c -do "$(VSIM_DO)"
else
VOPT_FLAGS += +acc
VSIM_FLAGS += -wlf cva6tb.wlf -do "log -r /*; run -all"
endif

$(VSIM_BUILD_DIR):
	@mkdir -p $(VSIM_BUILD_DIR)

$(VSIM_BUILD_DIR)/vsim_compile.tcl: $(ROOT_DIR)/Bender.lock $(HW_DEPS) | $(VSIM_BUILD_DIR)
	$(BENDER) script vsim $(BENDER_FLAGS) --vlog-arg="$(VLOG_FLAGS)" > $@
ifneq ($(CVA6_CONFIG_PKG),)
	sed -i 's|"\$$ROOT/core/include/$(CVA6_CONFIG)_config_pkg\.sv"|"$(abspath $(CVA6_CONFIG_PKG))"|' $@
endif
ifeq ($(TRACER),1)
	@echo 'vlog -ccflags "-std=c++17 -O2" $(RV_COMMON_SRC) $(TRACER_SRC)' >> $@
endif
ifneq ($(NETLIST),)
	@echo 'vdel -lib work cva6_asic_top' >> $@
	@echo 'if {[file isdirectory gate]} {file delete -force gate}' >> $@
	@echo 'vlib gate' >> $@
	@echo 'source $(abspath $(NETLIST_MODELS))' >> $@
	@echo 'vlog -work gate -timescale 1ps/1ps $(abspath $(NETLIST))' >> $@
	@echo 'vopt -work gate cva6_asic_top -o cva6_asic_top_check' >> $@
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
	$(VERILATOR_TRACE) $(VERILATOR_THREADS) --x-assign fast --x-initial fast \
	$(if $(CLK_PERIOD_PS),-GClkPeriodPs=$(CLK_PERIOD_PS))

# Shared by both consumers, so listed once: naming it twice duplicates symbols.
ifneq ($(filter 1,$(TRACER) $(COSIM)),)
VERILATOR_FLAGS += $(RV_COMMON_SRC)
endif

ifeq ($(TRACER),1)
VERILATOR_FLAGS += $(TRACER_SRC)
endif

# Co-simulation: spike, the standalone reference and, with COSIM=1, its part of
# this build
include $(TEST_DIR)/hw/cosim/cosim.mk

$(VERILATOR_LOG_DIR):
	@mkdir -p $(VERILATOR_LOG_DIR)

$(VERILATOR_FLIST): $(ROOT_DIR)/Bender.lock $(HW_DEPS)
	$(BENDER) script verilator $(BENDER_FLAGS) -D ASSERTS_OFF -t verilator > $@
ifneq ($(CVA6_CONFIG_PKG),)
	sed -i 's|^.*/core/include/$(CVA6_CONFIG)_config_pkg\.sv$$|$(abspath $(CVA6_CONFIG_PKG))|' $@
endif

# The DPI sources are handed to verilator as positional arguments, which leaves
# them out of Vtest's prerequisites; these list them, so editing the C++ backend
# rebuilds the model.
ifneq ($(filter 1,$(TRACER) $(COSIM)),)
VERILATOR_DEPS += $(RV_COMMON_SRC) $(RV_COMMON_HDR)
endif
ifeq ($(TRACER),1)
VERILATOR_DEPS += $(TRACER_SRC) $(TRACER_HDR)
endif

$(VERILATOR_BUILD_DIR)/Vtest: $(VERILATOR_FLIST) $(VERILATOR_DEPS) | $(VERILATOR_LOG_DIR)
	cd $(VERILATOR_DIR) && $(VERILATOR) $(VERILATOR_FLAGS) --top-module test 2>&1 | tee $(VERILATOR_LOG_DIR)/build.log

.PHONY: verilator-build
verilator-build: $(VERILATOR_BUILD_DIR)/Vtest

.PHONY: verilator-run
verilator-run: $(VERILATOR_BUILD_DIR)/Vtest $(COSIM_RUN_DEPS) | $(VERILATOR_LOG_DIR)
	cd $(VERILATOR_DIR) && ./$(notdir $(VERILATOR_BUILD_DIR))/Vtest +binary=$(abspath $(HEXFILE)) +MaxCycles=$(MAX_CYCLES) $(strip $(SIM_ARGS) $(TB_ARGS) $(TRACER_ARGS) $(COSIM_SIM_ARGS)) 2>&1 | tee $(VERILATOR_LOG_DIR)/$(basename $(notdir $(HEXFILE))).log

# Benchmark cycles against the expected results of the model's configuration,
# sw/bench/expected/<config>.json: a report and a verdict line, **BENCH CHECK
# PASSED** or **BENCH CHECK FAILED** (see sw/bench/README.md). Runs the model
# under verilator/build, built with THREADS=1, like verilator-run.
BENCH_CHECK_SET    ?= full
BENCH_CHECK_STRICT ?= 1
BENCH_CHECK_JOBS   ?= $(shell nproc)

.PHONY: bench-check
bench-check:
	$(TEST_DIR)/util/run-bench.py check --sim $(VERILATOR_BUILD_DIR)/Vtest \
	  --set $(BENCH_CHECK_SET) -j $(BENCH_CHECK_JOBS) $(if $(filter 0,$(BENCH_CHECK_STRICT)),--report-only)

# Rewrites the expected results from a run of every benchmark: commit them with
# the change that moved the numbers
.PHONY: bench-check-update
bench-check-update:
	$(TEST_DIR)/util/run-bench.py check --update --sim $(VERILATOR_BUILD_DIR)/Vtest -j $(BENCH_CHECK_JOBS)

.PHONY: verilator-clean
verilator-clean:
	rm -rf $(VERILATOR_BUILD_DIR) $(VERILATOR_LOG_DIR)
	rm -f $(VERILATOR_FLIST)
	cd $(VERILATOR_DIR) && rm -f *.fst *.fst.hier *.flist *.json *.dasm *.csv *.log cva6tb_trace_hart_*
