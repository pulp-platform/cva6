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
QUESTA    ?= questa-2023.4
VERILATOR ?= verilator

# Target CVA6 configuration
CVA6_CONFIG   ?= cv64a6_rt
HEXFILE       ?= $(TEST_DIR)/sw/out/hello.hex
MAX_CYCLES    ?= 10000
TEST          ?= test_simple
ALLOWED_TESTS := test_simple
DEBUG         ?= 0

ifeq ($(filter $(TEST),$(ALLOWED_TESTS)),)
$(error TEST must be one of: $(ALLOWED_TESTS) (got '$(TEST)'))
endif

.PHONY: default
default: info

.PHONY: clean
clean: vsim-clean verilator-clean

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

$(ROOT_DIR)/Bender.lock:
	$(BENDER) checkout
	touch $@

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

$(VSIM_BUILD_DIR)/vsim_compile.tcl: $(ROOT_DIR)/Bender.lock | $(VSIM_BUILD_DIR)
	$(BENDER) script vsim $(BENDER_FLAGS) --vlog-arg="$(VLOG_FLAGS)" > $@
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
	cd $(VSIM_BUILD_DIR) && $(QUESTA) vsim $(VSIM_FLAGS) test_opt +binary=$(abspath $(HEXFILE)) +MaxCycles=$(MAX_CYCLES)

.PHONY: vsim-clean
vsim-clean:
	rm -rf $(VSIM_BUILD_DIR)

##################################
########## Verilator #############
##################################

VERILATOR_BUILD_DIR := $(VERILATOR_DIR)/build
VERILATOR_LOG_DIR   := $(VERILATOR_DIR)/logs
VERILATOR_FLIST     := $(VERILATOR_DIR)/cva6tb.flist
VERILATOR_WARNINGS  := -Wall -Wno-fatal -Wno-style -Wno-BLKANDNBLK -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-WIDTHCONCAT -Wno-ASCRANGE -Wno-IMPLICITSTATIC
VERILATOR_FLAGS     := --binary -j 0 --Mdir $(VERILATOR_BUILD_DIR) $(VERILATOR_WARNINGS) -O3 -f $(VERILATOR_FLIST) --assert --timing \
	--trace-fst --trace-structs --trace-threads 2 --x-assign fast --x-initial fast

$(VERILATOR_LOG_DIR):
	@mkdir -p $(VERILATOR_LOG_DIR)

$(VERILATOR_FLIST): $(ROOT_DIR)/Bender.lock
	$(BENDER) script verilator $(BENDER_FLAGS) -t verilator > $@

$(VERILATOR_BUILD_DIR)/Vtest: $(VERILATOR_FLIST) | $(VERILATOR_LOG_DIR)
	cd $(VERILATOR_DIR) && $(VERILATOR) $(VERILATOR_FLAGS) --top-module test | tee $(VERILATOR_LOG_DIR)/build.log

.PHONY: verilator-build
verilator-build: $(VERILATOR_BUILD_DIR)/Vtest

.PHONY: verilator-run
verilator-run: $(VERILATOR_BUILD_DIR)/Vtest | $(VERILATOR_LOG_DIR)
	cd $(VERILATOR_DIR) && ./$(notdir $(VERILATOR_BUILD_DIR))/Vtest +binary=$(abspath $(HEXFILE)) +MaxCycles=$(MAX_CYCLES) | tee $(VERILATOR_LOG_DIR)/$(basename $(notdir $(HEXFILE))).log

.PHONY: verilator-clean
verilator-clean:
	rm -rf $(VERILATOR_BUILD_DIR) $(VERILATOR_LOG_DIR)
	rm -f $(VERILATOR_FLIST)
	cd $(VERILATOR_DIR) && rm -f *.fst *.fst.hier *.flist *.json *.dasm *.csv
