# Copyright 2018 ETH Zurich and University of Bologna.
# Copyright and related rights are licensed under the Solderpad Hardware
# License, Version 0.51 (the "License"); you may not use this file except in
# compliance with the License.  You may obtain a copy of the License at
# http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
# or agreed to in writing, software, hardware and materials distributed under
# this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
# CONDITIONS OF ANY KIND, either express or implied. See the License for the
# specific language governing permissions and limitations under the License.
#
# Author: Michael Schaffner <schaffner@iis.ee.ethz.ch>, ETH Zurich
# Date: 15.08.2018
# Description: Makefile for the dcache testbench.
target          ?= cv64a6_imafdc_sv39
library         ?= work
toplevel        ?= tb
src-list        := tb.list
inc-path        := $(shell pwd)/hdl/
src             := $(shell xargs printf '\n%s' < $(src-list)  | cut -b 1-)
compile_flag    += +cover+i_dut -incr -64 -nologo -svinputport=compat -override_timescale 1ns/1ps -suppress 2583 -suppress 13262 -suppress 2986 +cover
sim_opts        += -64 -coverage -classdebug -voptargs="+acc"
questa_version  ?= ${QUESTASIM_VERSION}
incdir          += ../common/ ../../../vendor/pulp-platform/axi/include/ ../../../vendor/pulp-platform/common_cells/include/  ../../../vendor/planv/ace/include/
wave            := wave.do

ifeq ($(toplevel),tb_ace)
	wave            := wave_ace.do
	target          = cv64a6_imafdc_sv39_wb
endif
ifeq ($(toplevel),tb_ace_direct)
	wave            := wave_ace_direct.do
	target          = cv64a6_imafdc_sv39_wb
endif
# add package to start of file list
cva6_config_pkg := ../../../core/include/$(target)_config_pkg.sv
src  := $(cva6_config_pkg) $(src)

AVOID_BUG_32 = 1 # don't trigger https://github.com/pulp-platform/axi_riscv_atomics/issues/32
sim_opts += +AVOID_BUG_32=$(AVOID_BUG_32)

# Iterate over all include directories and write them with +incdir+ prefixed
# +incdir+ works for Verilator and QuestaSim
list_incdir := $(foreach dir, ${incdir}, +incdir+$(dir))

build: clean
	@vlib${questa_version} $(library)
	@vlog${questa_version} -work $(library) -pedanticerrors $(src) $(compile_flag) $(list_incdir)
	@touch $(library)/.build

# this starts modelsim with gui
sim: build
	vsim${questa_version} -lib $(library) $(toplevel) -do "do $(wave)" $(sim_opts)

# batch mode without gui
simc: build
	vsim${questa_version} -lib $(library) $(toplevel) -c -do "run -all; exit" $(sim_opts)

clean:
	rm -rf $(library)

.PHONY: clean simc sim build
