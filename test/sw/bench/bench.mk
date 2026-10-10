# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
#
# Benchmarks (CoreMark, Embench-IoT, RaiderSTREAM) for the standalone
# testbench, included by ../Makefile. Everything lands in out/bench/: one
# <name>.elf/.hex/.dump per benchmark, plus build-info.txt recording what
# produced them. See README.md.

BENCH_DIR     := $(SW_DIR)/bench
BENCH_OUT_DIR := $(OUT_DIR)/bench
BENCH_OBJ_DIR := $(BENCH_OUT_DIR)/obj

# Sources are pinned to the revisions Philipp Schilk evaluated CVA6S+ with
# (MSc thesis, 2025): Embench-IoT master as of 2023-11-20, which his fork was
# based on, and CoreMark master as of 2025-05-01.
EMBENCH_REV   := 54fd9a0f10aabc662991567ca19ff3e61bfd3939
COREMARK_REV  := 1f483d5b8316753a742cbf5590caf5bd0a4e4777
EMBENCH_DIR   := $(DEPS_DIR)/embench-iot
COREMARK_DIR  := $(DEPS_DIR)/coremark

# RaiderSTREAM: master as of 2026-03-27.
RAIDERSTREAM_REV := 1e4f96392fd1b985fec2fec2120f03e31ed2b73a
RAIDERSTREAM_DIR := $(DEPS_DIR)/raiderstream

# Source paths are compiled in relative to sw/ (Embench's support code asserts,
# so __FILE__ is in five programs' data): the binaries do not depend on where
# the repository is checked out.
BENCH_PREFIX_MAP := -ffile-prefix-map=$(SW_DIR)=.

# Support library: startup, ROI hooks, console and printf. Built without LTO,
# unlike libcva6, so that it links with any benchmark's flags.
BENCH_LIB_FLAGS := -march=rv64gc_zifencei -mabi=lp64d -mcmodel=medany -O2 -g \
                   -ffunction-sections -fdata-sections -Wall -Wextra $(BENCH_PREFIX_MAP)
BENCH_INC       := -I$(BENCH_DIR)/include -I$(INC_DIR) -I$(DEPS_INC)
BENCH_CRT0      := $(BENCH_OBJ_DIR)/crt0.o
BENCH_LIB       := $(BENCH_OUT_DIR)/libbench.a
BENCH_LIB_OBJS  := $(BENCH_OBJ_DIR)/bench.o $(BENCH_OBJ_DIR)/console.o $(BENCH_OBJ_DIR)/printf.o
BENCH_LDFLAGS   := -nostartfiles -Wl,--gc-sections -T$(LD_DIR)/link.ld

# CoreMark: Philipp's flags, but zicsr spelled out (GCC 12 and later no longer
# imply it, and the cycle counter read needs it).
COREMARK_ITERATIONS ?= 15
COREMARK_CFLAGS     := -O3 -march=rv64imafdc_zicsr_zifencei -mabi=lp64d -mcmodel=medany \
                       -fno-builtin-printf
COREMARK_DEFS       := -DITERATIONS=$(COREMARK_ITERATIONS) -DPERFORMANCE_RUN=1 \
                       -DFLAGS_STR='"$(COREMARK_CFLAGS)"'
COREMARK_SRCS       := $(addprefix $(COREMARK_DIR)/,core_list_join.c core_main.c core_matrix.c \
                       core_state.c core_util.c) $(BENCH_DIR)/coremark/core_portme.c

# Embench's flags, for the benchmarks that bring none of their own.
BENCH_CFLAGS := -march=rv64gc_zifencei -mabi=lp64d -mcmodel=medany -O3 -g -fdata-sections \
                -ffunction-sections -fno-tree-loop-distribute-patterns -funroll-all-loops \
                -falign-jumps=4 -falign-functions=16 -Wall -Wextra $(BENCH_PREFIX_MAP)

# RaiderSTREAM: one program per access pattern in RS_PATTERNS, rs-<pattern>,
# each running its four kernels on arrays of RS_ARRAY_SIZE doubles (a power of
# two). By default every pattern, on arrays eight times the data cache.
RS_PATTERNS   ?= seq gather scatter sg central
RS_ARRAY_SIZE ?= 32768
RS_ELFS       := $(patsubst %,$(BENCH_OUT_DIR)/rs-%.elf,$(RS_PATTERNS))


$(EMBENCH_DIR)/:
	git clone https://github.com/embench/embench-iot.git $(EMBENCH_DIR)
	cd $(EMBENCH_DIR) && git checkout $(EMBENCH_REV)

$(COREMARK_DIR)/:
	git clone https://github.com/eembc/coremark.git $(COREMARK_DIR)
	cd $(COREMARK_DIR) && git checkout $(COREMARK_REV)

$(RAIDERSTREAM_DIR)/:
	git clone https://github.com/michael-beebe/RaiderSTREAM.git $(RAIDERSTREAM_DIR)
	cd $(RAIDERSTREAM_DIR) && git checkout $(RAIDERSTREAM_REV)

$(BENCH_OBJ_DIR)/crt0.o: $(BENCH_DIR)/lib/crt0.S
	@mkdir -p $(dir $@)
	$(RV_CC) $(BENCH_LIB_FLAGS) -c $< -o $@

$(BENCH_OBJ_DIR)/bench.o: $(BENCH_DIR)/lib/bench.c $(BENCH_DIR)/include/bench.h | $(DEPS_DIR)/printf
	@mkdir -p $(dir $@)
	$(RV_CC) $(BENCH_LIB_FLAGS) $(BENCH_INC) -c $< -o $@

$(BENCH_OBJ_DIR)/console.o: $(LIB_DIR)/console.c
	@mkdir -p $(dir $@)
	$(RV_CC) $(BENCH_LIB_FLAGS) $(BENCH_INC) -c $< -o $@

$(BENCH_OBJ_DIR)/printf.o: | $(DEPS_DIR)/printf
	@mkdir -p $(dir $@)
	$(RV_CC) $(BENCH_LIB_FLAGS) $(BENCH_INC) -c $(DEPS_DIR)/printf/printf.c -o $@

$(BENCH_LIB): $(BENCH_LIB_OBJS)
	$(RV_AR) rcs $@ $^

# The path map is not in COREMARK_CFLAGS, which CoreMark prints.
$(BENCH_OUT_DIR)/coremark.elf: $(BENCH_DIR)/coremark/core_portme.c $(BENCH_DIR)/coremark/core_portme.h \
                               $(BENCH_CRT0) $(BENCH_LIB) | $(COREMARK_DIR)/
	$(RV_CC) $(COREMARK_CFLAGS) $(COREMARK_DEFS) $(BENCH_PREFIX_MAP) -I$(COREMARK_DIR) \
	    -I$(BENCH_DIR)/coremark $(BENCH_INC) $(BENCH_LDFLAGS) -o $@ $(BENCH_CRT0) $(COREMARK_SRCS) \
	    $(BENCH_LIB) -lm

.PHONY: bench-coremark
bench-coremark: $(BENCH_OUT_DIR)/coremark.hex $(BENCH_OUT_DIR)/coremark.dump

# Upstream's OpenMP kernels, compiled as they are but without OpenMP, against an
# empty omp.h: each runs as a plain loop.
$(BENCH_OBJ_DIR)/rs-kernels.o: $(BENCH_DIR)/raiderstream/omp.h | $(RAIDERSTREAM_DIR)/
	@mkdir -p $(dir $@)
	$(RV_CC) $(BENCH_CFLAGS) -Wno-unknown-pragmas -Wno-unused-parameter -DSTREAM_TYPE=double \
	    -I$(BENCH_DIR)/raiderstream -c $(RAIDERSTREAM_DIR)/src/RaiderSTREAM/Impl/RS_OMP/RS_OMP_IMPL.c -o $@

$(BENCH_OUT_DIR)/rs-%.elf: $(BENCH_DIR)/raiderstream/raiderstream.c $(BENCH_OBJ_DIR)/rs-kernels.o \
                           $(BENCH_DIR)/include/bench.h $(BENCH_CRT0) $(BENCH_LIB)
	$(RV_CC) $(BENCH_CFLAGS) -DSTREAM_TYPE=double -DRS_ARRAY_SIZE=$(RS_ARRAY_SIZE) \
	    -DRS_PATTERN=RS_$$(echo $* | tr a-z A-Z) $(BENCH_INC) $(BENCH_LDFLAGS) -o $@ \
	    $(BENCH_CRT0) $< $(BENCH_OBJ_DIR)/rs-kernels.o $(BENCH_LIB)

.PHONY: bench-raiderstream
bench-raiderstream: $(RS_ELFS) $(RS_ELFS:.elf=.hex) $(RS_ELFS:.elf=.dump)

# Embench's own build_all.py does the compiling, against a board description
# copied into its tree; the environment carries the paths board.cfg needs.
# Each benchmark is then renamed em-<name> next to CoreMark.
.PHONY: bench-embench
bench-embench: $(BENCH_CRT0) $(BENCH_LIB) | $(EMBENCH_DIR)/
	rm -rf $(EMBENCH_DIR)/config/riscv32/boards/cva6tb
	cp -r $(BENCH_DIR)/embench $(EMBENCH_DIR)/config/riscv32/boards/cva6tb
	cd $(EMBENCH_DIR) && BENCH_CC=$(RV_CC) BENCH_INC_DIR=$(BENCH_DIR)/include \
	    BENCH_PRINTF_DIR=$(DEPS_INC) BENCH_LINK_LD=$(LD_DIR)/link.ld \
	    BENCH_CRT0=$(BENCH_CRT0) BENCH_LIB=$(BENCH_LIB) BENCH_PREFIX_MAP=$(BENCH_PREFIX_MAP) \
	    python3 ./build_all.py --arch riscv32 --chip generic --board cva6tb \
	        --builddir $(BENCH_OUT_DIR)/embench --logdir $(BENCH_OUT_DIR)/embench-logs \
	        --timeout 120 --clean
	for src in $(EMBENCH_DIR)/src/*/; do \
	    b=$$(basename $$src); \
	    cp $(BENCH_OUT_DIR)/embench/src/$$b/$$b $(BENCH_OUT_DIR)/em-$$b.elf || exit 1; \
	    $(RV_OBJCOPY) -O verilog $(BENCH_OUT_DIR)/em-$$b.elf $(BENCH_OUT_DIR)/em-$$b.hex; \
	    $(RV_OBJDUMP) -d $(BENCH_OUT_DIR)/em-$$b.elf > $(BENCH_OUT_DIR)/em-$$b.dump; \
	done

$(BENCH_OUT_DIR)/%.hex: $(BENCH_OUT_DIR)/%.elf
	$(RV_OBJCOPY) -O verilog $< $@

$(BENCH_OUT_DIR)/%.dump: $(BENCH_OUT_DIR)/%.elf
	$(RV_OBJDUMP) -d $< > $@

# What produced the binaries, for comparing results across builds.
.PHONY: bench-info
bench-info: | $(EMBENCH_DIR)/ $(COREMARK_DIR)/ $(RAIDERSTREAM_DIR)/
	@mkdir -p $(BENCH_OUT_DIR)
	@{ echo "compiler:      $$($(RV_CC) --version | head -1) ($(RV_CC))"; \
	   echo "embench-iot:   $$(git -C $(EMBENCH_DIR) rev-parse HEAD)"; \
	   echo "coremark:      $$(git -C $(COREMARK_DIR) rev-parse HEAD)"; \
	   echo "coremark iter: $(COREMARK_ITERATIONS)"; \
	   echo "raiderstream:  $$(git -C $(RAIDERSTREAM_DIR) rev-parse HEAD), $(RS_ARRAY_SIZE) elements"; \
	   echo "bench sources: $$(git -C $(BENCH_DIR) log -1 --format=%H -- . 2>/dev/null)$$(git -C $(BENCH_DIR) diff --quiet -- . 2>/dev/null || echo ' (modified)')"; \
	 } > $(BENCH_OUT_DIR)/build-info.txt
	@cat $(BENCH_OUT_DIR)/build-info.txt

.PHONY: bench
bench: bench-coremark bench-embench bench-raiderstream bench-info

# The set CI checks, and make bench-check with it, built into out/bench-ci:
# every benchmark but rs-sg, and RaiderSTREAM on arrays of half the default
# size, still four times the data cache. That halves the longest simulations.
BENCH_CI_VARS := RS_PATTERNS="seq gather scatter central" RS_ARRAY_SIZE=16384

.PHONY: bench-ci
bench-ci:
	$(MAKE) -C $(SW_DIR) bench BENCH_OUT_DIR=$(OUT_DIR)/bench-ci $(BENCH_CI_VARS)

.PHONY: bench-clean
bench-clean:
	rm -rf $(BENCH_OUT_DIR) $(OUT_DIR)/bench-ci $(EMBENCH_DIR)/config/riscv32/boards/cva6tb
