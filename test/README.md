# CVA6 standalone testbench

A self-contained SoC testbench for CVA6, independent from the OpenHW `verif/` flow.
It wraps the core in a minimal AXI/regbus SoC, preloads a program image into simulation
memory and polls an end-of-computation register to decide pass or fail.

Verilator is the default simulator: it is open source, so every flow described here is
reproducible without a license. QuestaSim is supported by the same [Makefile](Makefile) for
those who have access to it.

## Directory layout

| Path | Content |
| --- | --- |
| [hw/](hw/) | Testbench SoC and its peripherals (SystemVerilog) |
| &nbsp;&nbsp;&nbsp;&nbsp;[bootrom/](hw/bootrom/) | Boot ROM sources and the generated `cva6tb_bootrom.sv` |
| [sw/](sw/) | Bare-metal test programs, support library and linker script |
| &nbsp;&nbsp;&nbsp;&nbsp;[deps/](sw/deps/) | External sources cloned on demand (`printf`, `riscv-tests`, `riscv-hyp-tests`) and their patches |
| &nbsp;&nbsp;&nbsp;&nbsp;[include/](sw/include/) | Headers of the support library and inline CSR helpers |
| &nbsp;&nbsp;&nbsp;&nbsp;[lib/](sw/lib/) | Support library sources: startup code, console driver, CLIC helpers |
| &nbsp;&nbsp;&nbsp;&nbsp;[link/](sw/link/) | Linker script: memory regions and peripheral base addresses |
| &nbsp;&nbsp;&nbsp;&nbsp;[tests/](sw/tests/) | Test programs, one binary per `.c`/`.S` file |
| [util/](util/) | Boot ROM generator, `riscv-tests` regression runner |
| [verilator/](verilator/) | Verilator build directory and simulation logs |
| [vsim/](vsim/) | QuestaSim build directory and wave scripts |

## Prerequisites

- [Bender](https://github.com/pulp-platform/bender) for dependency and file list handling
- `riscv64-unknown-elf-` GCC toolchain (override the search path with `RV_BINROOT`)
- [Verilator](https://github.com/verilator/verilator) (`VERILATOR`, defaults to `verilator`)
- Optionally QuestaSim (`QUESTA`, defaults to `questa-2025.3`)
- Python 3.12 for the hardware generators; the environment is pinned with
  [uv](https://docs.astral.sh/uv/) (see [pyproject.toml](pyproject.toml)) — run `uv sync`
  in this directory and activate `.venv` before building

Dependencies must be checked out once from the repository root:

```sh
bender checkout
```

## Quick start

```sh
make -C sw all                     # build the test programs under sw/tests
make verilator-build               # elaborate the testbench
make verilator-run HEXFILE=sw/out/hello.hex MAX_CYCLES=100000
```

If you have access to QuestaSim, the flow is symmetric:

```sh
make vsim-build
make vsim-run HEXFILE=sw/out/hello.hex
make vsim-run DEBUG=1               # GUI-friendly: +acc, full signal logging to cva6tb.wlf
```

`make clean` removes both build directories.

### Makefile variables

| Variable | Default | Meaning |
| --- | --- | --- |
| `CVA6_CONFIG` | `cv64a6_imafdchsclic_sv39_wb` | CVA6 configuration target passed to Bender |
| `HEXFILE` | `sw/out/hello.hex` | Verilog hex image to preload |
| `MAX_CYCLES` | `10000` | Simulation timeout in core cycles |
| `TEST` | `test_simple` | Testbench top (currently the only one) |
| `ZERO_SIM_MEM` | unset | Initialise simulation memories to zero instead of random |
| `VERILATOR_JOBS` | `0` | Verilator parallelism (`0` = all cores) |
| `DEBUG` | `0` | QuestaSim only: enable access and waveform logging |

## Testbench structure

[hw/cva6tb_test_simple.sv](hw/cva6tb_test_simple.sv) is the simulation top. It generates the
1 GHz core clock and the 1 MHz RTC, instantiates the SoC, preloads the binary, then polls the
EOC register every cycle until it is set or `MaxCycles` elapses.

[hw/cva6tb_soc.sv](hw/cva6tb_soc.sv) is the device under test: CVA6 as the single AXI master,
an AXI crossbar with four slaves (error slave, regbus bridge, SPM, main memory), and a regbus
demux fanning out to the peripherals. Atomics are handled by `axi_riscv_atomics` in front of
both memories, and main memory sits behind an AXI delayer (10-cycle channel delay) to emulate
realistic latency. Accesses to the uncached DRAM alias at `0x4000_0000` are remapped onto the
cached region before reaching the memory model.

Parameters, types and the address map live in [hw/cva6tb_pkg.sv](hw/cva6tb_pkg.sv).

### Address map

| Region | Base | Size |
| --- | --- | --- |
| AXI error slave | `0x0000_0000` | 64 KiB |
| Boot ROM | `0x0001_0000` | 4 KiB |
| CLINT | `0x0204_0000` | 256 KiB |
| PLIC | `0x0C00_0000` | 64 MiB |
| Platform control registers | `0x1000_0000` | 4 KiB |
| Simulation console | `0x1000_1000` | 4 KiB |
| RTC timer | `0x1000_2000` | 4 KiB |
| CLIC | `0x1004_0000` | 192 KiB |
| SPM | `0x2000_0000` | 1 MiB |
| DRAM (uncached alias) | `0x4000_0000` | 256 MiB |
| DRAM | `0x8000_0000` | 256 MiB |

### Peripherals

- **[cva6tb_regs.sv](hw/cva6tb_regs.sv)** — platform control registers. Offset `0x0` is the
  read-only boot mode; offset `0x4` is the EOC register: bit 0 signals end of computation,
  bits 31:1 carry the return code.
- **[cva6tb_sim_console.sv](hw/cva6tb_sim_console.sv)** — byte-write character sink. Writes
  are buffered and flushed to stdout as `[CON]` lines on newline.
- **[cva6tb_rtc_timer.sv](hw/cva6tb_rtc_timer.sv)** — RTC-driven 64-bit counter with a compare
  register, wired to both the CLIC and the PLIC.
- **[cva6tb_clk_rst_gen.sv](hw/cva6tb_clk_rst_gen.sv)** — clock and reset generation.
- **CLIC / CLINT / PLIC** — external IPs. Interrupt lines 0–17 (CLIC) and 0–15 (PLIC) are
  reserved for internal sources (`msip`, `mtip`, RTC timer); the remaining lines are exposed
  as SoC inputs and currently tied off.

The PLIC is regenerated from [hw/rv_plic.cfg.hjson](hw/rv_plic.cfg.hjson) and the CLINT from
`clint.mk`; both run automatically as part of the build.

### Boot flow

The testbench reads the first line of the hex image to determine its load address:

- image at `0x2000_0000` → loaded into the SPM, boot mode `1`
- anything else → loaded into main memory, boot mode `0`

The core always resets into the boot ROM ([hw/bootrom/cva6tb_bootrom.S](hw/bootrom/cva6tb_bootrom.S)),
which clears the register file, sets up `sp`/`gp` and a trap handler, reads the boot mode
register and jumps to the corresponding memory. When the program returns, the boot ROM writes
its return code into the EOC register and parks in a `wfi` loop.

Regenerate the boot ROM after editing it with `make bootrom` (`make bootrom-clean` to reset).

### Pass/fail protocol

The testbench passes when the EOC register is set and the return code equals `RetCodeSuccess`
(default `0`); it fails on a mismatching code or on timeout. The result is printed as
`**SIMULATION PASSED**` or `**SIMULATION FAILED**`.

### Simulator plusargs

| Plusarg | Default | Meaning |
| --- | --- | --- |
| `+binary=<path>` | — | Verilog hex image to preload |
| `+MaxCycles=<n>` | `10000` | Cycle timeout |
| `+RetCodeSuccess=<n>` | `0` | Return code that counts as a pass |

## Test software

[sw/](sw/) builds every `.c` and `.S` file under [sw/tests/](sw/tests/) into an ELF, a
disassembly and a hex image in `sw/out/`. Programs are linked against `libcva6.a`, built from
[sw/lib/](sw/lib/) (startup code, console driver, CLIC helpers) plus a vendored `printf`, and
placed by [sw/link/link.ld](sw/link/link.ld), which also exports the peripheral base addresses
used by the headers in [sw/include/](sw/include/).

```sh
make -C sw all       # build all tests
make -C sw clean     # remove sw/out
```

## External test suites

`riscv-tests` and `riscv-hyp-tests` are cloned into `sw/deps/` and patched for this platform
on first build:

```sh
make -C sw riscv-tests           # clone, patch and build the ISA tests
make -C sw riscv-hyp-tests       # clone, patch and build the hypervisor tests
```

The ISA regression is driven by [util/run-riscv-tests.sh](util/run-riscv-tests.sh), which runs
the selected tests in parallel on the Verilator binary and summarises the results. It expects
`make verilator-build` to have been run first:

```sh
NJOBS=8 ./util/run-riscv-tests.sh
```

Per-test logs are written to `verilator/logs/`. Note that the suite uses `+RetCodeSuccess=1`,
matching the `riscv-tests` convention.

The hypervisor tests produce a single image that can be run directly:

```sh
make verilator-run HEXFILE=sw/deps/riscv-hyp-tests/build/cva6/rvh_test.hex MAX_CYCLES=2000000
```

## Waveforms

Verilator always dumps `cva6tb.fst` into [verilator/](verilator/), viewable with GTKWave or
Surfer. On QuestaSim, run with `DEBUG=1` and load the wave scripts in
[vsim/waves/](vsim/waves/):

```tcl
do ../waves/cva6.tcl
do ../waves/clic.tcl
```
