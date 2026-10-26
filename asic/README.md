# CVA6 ASIC implementation

The design side of CVA6's ASIC flow: synthesis and place and route of the core
for power, performance and area evaluation, and simulation of its netlists in
the testbench of [test/](../test/). Each technology's flow lives in a
repository of its own, which is not open; `asic/Makefile` clones it into
`asic/<tech>` at a pinned commit and includes its `<tech>.mk`.

## Directory layout

| Path | Content |
| --- | --- |
| [Makefile](Makefile) | Knobs, SRAM inventory, run records and the stages; one section per technology with its pinned repository |
| [src/](src/) | `cva6_asic_top`, the implementation wrapper (Bender target `cva6_asic`) |
| [constraints/](constraints/) | The timing intent of `cva6_asic_top` in SDC (`cva6.sdc`) and its parameters (`cva6.tcl`) |
| [scripts/](scripts/) | `sram_inventory.py`: the SRAMs of a configuration, elaborated with slang |
| `<tech>/` | The technology's flow, cloned by `make <tech>-init` (ignored by git) |
| `<tech>/build/` | Default run directories, inside the technology's clone |

## Usage

```sh
make -C asic deps          # submodule and Bender dependencies
make -C asic gf22-init     # the GF22 flow at its pinned commit (needs access)
make -C asic synth CLK_PERIOD=1000
make -C asic pnr
make -C asic sim ASIC_SIM_NETLIST=<run>/out/cva6_asic_top.v ASIC_SIM_BINARY=<program>.hex
make -C asic spef
make -C asic power POWER_SAIF=<sim dir>/saif/phase_1.saif
```

`CVA6_CONFIG` selects the configuration (a Bender target of cva6), `CLK_PERIOD`
the clock period in ps; the comments in the Makefile describe every knob, and
each run records its resolved parameters in `params.json` of its run directory.

`sim` runs a program of test/'s software build in test/'s testbench, with
`cva6_asic_top` as the core (test/'s `ASIC_TOP=1`), on QuestaSim: its RTL, or
with `ASIC_SIM_NETLIST` a netlist of it, with the technology's simulation models,
without delays or with the typical delays of an SDF (`ASIC_SIM_SDF`, which place
and route writes beside its netlist). The clock runs at the netlist's implemented
period, or at `ASIC_SIM_CLK_PERIOD`. The wrapper and the testbench build the same
configuration, with test/'s memory attributes (`test/hw/cva6tb_cfg_pkg.sv`), so a
netlist runs test/'s programs.
`ASIC_SIM_SAIF=1` records the switching activity of each workload phase a
program marks (the benchmarks mark their measured region; see
`test/sw/bench/README.md`) into `saif/` of the simulation directory.

`power` reports the power of a place-and-route run at the technology's typical
corner, per hierarchy: with the tools' default activity, and with the activity
of each SAIF `sim` recorded on the run's netlist (`POWER_SAIF`), with or without
its SDF. It needs the run's parasitics (`spef`).
