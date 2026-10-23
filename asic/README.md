# CVA6 ASIC implementation

The design side of CVA6's ASIC flow: synthesis and place and route of the core
for power, performance and area evaluation. Each technology's flow lives in a
repository of its own, which is not open; `asic/Makefile` clones it into
`asic/<tech>` at a pinned commit and includes its `<tech>.mk`.

## Directory layout

| Path | Content |
| --- | --- |
| [Makefile](Makefile) | Knobs, SRAM inventory, run records and the stages; one section per technology with its pinned repository |
| [src/](src/) | `synth_top`, the implementation wrapper (Bender target `cva6_asic`) |
| [constraints/](constraints/) | The timing intent of `synth_top` in SDC (`cva6.sdc`) and its parameters (`cva6.tcl`) |
| [scripts/](scripts/) | `sram_inventory.py`: the SRAMs of a configuration, elaborated with slang |
| `<tech>/` | The technology's flow, cloned by `make <tech>-init` (ignored by git) |
| `build/` | Default run directories (ignored by git) |

## Usage

```sh
make -C asic deps          # submodule and Bender dependencies
make -C asic gf22-init     # the GF22 flow at its pinned commit (needs access)
make -C asic synth CLK_PERIOD=1000
make -C asic pnr
```

`CVA6_CONFIG` selects the configuration (a Bender target of cva6), `CLK_PERIOD`
the clock period in ps; the comments in the Makefile describe every knob, and
each run records its resolved parameters in `params.json` of its run directory.
