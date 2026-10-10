# Benchmarks

Benchmarks built to run on the standalone testbench and to report how many cycles and instructions
their timed region took:

| Name | What | Stresses |
| --- | --- | --- |
| `coremark` | CoreMark | the pipeline; fits in the caches |
| `em-<name>` | the 22 programs of Embench-IoT | the pipeline; fits in the caches, except `nsichneu`'s code |
| `dhrystone` | Dhrystone 2.1, as riscv-tests ships it | nothing the others do not; a DMIPS/MHz reference |
| `rs-<pattern>` | RaiderSTREAM's four kernels for one access pattern | the data cache and the memory: misses, misses in flight, write-backs |
| `chase-<size>` | a pointer chase over a working set of that size | the load-to-use latency of wherever the working set fits |

```sh
make -C sw bench
make verilator-build CVA6_CONFIG=cv64a6_splus THREADS=1
./util/run-bench.py run --tag cva6s+
```

`make -C sw bench` clones the sources into `sw/deps/` at pinned revisions and writes one
`<name>.elf`, `.hex` and `.dump` per benchmark to `sw/out/bench/`; `bench-<suite>` builds one suite
(`coremark`, `embench`, `dhrystone`, `raiderstream`, `chase`). `sw/out/bench/build-info.txt`
records the compiler and the source revisions they were built from; the runner copies it into its
results.

## Measurement

Every benchmark brackets its timed region with `bench_roi_start()` and `bench_roi_stop()`
([include/bench.h](include/bench.h)): Embench's `start_trigger`/`stop_trigger` around the timed
`benchmark()` call, CoreMark's `start_time`/`stop_time` around its iterations. Both read `mcycle`
and `minstret`, and the program prints the deltas as

```
[BENCH] roi cycles=<n> instret=<n>
```

Each hook also retires a marker, `slti x0, x0, 1` at the start and `slti x0, x0, 2` at the stop.
This is a custom-use HINT: a nop on any RISC-V hart, but visible in the RVFI retirement stream,
so a testbench-side observer can measure the same region without relying on the core's own
counters.

A benchmark passes when the testbench reports `**SIMULATION PASSED**` (its `main` returned 0,
which all but CoreMark do only when their results verify), it printed the region's counters, and,
except for Embench, its validation line was printed: `Correct operation validated` for CoreMark,
which returns 0 even when its checksums mismatch, `[DHRY] results verified`,
`[RS] results verified` or `[CHASE] chain verified` for the others. Any trap ends the run through
the EOC register with return code `0x100 + mcause`, after printing `mcause`, `mepc` and `mtval`.

## Memory benchmarks

The other suites fit in the caches: their data cache misses are close to none. These two exercise
the data side, and the testbench memory behind it.

**RaiderSTREAM** ([upstream](https://github.com/michael-beebe/RaiderSTREAM)) extends STREAM with
irregular access patterns. `rs-<pattern>` runs the four kernels of one pattern, in STREAM's order,
all in the measured region, on arrays of `RS_ARRAY_SIZE` doubles (default 32768, 256 KiB each, 8×
the data cache):

| Pattern | Copy | Scale | Add | Triad |
| --- | --- | --- | --- | --- |
| `seq` (STREAM) | `c[j] = a[j]` | `b[j] = s*c[j]` | `c[j] = a[j]+b[j]` | `a[j] = b[j]+s*c[j]` |
| `gather` | `c[j] = a[i1[j]]` | `b[j] = s*c[i1[j]]` | `c[j] = a[i1[j]]+b[i2[j]]` | `a[j] = b[i1[j]]+s*c[i2[j]]` |
| `scatter` | `c[i1[j]] = a[j]` | `b[i1[j]] = s*c[j]` | `c[i1[j]] = a[j]+b[j]` | `a[i1[j]] = b[j]+s*c[j]` |
| `sg` | `c[i1[j]] = a[i2[j]]` | `b[i2[j]] = s*c[i1[j]]` | `c[i1[j]] = a[i2[j]]+b[i3[j]]` | `a[i2[j]] = b[i3[j]]+s*c[i1[j]]` |
| `central` | `c[0] = a[0]` | `b[0] = s*c[0]` | `c[0] = a[0]+b[0]` | `a[0] = b[0]+s*c[0]` |

`RS_PATTERNS` lists the patterns built, by default all five.

The kernels are upstream's OpenMP ones, compiled without OpenMP; [raiderstream/](raiderstream/)
replaces upstream's C++ harness, which needs an OS:
- **Arrays** are placed past the program image (`__base_heap`), so neither the image nor the
  startup's `.bss` clearing grows with them. Like `malloc`ed arrays they are at the same page
  offset.
- **Index arrays** are pseudo-random permutations, like upstream's shuffles, but each element is
  computed from its position by a bijective hash, so building them makes no random accesses.
- **Validation**: upstream's is disabled. Here `a` starts as `j + 1` instead of 1, so that values
  are distinct integers. The sums of the arrays and the relations each kernel establishes, sampled
  every 61 elements, must hold exactly.
- **Per kernel**, a `[RS]` line gives its cycles and the bytes it moves (counted as upstream
  does: values read and written, indices read) per 100 cycles.

`central` touches one element of each array, so it hits in the cache. It is not a pure hit
baseline on CVA6: a load waits for the store buffer to drain when its page offset matches a
pending store's (`core/store_buffer.sv`), and `a[0]`, `b[0]` and `c[0]` share their page offset.

**The pointer chase** ([chase/](chase/)) follows `CHASE_LOADS` (default 16384) pointers through
16-byte nodes linked in one cycle in pseudo-random order, after a warm-up lap. Each load depends
on the previous one, so the cycles per load are the load-to-use latency of wherever the working
set fits. A 16-byte node is the smallest data cache line of any CVA6 configuration, so the
working set occupies `<size>` of the cache whatever the line size. The chase must end at the node
predicted from the number of loads; `[CHASE]` gives the cycles per 100 loads. `CHASE_SIZES`
lists the sizes built, by default `16k`, `64k` and `256k`: within the 32 KiB data cache, twice and
eight times its size. Smaller sets take the cycles of `16k`, larger ones those of `256k`.

**The testbench memory** answers every request after `+MemDelay` cycles (default 10, about 38
cycles per load miss on `cv64a6_splus`) and moves one 64-bit beat per cycle, without bank or row
effects or a bandwidth limit of its own. These benchmarks therefore measure how the core and its
caches cope with a fixed latency (how many misses they keep in flight, how much they wait), not
a realistic DRAM bandwidth.

## Showing a run

[../../util/plot-run.py](../../util/plot-run.py) shows one run: one core configuration, any
number of benchmarks. It runs in the uv environment (see [../../README.md](../../README.md)):

```sh
uv run ./util/plot-run.py verilator/bench/cva6s+ [--out <dir>] [--format png|svg|pdf]
```

It writes, by default to `<run>/plots/`:

| File | Content |
| --- | --- |
| `ipc` | IPC per benchmark, and the geometric mean of each suite |
| `ipc.csv` | IPC, cycles and instructions per benchmark |

## Comparing runs

[../../util/compare-runs.py](../../util/compare-runs.py) compares several runs: core
configurations, the points of a design space exploration, a change against its baseline. Like
`plot-run.py`, it runs in the uv environment. The first run is the baseline unless `--baseline`
names another:

```sh
uv run ./util/compare-runs.py verilator/bench/<a> verilator/bench/<b> [...] [--labels a b ...] [--baseline a] [--out <dir>]
```

It writes, by default to `verilator/bench/compare/`, and prints as tables:

| File | Content |
| --- | --- |
| `speedup` | each run's geometric mean speedup over the baseline |
| `speedup-map` | the speedup per benchmark and run |
| `runs.csv` | per run: configuration, simulation arguments, IPC, speedup |
| `suites.csv` | per suite: each run's geometric mean speedup |
| `benchmarks.csv` | per benchmark: IPC and speedup in every run |

**Speedups** come from the regions' cycles. They cover only benchmarks that pass in both runs and
run the same binary, that is, retire the same number of instructions; the others are listed as not
compared. Besides the geometric mean over all of them, the speedup of each suite is the geometric
mean over its benchmarks (`SUITES` in `benchresults.py`, by name prefix). The suites measure
different things and differ in size, Embench counting 22 programs and CoreMark one: compare runs
by suite before relying on the overall mean.

## Sweeping configurations

[../../util/sweep.py](../../util/sweep.py) runs the benchmarks over variants of a core
configuration, the runs of a design space exploration, and compares them. It runs in the uv
environment and drives the builds, `run-bench.py` and `compare-runs.py` itself:

```sh
uv run ./util/sweep.py sw/bench/sweeps/example.toml [--build-jobs N] [-j N] [--rebuild] [--reuse-runs] [--dry-run]
```

A sweep file ([sweeps/example.toml](sweeps/example.toml)) names a base configuration and the points
to run. Points can be listed explicitly, the first being the baseline, or as a grid of every
combination. A point changes fields of the configuration's `cva6_cfg`, its simulation arguments,
or both:

```toml
name = "example"
config = "cv64a6_splus"
benchmarks = ["coremark", "em-"]          # optional: names containing one of these

[[point]]
label = "baseline"

[[point]]
label = "no-load-pipe-reg"
params = { NrLoadPipeRegs = 0 }           # a SystemVerilog expression, cast like the field's value
sim_args = "+MemDelay=0"

[grid]                                    # every combination, after the points
BHTEntries = [64, 128]
sim_args = ["", "+MemDelay=0"]
```

**Models.** Each distinct set of parameters makes one model. Its configuration package is the
base package with those fields replaced, each checked to be a field of `cva6_cfg`. The model is
built in its own directory, passing that package as `CVA6_CONFIG_PKG`, with several builds in
parallel.

**Reuse.** A model is reused by the points that differ only in simulation arguments, and by later
invocations as long as the package, the build options and the tracked sources are unchanged
(`--rebuild` forces new builds). `--reuse-runs` also keeps the results of points already run.
`--dry-run` shows the points and each model's changed lines.

**Output,** under `verilator/bench/sweeps/<name>/`:
- `models/<parameters>/`: package, build and logs;
- `runs/<point>/`: `run-bench.py`'s results;
- `compare/`: `compare-runs.py`'s results;
- `sweep.json`, which records what each point is.

## Running

[../../util/run-bench.py](../../util/run-bench.py) runs every image in `sw/out/bench` on a built
model in parallel, one single-threaded simulation per job, so build the model with `THREADS=1`.
It writes a log per benchmark plus `results.json` and `results.csv` to `--out` (default
`verilator/bench/<tag or config>`, which `make verilator-clean` does not touch), and prints the
table, the geometric mean IPC overall and per suite, CoreMark/MHz and DMIPS/MHz;
[plot-run.py](#showing-a-run) shows the rest.
`results.json` also records the model's configuration, the RTL commit (and whether the tree
was dirty) and the software build info.

```sh
./util/run-bench.py run --sim <model>/Vtest --out <dir> --tag <label> [-j N] [--filter em-crc32 coremark]
```

The model under `verilator/build` is replaced on every rebuild; copy `Vtest` together with
`cva6tb.flist` (the runner reads the configuration from it) to keep one, e.g. under
`verilator/models/<cfg>/`.

## Checking for performance changes

Each configuration CI checks has its expected results under [expected/](expected/):
`cv64a6_imafdch_sv39_wb`, `cv64a6_imafdchsclic_sv39_wb`, `cv64a6_imafdchsclic_sv39_hpdcache_wb`,
`cv64a6_imafdchxhclic_sv39_wb`, `cv64a6_imafdchxhclic_sv39_hpdcache_wb` and `cv64a6_splus`. They
hold every benchmark's cycles and instructions, as one run measured them, with the compiler, the
source revisions and the RTL commit of that run. `check` runs the benchmarks of a model's expected
results and reports each one's cycles against them, then the geometric mean of the change per
suite and overall:

```sh
make -C sw bench-ci
make verilator-build CVA6_CONFIG=cv64a6_splus THREADS=1
make bench-check                                       # or: ./util/run-bench.py check
```

The expected results are those of the set CI checks, which `make -C sw bench-ci` builds into
`sw/out/bench-ci/`: the benchmarks of `make -C sw bench`, but RaiderSTREAM without `rs-sg` and on
arrays of 16384 doubles instead of 32768 (`BENCH_CI_VARS` in [bench.mk](bench.mk)). Without
these, RaiderSTREAM would take most of a check's time, `rs-sg` alone over 20 minutes on the
standard write-back cache; with them, each array is still four times the data cache.
`make -C sw bench` stays the set to report numbers from.

For example, a `cv64a6_splus` model with `NrLoadPipeRegs` = 0, checked on the quick set:

```
benchmark      expected   cycles   change
-------------  --------  -------  -------
coremark        4653979  4035755  -13.28%
em-aha-mont64   1448049  1447627   -0.03%
em-minver        346307   327810   -5.34%
em-statemate    1175360  1154737   -1.75%
em-ud            955703   881812   -7.73%
em-wikisort      576167   478062  -17.03%
rs-central       754903   689409   -8.68%
---
cycles against the expected results, geometric mean (+ slower, - faster):
  CoreMark       -13.28%  (1)
  Embench        -6.57%  (5)
  RaiderSTREAM   -8.68%  (1)
  all            -7.86%  (7)
**BENCH CHECK FAILED**: 7 of 7 benchmarks changed; if intended, update sw/bench/expected/cv64a6_splus.json with --update (make bench-check-update) and commit it
```

The last line is the verdict:
- **`**BENCH CHECK PASSED**`**: every benchmark took the expected cycles.
- **`**BENCH CHECK FAILED**`**: a benchmark changed, did not pass, or retired other instructions
  than expected (it is not the binary the expected results measured: another compiler or other
  sources). A failing benchmark, or another binary, also fails with `BENCH_CHECK_STRICT=0`.
- **`**BENCH CHECK REPORTED**`**: with `BENCH_CHECK_STRICT=0` (`--report-only`), cycles changed;
  the report is all there is.

A change that moves the numbers on purpose updates the expected results in the same commit:
`make bench-check-update` runs every benchmark under `sw/out/bench-ci/` and rewrites
`expected/<config>.json`, so the diff shows reviewers which numbers moved and by how much.
Simulation is deterministic, so a difference is never noise. Two changes that both move numbers
conflict in the file; after rebasing, the second updates it again.

| Variable | Default | Meaning |
| --- | --- | --- |
| `BENCH_CHECK_SET` | `full` | `full`: every benchmark of the expected results; `quick`: one or more per suite, about 5 minutes with 4 jobs |
| `BENCH_CHECK_STRICT` | `1` | `0` reports changed cycles instead of failing |
| `BENCH_CHECK_JOBS` | all cores | simulations at once |

With 4 jobs, the full set takes about 15 minutes on `cv64a6_splus` and the HPDcache configurations,
and about 20 on those with the standard write-back cache. The cycles do not depend on the Verilator
version: the expected results check the same under 5.042 and 5.046.
In CI it runs alongside the architectural tests, which take about an hour:

```sh
make -C sw bench-ci RV_BINROOT=<GCC 14.2 bin directory>  # the compiler the expected results name
make verilator-build CVA6_CONFIG=<config> THREADS=1
make bench-check
# judge the job by its log: **BENCH CHECK PASSED**
```

## How the binaries are built

The compiler flags follow those of the CVA6S+ frontend evaluation by Philipp Schilk (MSc thesis,
ETH Zurich, 2025, Section 3.2), so that results can be set against his. The source revisions are
the ones he used. The differences:

- **`-mcmodel=medany` instead of `medlow`** for Embench. The testbench DRAM is at
  `0x8000_0000`, which `medlow` cannot address on RV64.
- **The full newlib instead of newlib-nano.** The GCC 14.2 toolchain they are built with
  ships `nano.specs` but not the nano libraries. The full newlib's `memcpy` and `memset` are
  the speed-optimised variants.
- **`zicsr` spelled out in CoreMark's `-march`**, since GCC 12 and later no longer imply it.
- **`-fno-ipa-cp-clone` for Embench.** Without it, GCC compiles each program's `benchmark_body`
  twice: a copy for the warm-up pass and one specialised for the timed pass's repetition count.
  The warm-up then never runs the timed code, which starts cold in the instruction cache, and the
  specialised copy can drop work: `nbody` timed 445 instructions instead of 21535, `ud` a third
  of its work. With the flag, both passes run the same code. RaiderSTREAM and the pointer chase,
  which have no warm-up pass, are compiled without it.
- **The compiler**: GCC 14.2 here; his version is not documented.

| Suite | Flags |
| --- | --- |
| Embench-IoT | `-march=rv64gc_zifencei -mabi=lp64d -mcmodel=medany -O3 -g -fdata-sections -ffunction-sections -fno-tree-loop-distribute-patterns -funroll-all-loops -falign-jumps=4 -falign-functions=16 -fno-ipa-cp-clone` |
| CoreMark | `-march=rv64imafdc_zicsr_zifencei -mabi=lp64d -mcmodel=medany -O3 -fno-builtin-printf`, 15 iterations, performance-run seeds |
| Dhrystone | riscv-tests' flags: `-march=rv64gc_zifencei -mabi=lp64d -mcmodel=medany -std=gnu99 -O2 -ffast-math -fno-common -fno-builtin-printf -fno-tree-loop-distribute-patterns`; inlining disabled in its sources; 500 runs |
| RaiderSTREAM, pointer chase | Embench's |

Every compilation also takes `-ffile-prefix-map=<sw>=.`, which does not change the code. Embench's
support code asserts, which puts its file name in the data of five programs; with the map, the
binaries are the same from any checkout, and so are the cycles.

Dhrystone is riscv-tests' copy (`benchmarks/dhrystone`, from the pinned riscv-tests the other
tests use), compiled as riscv-tests does so that its DMIPS/MHz compares with the figures published
from it. [dhrystone/](dhrystone/) maps riscv-tests' statistics hooks onto the measured region and
checks the final values of the program's globals, which it only states. The runner computes
DMIPS/MHz from the region's cycles and the 1757 Dhrystones per second of 1 DMIPS. Dhrystone mostly
runs `strcpy` and `strcmp`, newlib's here, and its numbers depend heavily on the compiler and its
flags: they compare only with figures built the same way.

Embench runs with `CPU_MHZ=1` and one warm-up pass before the timed one. Its own
`build_all.py` compiles it, against the board description in [embench/](embench/), which the
build copies into the Embench tree. CoreMark is compiled directly, with the port in
[coremark/](coremark/), derived from upstream's `barebones` port. CoreMark defines a second as
100000 cycles to pass its 10-second check; its printed times are therefore meaningless, and
CoreMark/MHz is computed by the runner from cycles.

## Layout

| Path | Content |
| --- | --- |
| [bench.mk](bench.mk) | Build rules, included by `sw/Makefile` |
| [include/bench.h](include/bench.h), [lib/bench.c](lib/bench.c) | ROI hooks, markers, report and trap handler |
| [lib/crt0.S](lib/crt0.S) | Startup: zeroes `.bss`, enables the FPU, keeps the boot ROM's return address |
| [embench/](embench/) | Embench-IoT board description `cva6tb` |
| [coremark/](coremark/) | CoreMark port |
| [dhrystone/](dhrystone/) | Dhrystone: `util.h` stand-in and result check |
| [raiderstream/](raiderstream/) | RaiderSTREAM driver, and an empty `omp.h` for upstream's kernels |
| [chase/](chase/) | Pointer chase |
| [expected/](expected/) | Expected results per configuration, for `check` |

## Known limitations

- The benchmarks are built for RV64GC (`lp64d`), so they run, and `check` covers, 64-bit
  configurations with an FPU, which is what the testbench is run on. A 32-bit or FPU-less
  configuration would need builds of its own, and expected results from them.
- Some Embench regions are tiny at `CPU_MHZ=1` (`nbody` retires under 500 instructions), so
  their IPC mostly reflects cold-start effects rather than steady-state performance.
- A `cv64a6_splus` model simulates at roughly 25k cycles/s per job, so a full run takes about 8
  minutes on 8 cores. The longest benchmark, `rs-sg`, takes some 11 million cycles, including what
  runs before its region.
