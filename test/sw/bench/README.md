# Benchmarks

Benchmarks built to run on the standalone testbench and to report how many cycles and instructions
their timed region took:

| Name | What | Stresses |
| --- | --- | --- |
| `coremark` | CoreMark | the pipeline; fits in the caches |
| `em-<name>` | the 22 programs of Embench-IoT | the pipeline; fits in the caches, except `nsichneu`'s code |
| `rs-<pattern>` | RaiderSTREAM's four kernels for one access pattern | the data cache and the memory: misses, misses in flight, write-backs |

```sh
make -C sw bench
make verilator-build CVA6_CONFIG=cv64a6_splus THREADS=1
./util/run-bench.py run --tag cva6s+
```

`make -C sw bench` clones the sources into `sw/deps/` at pinned revisions and writes one
`<name>.elf`, `.hex` and `.dump` per benchmark to `sw/out/bench/`; `bench-<suite>` builds one suite
(`coremark`, `embench`, `raiderstream`). `sw/out/bench/build-info.txt`
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
which returns 0 even when its checksums mismatch, and `[RS] results verified` for RaiderSTREAM. Any trap ends the run through
the EOC register with return code `0x100 + mcause`, after printing `mcause`, `mepc` and `mtval`.

## Memory benchmarks

The other suites fit in the caches: their data cache misses are close to none. RaiderSTREAM
exercises the data side, and the testbench memory behind it.

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

**The testbench memory** answers every request after `+MemDelay` cycles (default 10, about 38
cycles per load miss on `cv64a6_splus`) and moves one 64-bit beat per cycle, without bank or row
effects or a bandwidth limit of its own. RaiderSTREAM therefore measures how the core and its
caches cope with a fixed latency (how many misses they keep in flight, how much they wait), not
a realistic DRAM bandwidth.

## Running

[../../util/run-bench.py](../../util/run-bench.py) runs every image in `sw/out/bench` on a built
model in parallel, one single-threaded simulation per job, so build the model with `THREADS=1`.
It writes a log per benchmark plus `results.json` and `results.csv` to `--out` (default
`verilator/bench/<tag or config>`, which `make verilator-clean` does not touch), and prints the
table, the geometric mean IPC overall and per suite, and CoreMark/MHz.
`results.json` also records the model's configuration, the RTL commit (and whether the tree
was dirty) and the software build info.

```sh
./util/run-bench.py run --sim <model>/Vtest --out <dir> --tag <label> [-j N] [--filter em-crc32 coremark]
```

The model under `verilator/build` is replaced on every rebuild; copy `Vtest` together with
`cva6tb.flist` (the runner reads the configuration from it) to keep one, e.g. under
`verilator/models/<cfg>/`.

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
  of its work. With the flag, both passes run the same code. RaiderSTREAM, which has no warm-up
  pass, is compiled without it.
- **The compiler**: GCC 14.2 here; his version is not documented.

| Suite | Flags |
| --- | --- |
| Embench-IoT | `-march=rv64gc_zifencei -mabi=lp64d -mcmodel=medany -O3 -g -fdata-sections -ffunction-sections -fno-tree-loop-distribute-patterns -funroll-all-loops -falign-jumps=4 -falign-functions=16 -fno-ipa-cp-clone` |
| CoreMark | `-march=rv64imafdc_zicsr_zifencei -mabi=lp64d -mcmodel=medany -O3 -fno-builtin-printf`, 15 iterations, performance-run seeds |
| RaiderSTREAM | Embench's |

Every compilation also takes `-ffile-prefix-map=<sw>=.`, which does not change the code. Embench's
support code asserts, which puts its file name in the data of five programs; with the map, the
binaries are the same from any checkout, and so are the cycles.

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
| [raiderstream/](raiderstream/) | RaiderSTREAM driver, and an empty `omp.h` for upstream's kernels |

## Known limitations

- The benchmarks are built for RV64GC (`lp64d`), so they run on 64-bit configurations with an
  FPU, which is what the testbench is run on. A 32-bit or FPU-less configuration would need builds
  of its own.
- Some Embench regions are tiny at `CPU_MHZ=1` (`nbody` retires under 500 instructions), so
  their IPC mostly reflects cold-start effects rather than steady-state performance.
- A `cv64a6_splus` model simulates at roughly 25k cycles/s per job, so a full run takes about 8
  minutes on 8 cores. The longest benchmark, `rs-sg`, takes some 11 million cycles, including what
  runs before its region.
