# Generated stimulus

Random programs, run in lockstep against the reference. This is what co-simulation is *for*:
the generated programs check nothing themselves and always report success, because the
co-simulation is the checker.

A generator therefore only has to produce a **legal program**, and the comparison covers every
architectural effect of every instruction it emits.

```sh
make verilator-build COSIM=1 CVA6_CONFIG=cv64a6_imafdc_sv39_wb
./util/run-fuzz.sh --count 100
```

The programs use that configuration's ISA, B, Zcb and Zicond included, so it is the one to
build; on a configuration without one of them its instructions trap on both sides.

A seed that diverges keeps its assembly, its hex, its ELF and the simulation log under
`verilator/fuzz/<seed>/`, so it replays on its own:

```sh
./util/run-fuzz.sh --seed 1234 --keep
```

| Option | Meaning |
| --- | --- |
| `--count <n>` | How many seeds to run (default 20) |
| `--seed <n>` | Run one specific seed |
| `--insns <n>` | Instructions in the generated body (default 2000) |
| `--jobs <n>` | Parallel simulations (default 8) |
| `--keep` | Keep the artefacts of passing seeds too |
| `--no-fp` | Omit floating point |
| `--no-traps` | Omit deliberate traps, leaving `mtvec` never taken |
| `--no-loops` | Omit counted loops, leaving control flow forward-only |
| `--no-umode` | Omit U-mode segments |
| `--no-smode` | Omit S-mode segments and trap delegation |
| `--no-counters` | Omit counter reads and the `mcounteren`/`scounteren` setup |
| `--no-warl` | Omit the WARL CSR probing |
| `--lrsc` | Emit LR/SC — see below |

## What the generator guarantees

[`util/gen_random_test.py`](../../util/gen_random_test.py) is a few hundred lines and has one
job: make a divergence mean a bug. Everything it does follows from that.

- **Every register and every byte of memory it reads is initialised first.** The reference
  zero-fills its memory and `COSIM=1` forces the testbench to do the same; initialising what a
  program reads keeps both models on values the program chose.
- **Memory accesses stay inside a scratch buffer it owns**, at aligned offsets, so every access
  reaches memory the program owns.
- **Every program terminates**, so a run that ends is a finished run. Control flow is
  forward-only apart from counted loops, whose trip count is written into a register the loop
  body never writes.
- **Instructions with a known, understood divergence stay out of the default mix**, so every
  divergence a run reports is a new one.

Within that it covers the ALU, the word-width variants, multiply and divide including the
defined division-by-zero and overflow corners, Zba/Zbb/Zbs, Zicond, loads and stores of every
width, atomics, taken and not-taken branches, single and double precision arithmetic, the moves
between the register files where NaN boxing shows up, and CSR access.

### Traps

A program takes traps on purpose, which brings `trap_cause`, `trap_tval` and the trap CSRs into
the comparison: `mepc`, `mcause`, `mtval` and `mstatus`'s `MPP`/`MPIE` on entry, and `mstatus`
again on the way out.

Four kinds are emitted. Each traps on *any* conformant implementation, so the reference takes the
same trap and the cause is compared:

| Emitted | Cause | `mtval` |
| --- | --- | --- |
| `ecall` | 11, environment call from M | — |
| `.word 0x00000000` | 2, illegal instruction | the instruction |
| `lh`/`lw`/`ld` one byte off an aligned slot | 4, load address misaligned | the address |
| `sh`/`sw`/`sd` likewise | 6, store address misaligned | the address |

The handler is four instructions — read `mepc`, add 4, write it back, `mret` — and needs no
save area because `x4` is reserved for it. Every deliberate trap is a 32-bit encoding, so the
step is always 4 and the handler needs no `mcause`. A trap the generator did not intend is
stepped over the same way and still compared: the co-simulation is what reports it.

`mtvec` is set in direct mode to a 256-byte aligned handler. That is the alignment CVA6 requires
of a *vectored* base, so `mtvec` holds the same value in both models whichever mode is selected.

The trap kinds are the ones whose outcome the architecture fixes. `ebreak` is not one of them:
with debug support compiled in, whether it traps or enters debug mode depends on `dcsr.ebreakm`.

### U mode

Stretches of the body run in U mode. What U mode adds is **permission**, which is where an
implementation can be wrong:

- a CSR access that is legal in M is an illegal instruction in U, which is what
  every `csr` instruction inside a U-mode segment becomes;
- a load or store only works because the preamble opens a PMP region for it, so
  the PMP match is in the comparison for the length of the segment;
- a counter read needs **both** `mcounteren` and `scounteren` — see below.

Entry is `mret` with `mstatus.MPP` cleared; the way back is `ecall`, which the
handler recognises as cause 8 and returns from to M mode. Every other cause
returns to the mode it came from, so a fault inside a segment leaves the segment
running. Both models take the same path, so the mode is compared on every
retirement in between, and the co-simulation's report counts **retirements by
mode**, which shows how much of a run left M.

### S mode and delegation

S-mode segments work the same way, with `mstatus.MPP` set to 1, and they add
**delegation**. `medeleg` decides whether a trap taken in S or U goes to the S
handler through `stvec` or to the M handler through `mtvec`, so one program
exercises both — `sepc`, `scause`, `stval` and `sret` on one path, `mepc`,
`mcause`, `mtval` and `mret` on the other, with the cause deciding which.

Causes 2, 4 and 6 are the ones delegated:

- **Ecalls (8, 9) stay with M.** An `ecall` is how a segment asks to come back
  to M, and the M handler is what ends a segment.
- **2, 4 and 6 are delegatable in both models.** Which causes `medeleg` permits
  delegating is implementation defined; the two models differ on bits 0, 9 and
  19, which the co-simulation's
  [`deleg`](../../hw/cosim/README.md#named-rule-sets) rule set covers. The
  delegation the generator sets up needs no rule.

A counter read is gated differently in each mode, which is the point of having
both: `mcounteren` alone governs S, while U needs `mcounteren` **and**
`scounteren`. The same instruction in the same program therefore takes a
different decision depending on which segment it landed in.

### Counter reads, and both enables

`mcounteren` gates S and U; `scounteren` gates U on top of it. So a U-mode
counter read requires a set bit in *both*, and the preamble sets the two to
**independent** random masks precisely so the `mcounteren`-set,
`scounteren`-clear case occurs — the combination a wrong implementation lets
through, and one that cannot be reached from M mode at all.

The two models count at different rates, so the co-simulation's built-in
`counter-*` rules copy the DUT's value into the reference before it steps. What
is compared is whether the access was permitted.

The counter pool leaves out `time`. Zicntr requires the CSR, and the
architecture lets the execution environment provide it: CVA6 does, with no
`CSR_TIME` case in `csr_regfile.sv`, so `rdtime` is an illegal instruction in
every mode, M included, and this platform puts `mtime` in the CLINT at
`0x0204_bff8`. A reference built with Zicntr implements the CSR. Both are
conformant; see
[the co-simulation README](../../hw/cosim/README.md#the-time-csr-which-the-hart-need-not-provide).

### WARL CSR probing

The generator writes a pattern to a CSR, reads it back, and restores the
original value. The readback is what is compared — through the destination
register and through the CSR snapshot both — so a field that masks differently
in the two models shows up here and nowhere else in a generated program. CSR
legalisation is a large share of what a core can get wrong, and the probing puts
it under the comparison on every seed.

| Probed | What it exercises |
| --- | --- |
| `pmpaddr0`–`7`, `pmpcfg0` | granularity and `A`-field WARL |
| `mtvec`, `stvec` | mode field and vector base alignment |
| `mcounteren`, `scounteren`, `mcountinhibit` | which counters exist |
| `medeleg`, `mideleg` | which causes may be delegated |
| `mepc`, `sepc`, `mtval`, `stval` | low-bit masking |
| `menvcfg`, `senvcfg` | `FIOM`, and `CBIE`/`CBCFE` with Zicbom |
| `satp` | the `MODE` field, where CVA6 takes bare and Sv39 only |
| `frm`, `fcsr` | field widths |

Not probed, each for a stated reason: `mstatus`, because `MBE`/`SBE`/`UBE` are a
legal divergence the probing would meet on every seed; `mcause`/`scause`, which
CVA6 narrows and a reference stores whole; `mie`/`mip`, where arrival is
structural and enabling an interrupt would reach a handler these programs do not
have; and `misa`, hardwired on CVA6 and writable on the reference.

The original value is saved first and restored after, so the program stays
legal — a random `mtvec` would send the next trap into the weeds, a random
`satp` would turn on translation with no page tables, and a `pmpcfg` with its
lock bits set could not be put back at all. The window between write and
restore is two M-mode CSR accesses, neither of which traps, so nothing can be
taken inside it. M mode only: from U or S the first access would trap on
permission before reaching the WARL behaviour, which is a different test.

What the probing reaches between the two models is legal, implementation-defined
divergence, and the rule sets `run-fuzz.sh` enables by default cover it
(`misa-ro`, `lrsc`, `ids`, `deleg`, `tvec`, `endianness`, `hpm`). Three cases are
handled specifically:

- **`pmpcfg` with `A = NA4`.** With a granularity of eight bytes NA4 is not
  selectable. Both models reject it and *resolve it differently*: CVA6 keeps the
  entry's previous `A`, a reference substitutes NAPOT. The generator writes only
  selectable encodings, so the address mode of every entry stays compared.
- **The number of `hpmcounter`s.** CVA6 implements six; a reference built with
  Zihpm implements all twenty-nine and has no knob to say otherwise, so the
  enable and inhibit bits above the implemented count read back set on one side
  and clear on the other. The
  [`hpm`](../../hw/cosim/README.md#named-rule-sets) rule set covers this.
  `MHPMCounterNum` is a hardcoded `ariane_pkg` localparam, outside `cva6_cfg_t`,
  so the ISA-string table does not carry it.
- **The cache-block operation enables.** With Zicbom, CVA6 leaves reset with
  `CBIE` and `CBCFE` set in `menvcfg` and `senvcfg`; their reset value is
  unspecified. The co-simulation starts the reference from the same values,
  taken from the configuration, so the probes compare both registers, the
  first read included. `CBIE` also reserves the encoding `10`, which the two
  models resolve differently: CVA6 keeps the field's previous value, a
  reference writes `00`. The generator writes only the three legal encodings,
  so the field stays compared.

### Loops

Counted backward branches, two to eight trips over a body of two to six instructions. This is
the only thing here that runs the same instruction twice, which is the point: forwarding, the
branch predictor and the non-blocking cache all behave differently on the second pass, and a
mispredicted backward branch is a different case from a mispredicted forward one.

The trip counter is locked for the length of the body, so no instruction inside can write it
and turn a bounded loop into an unbounded one. A loop body draws from the arithmetic, memory,
FP and CSR kinds only — no nested branch, loop or trap.

## Kept out of the default mix

- **LR/SC** is opt-in (`--lrsc`). Whether a reservation survives is implementation defined, so
  the two models can disagree about whether an `sc` succeeded without either being wrong — and
  once they disagree, the memory they wrote disagrees too. `--lrsc` is for exploring that
  difference.
- **`mcause` and `scause`** are written only by traps. They are WARL, and CVA6 narrows them —
  it keeps the interrupt bit and the low bits of the exception code and drops the rest — where
  a reference model stores what it was given. Both are correct. The random CSR writes use
  `mscratch` and `sscratch`, which are plain read/write, and the WARL probing covers the other
  registers with save and restore.

## Where this goes next

The obvious extensions, roughly in order of value:

- **`mstatus.MPRV`.** The remaining permission modifier that needs no paging: it makes an M-mode
  load or store use `MPP`'s privilege for the access, so the PMP check applies as if the
  instruction were in U or S while the instruction itself still runs in M. `SUM` and `MXR` only
  mean something once translation is on.
- **Nested modes.** A U-mode segment inside an S-mode one, entered with `sret`, which is how real
  software gets to U and is the one path `sstatus.SPP` decides.
- **Traps inside segments on purpose.** A segment's interior currently traps only where the
  permission checks make it — a privileged CSR or a gated counter. A deliberate misaligned
  access inside a delegated segment would put `stval` under a comparison it does not get today.
- **Paging.** The co-simulation already handles Sv39 and the ISA corpus exercises it; generated
  page tables would exercise it far harder. Translation-aware generation is the expensive part;
  see *Privilege switching* under FORCE-RISCV below.
- **PMP, beyond the one open region.** A U-mode segment needs a PMP entry to reach memory at
  all, so there is one now — NAPOT over everything, RWX — which means PMP is matched on every
  U-mode access but only ever one way. Eight entries are forwarded to the reference and the
  granularity is matched, so generated regions with accesses either side of a boundary, and the
  NAPOT readback, would all compare.

## FORCE-RISCV

[FORCE-RISCV](https://github.com/openhwgroup/force-riscv) is an instruction *sequence generator*:
it models the ISA, so it can build programs with paging, privilege switching and non-trivial
exception handlers. It is Apache 2.0, needs no UVM, and emits plain ELF. The checking is the
same: its programs report success and the co-simulation decides.

```sh
make -C test force                      # clone, build, and install the cva6tb port
make -C test verilator-build COSIM=1 CVA6_CONFIG=cv64a6_imafdc_sv39_wb
./util/run-force.sh --count 20
```

| Option | Meaning |
| --- | --- |
| `--count <n>` | How many seeds to run (default 5) |
| `--seed <n>` | Run one specific seed |
| `--insns <n>` | Instructions the template asks for (default 2000) |
| `--switches <n>` | Privilege switches to spread through the body (default 0, see below) |
| `--jobs <n>` | Parallel simulations (default 8) |
| `--stop-after <n>` | Retirements to compare before stopping |
| `--keep` | Keep the artefacts of passing seeds too |

The seed is FORCE's own RNG seed, so a seed that diverges replays:
`./util/run-force.sh --seed 37 --keep`.

### How a generated program ends

On a budget. FORCE's end-of-test is a branch to itself, which the framework
expects the simulator to notice, and FORCE's operand solver materialises its own
address into whichever register a store uses, which rules out an EOC write in
the template.

`+cosim_stop_after=<n>` makes the co-simulation stop once it has compared that
many retirements, and a seed passes when the budget was spent with nothing
diverging. That is the right verdict in any case, since the comparison is the
checker and the program asserts nothing.

### The port

[`force/`](force/) is five files, three of them because FORCE keeps this configuration in Python:

| File | What it says |
| --- | --- |
| [riscv-cva6tb.config](force/riscv-cva6tb.config) | the framework configuration, derived from the current `riscv_rv64.config` |
| [memory-cva6tb.py](force/memory-cva6tb.py) | physical memory: DRAM only, and only the 256 MiB that exists |
| [PcConfig.py](force/PcConfig.py) | where reset, boot and the test itself are placed |
| [register_field_choices-cva6tb.xml](force/register_field_choices-cva6tb.xml) | register fields this core constrains |
| [cva6tb_random_force.py](force/cva6tb_random_force.py) | the test template: which instructions, and how much privilege switching |

What the port sets:

- **Memory is DRAM only**, the 256 MiB this testbench has; anything above `0x9000_0000` reads
  back as an AXI error. The configuration is derived from `riscv_rv64.config`, the one the
  current framework accepts.
- **The reset PC is in DRAM**, above the boot PC. The testbench decides which memory to preload
  from the *first* address in the hex image, so the whole image lies in DRAM. Nothing executes at
  the reset PC: the boot ROM jumps to `0x8000_0000`, which is where FORCE puts its boot code.
- **Paging is Sv39**, the mode CVA6 implements; `riscv_rv64.config` selects Sv48.
- **`mstatus`'s reserved span is pinned to zero**, which is what software is supposed to write
  there; see *mstatus.MBE* below.
- **The program ends on a budget** — see *How a generated program ends* above.

### Building it

`make force` uses the **host** compiler, the one exception in this testbench: FORCE's vendored
pybind11 predates GCC 13's stricter `<cstdint>` includes and needs an older GCC. FORCE also
embeds CPython and builds against its development headers; 3.6 is the version installed here with
headers, and the one FORCE's own README uses.

FORCE therefore runs on the host too, and so does `./util/run-force.sh`: it generates each program
there and starts the simulation in `$SIM_ENV`, the environment the model is built in, when that
is set (see [util/iis-env.sh](../../util/iis-env.sh)). `./util/run-fuzz.sh` runs like the other
runners, inside the environment the model is built in.

### mstatus.MBE

FORCE randomises `mstatus`'s WPRI span and its boot code writes the result. On this core bit 37
of that span is `mstatus.MBE`, and CVA6 *implements* it — `mbe` reaches the load and store units,
which byte-reverse per access size (`store_unit.sv`, `load_unit.sv`). The reference is built
little-endian and takes that from `cfg_t`. Both are conformant: MBE is WARL and an implementation
may provide one endianness or both.

Once the bit is set, the two models execute every load and store differently, which a masked CSR
rule cannot reconcile (see
[What a masked CSR rule cannot do](../../hw/cosim/README.md#what-a-masked-csr-rule-cannot-do)).
The port pins the span to zero, so both models run the same program with nothing to change in
either.

### Privilege switching

**Privilege switching is wired up and off by default** (`--switches <n>`). The template calls
FORCE's `systemCall` to move between M, S and U, which brings address translation, the `xstatus`
transitions and the handlers FORCE builds into the comparison; every load and store after a
switch is a translated one. With translation on, FORCE generates slowly — the cost is in its
state solving — so switches suit a few deep programs.

| | `gen_random_test.py` | FORCE, M mode | FORCE, with switches |
| --- | --- | --- | --- |
| Generation | fast | fast | slow |
| Reaches | traps, U and S mode, delegation, WARL probing | the M-mode instruction mix | paging, privilege switching, FORCE's handlers |
| Good for | volume | volume | a few deep programs |

`gen_random_test.py` is the volume generator. FORCE's value is the switches, which reach paging,
and it is used for a few deep programs.
