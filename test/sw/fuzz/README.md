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
  page tables would exercise it far harder.
- **PMP, beyond the one open region.** A U-mode segment needs a PMP entry to reach memory at
  all, so there is one now — NAPOT over everything, RWX — which means PMP is matched on every
  U-mode access but only ever one way. Eight entries are forwarded to the reference and the
  granularity is matched, so generated regions with accesses either side of a boundary, and the
  NAPOT readback, would all compare.
