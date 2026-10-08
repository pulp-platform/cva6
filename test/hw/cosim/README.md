# CVA6 co-simulation

Runs [spike](https://github.com/riscv-software-src/riscv-isa-sim) in lockstep with CVA6, one
reference instruction per RVFI retirement, and compares every architectural effect as it
happens.

```
*** [cva6tb_cosim] MISMATCH on rd x14 at retirement 1191 (cycle 7013, port 0)
      dut  pc 0xffffffffffe0239c  insn 0x0007b783  ld  a5, 0(a5)
      ref  expected 0x0000000000000000, dut had 0x7998be4943b6da9d
```

It reports a failure **at the instruction** that diverged, and it needs no expected answer, so
it checks any program.

## How it is put together

| File | Role |
| --- | --- |
| [cva6tb_cosim.sv](cva6tb_cosim.sv) | Samples RVFI at commit and calls the DPI backend. No strings, classes or queues. |
| [cva6tb_cosim_dpi.h](cva6tb_cosim_dpi.h) | The DPI-C interface between the two halves. |
| [cva6tb_cosim.cc](cva6tb_cosim.cc) | The DPI entry points and the handle table; kept thin. |
| [rv_cosim.h](rv_cosim.h) / [rv_cosim.cc](rv_cosim.cc) | The lockstep loop and every comparison. |
| [rv_ref.h](rv_ref.h) | What the loop needs from a reference model, and nothing more. |
| [rv_ref_spike.h](rv_ref_spike.h) / [rv_ref_spike.cc](rv_ref_spike.cc) | Spike behind that interface. |
| [rv_memif.h](rv_memif.h) / [rv_memif.cc](rv_memif.cc) | The memory the reference sees: spike's `simif_t` over the testbench address map. |
| [rv_policy.h](rv_policy.h) / [rv_policy.cc](rv_policy.cc) | The divergence rules and the coverage report. |
| [rv_cosim_standalone.cc](rv_cosim_standalone.cc) | The reference with no RTL at all. |

The decoder, symbol table and triggers come from [`../common/`](../common/), shared with the
tracer. Only this directory links against spike.

## Using it

```sh
make -C test spike                  # once: builds spike into test/tools/spike
make verilator-build COSIM=1 CVA6_CONFIG=cv64a6_imafdc_sv39_wb
make verilator-run   COSIM=1 CVA6_CONFIG=cv64a6_imafdc_sv39_wb HEXFILE=sw/out/hello.hex MAX_CYCLES=1000000
```

`MAX_CYCLES` defaults to 10 000, which suits `hello`; raise it for anything longer.

Co-simulation runs on Verilator only; `make` stops when `COSIM=1` is combined with the
QuestaSim targets. [cosim.mk](cosim.mk) holds the build. Spike and the standalone reference
are built in `$SPIKE_ENV`, a command prefix that [util/iis-env.sh](../../util/iis-env.sh) sets
to the environment Verilator runs in. Any other environment works if its C++ runtime is
compatible with the model's: the same libstdc++ ABI, and a libstdc++ and glibc where the model
runs at least as new as the ones spike was built against. A mismatch stops the model's link,
or its start, with undefined references or a missing `GLIBCXX`/`GLIBC` version.

`COSIM=1` co-simulates the configuration `CVA6_CONFIG` names, the testbench default included.
Spike models neither CLIC nor this testbench's peripherals, so `cv64a6_imafdc_sv39_wb`, the
smallest 64-bit configuration, is the one to bring a change up on. `COSIM=1` also forces
`ZERO_SIM_MEM`, so memory that was never written reads as zero on both sides, as the
reference's memory does.

A mismatch stops the simulation at the instruction that diverged, which also fails the run for
runners that judge by the log: [`util/run-riscv-tests.sh`](../../util/run-riscv-tests.sh) greps
for `SIMULATION PASSED`.

| Plusarg | Default | Meaning |
| --- | --- | --- |
| `+nocosim` | — | Disable co-simulation in a model that has it compiled in |
| `+cosim_image=<path>` | `+binary` | The hex image; defaults to the one the testbench loaded |
| `+cosim_bootrom=<path>` | — | Raw boot ROM image, passed automatically by the Makefile |
| `+cosim_rules=<a>,<b>` | none | Enable named rule sets |
| `+cosim_rule=<spec>` | — | Ad-hoc rules, see below |
| `+cosim_spike_log=<path>` | `/dev/null` | Write spike's own commit log here |
| `+cosim_max_reports=<n>` | `10` | Stop printing mismatches after this many |
| `+cosim_vaddr_bits=<n>` | from config | 39 for Sv39, 32 for Sv32, 0 for bare |
| `+cosim_inject_fault=<n>` | — | Corrupt retirement *n*, to prove the checker fires |

### Proving it is checking anything

`+cosim_inject_fault=` corrupts one retirement, so a run shows that the checker fires:

```sh
make verilator-run COSIM=1 CVA6_CONFIG=cv64a6_imafdc_sv39_wb HEXFILE=sw/out/hello.hex \
  SIM_ARGS="+cosim_inject_fault=60"
```

```
*** [cva6tb_cosim] MISMATCH on pc at retirement 60 (cycle 652, port 0)
%Fatal: [cva6tb_cosim] reference and DUT diverged
```

The end of run report counts the comparisons made of each field, so a clean run also shows what
it compared.

```
*** [cva6tb_cosim] INFO: comparisons made, by field:
***   pc                  125
***   insn                125
***   rd                  106
***   mem_addr             14
***   priv                125
```

## What is compared

`pc`, `insn`, `priv`, `rd`, `mem_addr`, `mem_wdata`, `trap_cause`, `trap_tval`, and the
architectural CSRs. Default is **strict**: a field with no matching rule is compared exactly.

The CSR comparison covers the trap registers, which checks trap *semantics* beyond the cause,
together with the delegation, protection and translation
registers: `mtvec`, `mie`, `mscratch`, `mcountinhibit`, `pmpcfg0-3`, `mcounteren`, and under
S-mode `stvec`, `sie`, `sscratch`, `scounteren`, `satp`, `medeleg` and `mideleg`. A WARL field
that accepts bits it should not surfaces here.

The snapshot reports a register when it changes, so the comparison covers the registers whose
changes both models derive from the program. Three are outside it:

| Excluded | Why |
| --- | --- |
| `mip` | raised by the interrupt lines, and in the reference by this co-simulation itself, at an arrival time that legitimately differs. |
| `mcycle`, `minstret` and their unprivileged aliases | advance at each model's own rate; the counter rules copy their reads into the reference. |
| `fflags` | accrued by floating point execution, so a change can land a cycle either side of the retirement that caused it. |

Two further limits are worth knowing. The comparison is of what *landed in the register*, so a
read-path bug -- a readback mux that masks bits the register holds -- shows up in the `rd`
comparison. And a write whose side effects reach another register (`fcsr` into
`fflags` and `frm`, `mstatus` into `sstatus`) is only checked on the one that changed.

The first batch of changes describes the DUT's reset state (CVA6 loads `mtvec` with
`boot_addr + 0x40` on leaving reset, for one). It seeds the reference, so both start from the
same architectural state, and the report counts each seeded register as unchecked. Reset state
RVFI does not report is seeded from the configuration: with Zicbom, CVA6 leaves reset with the
cache-block operation enables (`CBIE`, `CBCFE`) set in `menvcfg`, `senvcfg` and `henvcfg`, where
the reference resets them to zero. A register the reference does not implement -- the CLIC
additions, the CVA6 custom ones -- is counted and skipped, since reading it would raise an
illegal instruction inside the reference.

What RVFI reports shapes what is possible:

- **`mode` is two bits and says S for both HS and VS**, so the probes carry a `virt` bit beside
  it. Privilege and virtualisation describe one fact -- which mode the instruction ran in -- and
  are compared as one, reported as `U`, `HS`, `M`, `VU` or `VS`. The run reports how many
  retirements were virtualised, which shows whether the comparison saw VS at all.
- **`rd_addr` is five bits with no register-file tag**, and CVA6 muxes the FP result onto
  `rd_wdata`, so which file was written comes from decoding the instruction.
- **`order` and `pc_wdata` are never driven**, so the retirement counter is kept here and the
  PC check is "the reference's PC" against "this retirement's `pc_rdata`".
- **`tval` is not in RVFI** — it arrives out of band on `rvfi_to_iti.tval` — and CSR state is
  not either, which is why the trap registers come from `cva6_rvfi`'s separate `rvfi_csr`
  snapshot. Only registers that changed this cycle are reported.
- **An interrupt is not a trap.** RVFI's `trap` bit is gated on the cause's interrupt bit being
  clear (`cva6_rvfi.sv:522`), so it covers synchronous exceptions only. An interrupt is visible
  as `rvfi.intr` on the *first instruction of the handler*, which is where the co-simulation
  reports it — before that instruction, which is where it happened.

A load's value is checked through the register the load writes, which covers everything that
reaches architectural state; spike records a read with its address and length only.

## Divergence rules

Some things legitimately differ. Each such divergence is a rule that carries a **mandatory
reason**, is counted, and appears in the end of run report as the coverage hole it is.

A rule is for a divergence between two *conformant* implementations -- an implementation-defined
choice, a WARL field with more than one legal value -- so it reads as a statement about the
architecture. A few rules about this core specifically are fine
where they are understood and written down (`fence-t` is one), but they are the exception.

### What takes a rule

Three kinds of divergence, with different bounds, which is why they are handled differently:

| Kind | Bounded by | Handling |
| --- | --- | --- |
| **Structural** -- the reference cannot model it at all: the testbench peripherals, interrupt arrival, a cache, a TLB, CLIC | the reference's own architecture, so small and enumerable | built in, always on |
| **Implementation-defined** -- both implementations are right and they differ: `endianness`, `deleg`, `tinst` | what the corpus actually exercises | opt-in, added on demand |
| **Probe** -- software written to explore WARL space, like `sw/tests/csrprobe.S` writing all ones to every CSR | nothing: it diverges wherever the two implementations differ at all | co-simulated, as the cheapest way to find which rules are needed |

**Add a rule for an observed divergence.** The divergences real software provokes are few, and
a rule written for one is exercised by it. A missing rule fails loudly, at the instruction; a rule
is silent by design, so each one is backed by a divergence it was seen to cover.

**Judge a rule by its breadth.** A test in which a rule fires is still
worth running: the firing is counted as a hole and everything else in the test is still compared
exactly. What costs something is the rule itself, and rules are global -- a rule written to get
one test past one divergence applies to every program afterwards. A `masked` rule over six bits
of one CSR, or a `copy` of one register's read, is close to free. "Supply every load's data" or
"skip the `pc` check" gives up comparing loads, or control flow, for everything.

A test is therefore left failing when only a broad rule would get past it, as for a load that
depends on a stale TLB entry: `hfence_tests.c` stops the run there. A WARL field takes a rule a
few bits wide.

**The probes are worth co-simulating for the same reason.** `csrprobe.S` writes all ones to every
CSR, so it diverges early and often -- which makes it the cheapest way to discover which narrow
rules a configuration needs. A short probe run predicts the rule sets a whole corpus will ask
for.

**A `masked` CSR rule needs a `copy` on its read.** Masking relaxes the comparison but leaves the
two models holding different values; the next read of that register carries the difference into a
general register, where nothing diverges visibly until some later instruction and no rule can
name it. `endianness` and `deleg` each pair the two for this reason.

**Prune rules that stop firing.** The report lists rules that did not fire. One that has gone
quiet across the corpus is either obsolete or was never real, and removing it costs nothing and
restores strictness.

**Watch the ratio of comparisons to firings.** The report gives comparisons per field beside
firings per rule; what matters is how much the rules suppress.

```
*** [cva6tb_cosim] INFO: coverage holes, by rule:
***   mmio          mem_rdata  supply    1  the testbench peripherals are not modelled by the reference
***   misa-ro-read  rd         copy      3  CVA6-hardwires-misa-so-the-read-is-taken-from-the-DUT
```

A rule names a **field**, an **action**, and the scope it applies in:

| Action | Meaning |
| --- | --- |
| `compare` | the default; restores strictness inside a narrower scope |
| `masked` | compare only the bits in `mask` |
| `copy` | take the DUT's value into the reference, so the difference stops here |
| `supply` | hand the DUT's value to the reference as an input; do not compare |
| `hold` | the DUT hardwires this and ignores writes; restore the reference's value |
| `skip` | do not execute on the reference; apply the DUT's effects to it |
| `inject` | force state the DUT reached by means the reference does not have |

**`copy` and `supply` differ.** Supplying a value only declines to compare it: the reference
keeps its own, and everything computed from it diverges one instruction later. Copying takes the
DUT's value, which stops a known difference at the instruction that produced it. The counter rules
copy for the same reason: a counter read feeds later instructions, branches included, so both
models continue from the same value.

Scope is any of `csr=`, `addr_lo=`/`addr_hi=`, `scope=ram|mmio`, `priv_mask=`, and an
`insn_mask=`/`insn_match=` pair for a rule about one class of instruction. `budget=` states how
often a rule was expected to fire; exceeding it fails the run at the instruction that exceeded
it.

```sh
SIM_ARGS="+cosim_rule=name=my-rule,field=rd,csr=0x301,action=copy,why=hardwired-in-cva6"
```

Several rules are separated by `;`. A misspelled field, an unknown action or a missing reason
stops the simulation at startup, so every rule given is one that can match.

### Named rule sets

None is enabled by default. Every hole is opted into by name, so a clean run states exactly what
it left unchecked.

| Set | What it covers |
| --- | --- |
| `misa-ro` | CVA6 hardwires `misa` and ignores writes; spike implements `misa.C` as writable |
| `lrsc` | a failing store-conditional writes no memory in the reference |
| `ids` | `mvendorid`/`marchid`/`mimpid` name the implementation, so they differ by definition |
| `deleg` | which causes `medeleg` permits delegating is implementation defined: bits 0, 9 and 19 differ |
| `endianness` | `mstatus.MBE`/`SBE`/`UBE` are WARL; an implementation may provide one endianness or both |
| `tinst` | the reference throws memory faults with `tinst` and `tval2` zero, so `mtinst`, `mtval2`, `htinst` and `htval` carry nothing |
| `irq` | interrupt entry comes from the DUT, for running software that takes interrupts; delivery itself is then unchecked |
| `clic` | spike has no CLIC at all: its registers, its `xcause` format and its vectored entry |
| `hpm` | How many `hpmcounter`s a hart implements is implementation defined — the architecture lets `hpmcounter3`–`31` be hardwired to zero, and a hart that hardwires one hardwires its enable and inhibit bits too. CVA6 implements six; a reference built with Zihpm implements all twenty-nine and has no knob to say otherwise. Masks the hpm bits of `mcountinhibit`, `mcounteren` and `scounteren`, keeping `CY`, `TM` and `IR` compared, with a `copy` on each read. |
| `cache-alias` | a cacheless reference cannot model a dirty line behind the uncached alias |

```sh
make verilator-run COSIM=1 CVA6_CONFIG=cv64a6_imafdc_sv39_wb \
  SIM_ARGS="+cosim_rules=misa-ro,lrsc,ids,deleg" \
  HEXFILE=sw/deps/riscv-tests/build/isa/rv64ui-p-add.hex MAX_CYCLES=2000000
```

`clic` is the widest of them, and worth spelling out because it is the one that gives up most.
Spike has no CLIC, so the registers CLIC adds — `mtvt`, `mintstatus`, `mintthresh` and the
S-mode and VS-mode counterparts — do not exist there: an access to one is an illegal instruction
on the reference and an ordinary instruction on the DUT, so those are **stepped over** with the
DUT's result put into the reference. CLIC also redefines `xcause` to pack the previous privilege,
interrupt-enable and interrupt level alongside the exception code, and it vectors trap entry
through a table the reference knows nothing about, so the entry PC and the trap CSRs of that one
instruction are taken from the DUT. Everything the driver does *between* those points is still
compared.

The injected entry is scoped to the single retirement that follows the interrupt, so every other
retirement keeps its program counter check.

## The standalone tool

The reference with no RTL and no simulator, which is the cheapest debug loop in the project and
the thing that tells a co-simulation bug from an RTL bug:

```sh
make -C test cosim-standalone
./hw/cosim/rv_cosim_standalone sw/out/hello.hex \
    --bootrom hw/bootrom/cva6tb_bootrom.bin
```

```
image  : sw/out/hello.hex (loaded at 0x0000000080000000, DRAM, boot mode 0)
isa    : rv64gc_zba_zbb_zbs_zcb_zicond / MSU
[CON] Hello

125 instructions, final pc 0x000000000001009e
end of computation, return code 0
```

It models just enough of the peripherals to boot: the boot-mode register, the console and the
EOC register. `--spike-log` writes spike's own commit log, which
[`util/cmp_spike_trace.py`](../../util/cmp_spike_trace.py) will diff against a trace taken with
`+trace_format=spike` — an independent second opinion when a mismatch is ambiguous. Note that
spike emits no commit-log line for an instruction that traps, while CVA6's RVFI reports `ecall`
as a retirement, so the two logs offset by one around a trap without anything being wrong.

## Design notes

**Spike owns its memory.** It is preloaded from the same `.hex` the testbench loads, plus the
boot ROM, and spike's own execution keeps it current. A load whose value the reference computes
from its own copy catches a stale cache line or a bad store-buffer forward. This rests on CVA6
being the only master that can write RAM — see the comment at the top of [rv_memif.cc](rv_memif.cc).

**MMIO data is supplied.** The reference still executes the instruction, so the
effective address, the access size and the sign extension stay checked; only the data comes from
the DUT.

**Interrupts nudge the input, not the outcome.** An interrupt is raised as pending through
`mip` and nowhere else, so the reference decides delegation, vectoring, `mstatus.MPP/MPIE` and
the handler PC from its own state. Only the arrival time, which is cycle dependent and cannot be
derived, comes from the DUT.

**`processor_t` is driven directly**, through `rv_memif`, and the testbench's own hardware
provides the console and the exit. That means replicating what `sim_t` does after construction;
a missing step shows up as a divergence, so each one is listed here:

- `enable_log_commits()` is what populates `log_reg_write`; without it the reference reports no
  register writes at all. It also makes spike print, so it needs a real `FILE*`.
- `set_max_vaddr_bits()` is only ever called from `sim_t`, out of a device tree. Left at its
  default, `satp` masks its MODE field to zero on every write and a program that turns on
  virtual memory runs with it silently off.
- That must be followed by `reset()`: the writable masks of `mstatus` and `sstatus` capture
  `has_page` once, when the CSR objects are built, so `SUM`, `MXR` and `TVM` otherwise stay
  permanently unwritable.

**Privilege comes from `last_inst_priv`.** RVFI's `mode` is the retiring instruction's
privilege, which spike records as `last_inst_priv`; its `prv` after a step is the next
instruction's. The two differ on exactly the instructions that change privilege.

## Describing the DUT to the reference

An extension the reference implements and the DUT does not makes it execute what the DUT traps
on; one the DUT implements and the reference does not makes it trap where the DUT does not.
Either way the divergence surfaces on the retirement *after* the trap and names a program
counter, so the description has to be complete up front.

The mapping is therefore a table over every ISA-bearing field of `config_pkg::cva6_cfg_t`. Each
appears in `isa_bits()` in
[cva6tb_cosim.sv](cva6tb_cosim.sv), mirrored by `rv_isa_bits` in [rv_cosim.h](rv_cosim.h) and
spelled for spike by `rv_isa_string()`:

| `cva6_cfg_t` | spike | Note |
| --- | --- | --- |
| `RVA` `RVC` `RVD` `RVF` `RVH` `RVV` | `a` `c` `d` `f` `h` `v` | M is not a field: CVA6 always has it |
| `RVS` `RVU` | the privilege string | `M`, `MU` or `MSU`, and nothing else is accepted |
| `RVB` | `zba` `zbb` `zbs` `zbc` `zbkc` | `decoder.sv` decodes `clmul`/`clmulh`/`clmulr` under `RVB` |
| `ZKN` | `zkn` | AES, SHA, `pack`, `brev8`, `xperm*` — Zbkb, Zbkx, Zkne, Zknd, Zknh |
| `RVZCB` `RVZCMP` `RVZCMT` | `zcb` `zcmp` `zcmt` | |
| `RVZiCond` | `zicond` | |
| `RVZicntr` `RVZihpm` | `zicntr` `zihpm` | without them the counter CSRs do not exist at all |
| `RVZiCbom` | `zicbom` | the cache block size travels separately, in the dimensions |
| `SvnapotEn` | `svnapot` | without it a PTE's N bit reads as reserved and faults |

Three enabled features have no reference model at all. They are carried in the same mask so that
startup names them:

| `cva6_cfg_t` | Covered by |
| --- | --- |
| `RVSCLIC` `RVXHCLIC` | the [`clic`](#named-rule-sets) rule set |
| `CvxifEn` | nothing — no rules exist for it |
| `XF16` `XF16ALT` `XF8` `XFVec` | nothing — no rules exist for them |

The rest of `cva6_cfg_t` leaves the instruction set unchanged and stays out of the table: the
microarchitectural fields (caches, TLBs,
`SuperscalarEn`, the bypasses, the FPGA and technology options); the dimensions forwarded
separately as `dims` (`NrPMPEntries`, `Sdtrig*`, `DCACHE_LINE_WIDTH`), which spike takes from
its `cfg_t`; the platform fields with no bearing on decode
(`SoftwareInterruptEn`, `PerfCounterEn`, `TvalEn`, `DirectVecOnly`, `PMPEntryReadOnly`,
`PMPNapotEn`); `MmuPresent` and the Sv mode, which arrive as the virtual address width; and
`DebugEn`, whose registers the reference has but this testbench never drives.

Adding an extension field to `cva6_cfg_t` means adding it here. What startup prints — the ISA
string, the privilege string, the dimensions, the reference's own `misa`, and any unmodellable
feature — is the whole description, so it can be read against the configuration at a glance.

## Configurations

The co-simulation supports every configuration the Slang CI elaborates, plus three of its own.
Any of them runs with the commands under [Named rule sets](#named-rule-sets) and `CVA6_CONFIG`
set; `hello` and a cross-section of `riscv-tests` — integer, word, multiply, atomic, compressed,
single and double precision, M-mode and S-mode CSRs, and the virtual-memory variants — with
`misa-ro,lrsc,ids,deleg` exercise each one:

| Configuration | Notes |
| --- | --- |
| `cv64a6_imafdc_sv39_wb` | the smallest 64-bit configuration, and what to bring a change up on |
| `cv64a6_imafdch_sv39_wb` | H; the ISA string gains `h` |
| `cv64a6_splus` | **superscalar**, so both commit ports retire in the same cycle |
| `cv64a6_splusplus` | `cv64a6_splus` without the load pipeline register and with larger branch predictors; the same ISA |
| `cv64a6_imafdchsclic_sv39` | H + Ssclic, write-through cache |
| `cv64a6_imafdchsclic_sv39_wb` | the testbench default configuration |
| `cv64a6_imafdchsclic_sv39_hpdcache_wb` | H + Ssclic on the HPDCache |
| `cv64a6_imafdchxhclic_sv39_wb` | H + Xhclic, the H-mode CLIC variant |
| `cv64a6_imafdchxhclic_sv39_hpdcache_wb` | H + Xhclic on the HPDCache |

The cache type is invisible to the comparison: the co-simulation checks architectural state,
and a cache that is doing its job leaves no trace in it. The [`cache-alias`](#named-rule-sets)
rule exists for the one case where it does.

CLIC needs its own stimulus, since `riscv-tests` never enters CLIC mode. The `sw/tests` CLIC
programs run co-simulated on `cv64a6_imafdchxhclic_sv39_wb`, which `clic_vsmode` needs because
it runs its handler in VS mode.

```sh
make verilator-build COSIM=1 CVA6_CONFIG=cv64a6_imafdchxhclic_sv39_wb
make verilator-run   COSIM=1 CVA6_CONFIG=cv64a6_imafdchxhclic_sv39_wb \
  HEXFILE=sw/out/clic_smode.hex MAX_CYCLES=2000000 \
  SIM_ARGS="+cosim_rules=misa-ro,lrsc,ids,deleg,clic"
```

Under H, RVFI carries the virtualisation bit, so an S-mode CSR access made in VS mode is
compared as one. If an S-mode CSR and its VS counterpart ever alias differently in the two
models, the divergence arrives as a mismatch naming the register.

## Known divergences

`COSIM=1 ./util/run-riscv-tests.sh` runs the ISA corpus co-simulated, with `misa-ro`, `lrsc`,
`ids` and `deleg` enabled. `deleg` is in the runner's default because `rv64si-p-csr`,
`-ma_fetch`, `-sbreak` and `-scall` write `medeleg`. `rv64ui-p-fence_t` runs through the
built-in `fence-t` rule: `fence.t` is a PULP addition spike does not implement, so the rule skips
it, applies the DUT's effects and counts the hole. `rv64mzicbo-p-zero` needs Zicbom, which this
configuration does not enable, so the script's list leaves it out.

### What a masked CSR rule cannot do

A `masked` rule on a CSR relaxes the comparison of that register. That is sound for a field whose
value only *is*, and unsound for a field that changes how everything after it executes.

`mstatus.MBE` is the second kind. CVA6 implements big-endian data accesses — `mbe` reaches the
load and store units, which byte-reverse per access size — while a reference built little-endian
holds the bit at zero. Both are conformant, because the field is WARL. Once a program sets the
bit, the two models disagree about the meaning of every load and store, and a masked rule on
`mstatus` moves the first report from the write to some later load, where the cause is no longer
visible. A field like this is handled in the stimulus, which writes what software is supposed to
write there: zero, in a reserved span.

So before writing a `masked` rule on a CSR, ask what the field controls. If the answer is "the
behaviour of later instructions" — endianness, a paging mode, a delegation that redirects a trap,
`mstatus.MPRV` — the answer lies in the stimulus or in a `copy` that keeps both models in the same
state.

### Rules that select the same thing

`match()` returns the first rule that fits, so of two rules selecting the same register only the
first ever applies, and the other sits in the report having never fired. Two such rules are
therefore a startup error naming both.

`tvec` and `clic` both concern `mtvec`, for different reasons and in different bits:

| | `tvec` | `clic` |
| --- | --- | --- |
| Mode | `01`, vectored; both models agree | `11`, which the base architecture reserves and the reference will never hold |
| Bits that differ | `[7:2]`, the base alignment CVA6 makes stricter than required | `[1:0]` and the low base bits |
| Nature | implementation-defined: both are conformant | a custom feature the reference does not have, like `fence.t` |

Each keeps its own scope, and `tvec` still compares the mode field. A CLIC configuration enables
`clic`; a configuration without CLIC enables `tvec`. Enabling both is the error above.

### Why an interrupt's arrival point cannot be matched

RVFI reports an interrupt as `rvfi.intr` on the *first instruction of the handler*, so the
reference learns of it only after the DUT has already vectored. Raising it pending and stepping
once then puts the reference at the handler entry it decided on, which keeps delegation,
vectoring and `mstatus` compared. The instruction boundary at which the interrupt was taken is
not in RVFI, though, and where a program observes it -- the architectural corpus's `Interrupts*`
suites -- the two models separate.

Bridging that takes `inject`, which hands the entry PC over from the DUT and with it the
vectoring check, most of what there is to check about taking an interrupt. The `irq` rule set
does that for software that takes interrupts; without it, a program that observes the arrival
point diverges there.

### The time CSR, which the hart need not provide

Zicntr requires a `time` CSR, and the architecture lets the execution environment provide it.
CVA6 takes that option: `csr_regfile.sv` has a read case for `CSR_CYCLE` and `CSR_INSTRET` and
none for `CSR_TIME`, so `rdtime` raises an illegal instruction in every mode, M included, and this
platform puts `mtime` in the CLINT at `0x0204_bff8`. A reference built with Zicntr implements the
CSR and answers the read.

Both are conformant. The software run here leaves `time` unread, so no rule covers it. A workload
that reads it needs a rule on `trap_cause` that suppresses the expected trap: the divergence is a
trap the DUT takes and the reference does not, and `skip` and `supply` act on a retirement, which
this instruction is not on the DUT. The ISA-string table in
[Describing the DUT to the reference](#describing-the-dut-to-the-reference) maps a `cva6_cfg_t`
field to a spike extension, so it cannot express "Zicntr without `time`" either.

### What a cacheless reference cannot follow

`sw/tests/zicbom.S` measures whether `cbo.inval` discards dirty data, by storing through the
cached region, invalidating the line and reading the location back. CVA6 discards: the load
returns what memory held before the store. A cacheless reference returns the store.

Both are conformant — Zicbom permits `cbo.inval` to invalidate without writing back, and permits
an implementation to treat it as `cbo.flush` — so this is implementation-defined, and its
consequence is unbounded: after one discarded line the two memories differ for the rest of the
run. `zicbom.S` is a probe: it exists to measure an implementation-defined choice and print it,
its output is the documentation, and it runs natively.

The built-in `cbo-no-access` rule covers the other side of Zicbom. A cache-block operation is
cache maintenance and not an architectural access, but CVA6 issues one through the load/store
unit, so RVFI reports it with an effective address while the reference logs no access at all.
That is structural — true of any conformant reference — so the address of a `cbo.*` is excluded
from the comparison.

### What a reference without a TLB cannot follow

`riscv-hyp-tests` reaches a divergence no rule should paper over. `hfence_tests.c` repoints a
guest page table and then checks that a load *still* returns the old value, because the
translation is cached, before fencing and checking that it changes:

```c
val = hlvd(vaddr);           // the SWITCH1 page, 0x111111111
hpt_switch();                // repoint the guest page table at SWITCH2
cond &= hlvd(vaddr) == val;  // still 0x111111111: the entry is stale by design
hfence_vvma();
cond &= hlvd(vaddr) != val;  // now 0x222222222
```

CVA6 has a TLB and returns the cached value. The reference has none, walks the table again and
returns the new one. Both are right: the architecture permits a translation to be cached and
obliges software to fence, and a model that caches nothing is as conformant as one that does.

A test that *depends* on a stale translation therefore cannot be co-simulated against this
reference. The run stops at that load, and everything before it is compared, VS and VU mode
included.

## Roadmap

`rv_ref` is abstract so a second engine can be added without rewriting the comparison: when
CVA6 and one model disagree, a second model is what says which is wrong.
