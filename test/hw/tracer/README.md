# CVA6 instruction tracer

Writes one line per retired instruction, in the format of the legacy
[`common/local/util/instr_tracer.sv`](../../../common/local/util/instr_tracer.sv), but driven
from the standard **RVFI** interface and usable under Verilator.

```
     915ns      906 M 00000000800001a4 0 e6d70123 sb             a3, -414(a4)          a3  :0000000000000065 a4  :000000001000119e VA: 0000000010001000 PA: 00000010001000
     └─ time    └─ cycle                          └─ disassembly                       └─ written register  └─ source registers   └─ data addresses
                  └─ privilege                                                                               and their values
                    └─ pc         └─ mispredict
                                    └─ raw encoding
```

## How it is put together

| File | Role |
| --- | --- |
| [cva6tb_tracer.sv](cva6tb_tracer.sv) | Samples RVFI at commit and calls the DPI backend. No strings, classes or queues. |
| [cva6tb_tracer_dpi.h](cva6tb_tracer_dpi.h) | The DPI-C interface between the two halves. |
| [cva6tb_tracer.cc](cva6tb_tracer.cc) | Formats and writes the trace; owns the output file. |
| [rv_disasm.h](rv_disasm.h) / [rv_disasm.cc](rv_disasm.cc) | Self-contained RV32/RV64 disassembler. |
| [rv_format.h](rv_format.h) / [rv_format.cc](rv_format.cc) | The output formats and the factory that picks between them. |
| [rv_stats.h](rv_stats.h) / [rv_stats.cc](rv_stats.cc) | Optional end of run summary, written to its own file. |
| [rv_symbols.h](rv_symbols.h) / [rv_symbols.cc](rv_symbols.cc) | Symbol table and its ELF loader. |
| [rv_trigger.h](rv_trigger.h) / [rv_trigger.cc](rv_trigger.cc) | Start/stop conditions and the windows they delimit. |

All decoding, formatting and file I/O live on the C++ side, so the RTL half stays trivial and
the same backend works under Verilator, QuestaSim, VCS and Xcelium. Nothing links against
spike, fesvr or any other external library.

The tracer needs `cva6_rvfi`, which turns the core's `rvfi_probes_o` into RVFI proper. Both are
instantiated in [`cva6tb_soc.sv`](../cva6tb_soc.sv) behind `` `ifdef TARGET_TRACER ``, so a
default build elaborates neither and pays nothing.

## Using it

Tracing is a build-time option, because RVFI generation itself costs simulation time:

```sh
make verilator-build TRACER=1
make verilator-run   TRACER=1 HEXFILE=sw/out/hello.hex
```

The trace lands in `verilator/cva6tb_trace_hart_0.txt`. QuestaSim works the same way
(`make vsim-build TRACER=1`), with the DPI backend compiled by `vlog`.

RV32 configurations work too, with the test software rebuilt for the narrower width:

```sh
make verilator-build CVA6_CONFIG=cv32a6_imac_sv32 TRACER=1
make -C sw clean && make -C sw sw/out/hello.hex \
  SW_FLAGS="-DOT_PLATFORM_RV32 -march=rv32imac_zicsr_zifencei -mabi=ilp32 -mstrict-align -O2 \
            -Wall -Wextra -static -ffunction-sections -ffreestanding -fdata-sections"
```

Runtime behaviour is controlled with plusargs, passed through `SIM_ARGS`:

| Plusarg | Default | Meaning |
| --- | --- | --- |
| `+notrace` | — | Disable tracing in a build that has it compiled in |
| `+trace_file=<path>` | `cva6tb_trace_hart_0.txt` | Output file |
| `+trace_verbose=<n>` | `0` | Detail level, see below |
| `+trace_start=<trigger>` | open from the start | Open the trace window |
| `+trace_stop=<trigger>` | never closes | Close it |
| `+trace_symbols=<elf>[,…]` | off | Annotate with symbol names |
| `+trace_symbol_mode=<mode>` | `header` | `header`, `inline` or `both` |
| `+trace_symbol_start=<trigger>` | open from the start | Open the annotation window |
| `+trace_symbol_stop=<trigger>` | never closes | Close it |

```sh
make verilator-run TRACER=1 HEXFILE=sw/out/hello.hex SIM_ARGS="+trace_verbose=1"
```

### Triggers

Anything that can be switched on and off is delimited by a *window*, and every window takes the
same kind of trigger at each end:

| Trigger | Fires when |
| --- | --- |
| `1000` or `cycle:1000` | the cycle counter reaches 1000 |
| `pc:0x80000100` | the program counter is this address |
| `sym:main` | the program counter is this symbol's entry point |

The instruction meeting the start condition is inside the window; the one meeting the stop
condition is not. Both ends keep being evaluated, so a `pc:` or `sym:` trigger reopens the
window every time execution passes through it — which is what you want for tracing every call
to a function. A window with no start trigger is open from the first instruction, and one with
no stop trigger never closes.

```sh
# just the body of main
SIM_ARGS="+trace_symbols=sw/out/hello.elf +trace_start=sym:main +trace_stop=sym:_exit"
```

The closing summary reports how much was traced when a window was in play:
`41 instructions traced of 125 retired`.

### Symbol annotation

Off unless `+trace_symbols=` names at least one ELF file. Each source may be written
`<path>@<bias>`, and the bias is added to every address it contributes — which is how an image
assembled from several separately linked binaries is handled: name each one with the address it
was actually placed at.

```sh
SIM_ARGS="+trace_symbols=sw/out/hello.elf,hw/bootrom/cva6tb_bootrom.elf"
```

`header` mode, the default, prints an objdump style line whenever execution enters a different
symbol and leaves the instruction lines untouched:

```
000000008000018e <main>:
     868ns      859 M 000000008000018e 0 00000797 auipc          a5, 0x0               a5  :000000008000018e
```

`inline` mode appends `<name+0x..>` to every line instead, and `both` does the two together.

Only code symbols are kept: `STT_FUNC` and untyped labels such as `_start`, and only from
sections that are both allocated and executable. That drops the many symbols living in debug
sections, and RISC-V `$xrv64i...` mapping symbols are skipped by name.

Annotation has its own window, so it can be narrowed independently of the trace itself with
`+trace_symbol_start=` and `+trace_symbol_stop=`.

A rejected option — an unreadable ELF, an unknown symbol in a trigger, a misspelled mode —
stops the simulation rather than quietly producing the wrong trace.

### Output formats

`+trace_format=` picks how a line looks. The tracer decides *what* to report — window, symbols,
decode — and a formatter decides how it is spelled, so a new format is one class in
[rv_format.cc](rv_format.cc) and a name in the factory, with nothing else touched.

| Format | Purpose |
| --- | --- |
| `legacy` (default) | the format of `instr_tracer.sv`, meant to be read |
| `spike` | byte for byte what `spike -l --log-commits` writes, for diffing against spike |
| `rvfi` | the same information in `rvfi_tracer.sv`'s spelling, for the `verif/sim/` converters |
| `binary` | fixed size records, for tools rather than for people |

Both commit log dialects write two lines per instruction: the instruction, then its
architectural effect — privilege, PC, encoding, the register written and its value, and
` mem <addr>` / ` mem <addr> <data>` for loads and stores.

```sh
SIM_ARGS="+trace_format=spike"
```

```
core   0: 0x0000000080000008 (0x6783031b) addiw   t1, t1, 1656
core   0: 3 0x0000000080000008 (0x6783031b) x6  0x0000000000005678
```

**Why there are two dialects.** They are not interchangeable, and neither is a superset. Real
spike prefixes *both* lines with `core   N: ` and left aligns the register as `x6  0x…`;
`rvfi_tracer.sv` omits the prefix on the effect line and right aligns as `x 6 0x…`. The
converters under `verif/sim/` were written against the second: measured on the same run,
`verilator_log_to_trace_csv.py`'s `RD_RE` matches 106 of the `rvfi` effect lines and **0** of the
`spike` ones. So use `spike` to diff against spike, and `rvfi` to feed those scripts.

Traps follow the same split: `spike` writes `core   0: exception trap_illegal_instruction,
epc 0x…`, which is the wording the converters' `ILLE_RE` looks for, while `rvfi` writes
`ILLEGAL_INSTR exception @ 0x<pc> (0x<insn>)`.

**What `spike` still cannot reproduce.** Spike records CSR writes as `c832_mscratch 0x…`. RVFI
does not carry them per instruction — CVA6 exposes CSR state only through the separate
`rvfi_csr` structure, a snapshot of every register with write masks, which is not plumbed into
the retire call. Closing that gap is real work on the RVFI side, not a formatting choice. The
disassembly text also differs here and there, since it comes from this tracer's disassembler
rather than spike's: the layout is matched, but the pseudo-instruction choices are the legacy
tracer's (`sltiu` where spike says `seqz`, and so on).

Verbosity levels do not apply to either commit log dialect, whose shape is fixed, and symbol
header lines are suppressed for both because they would break a parser reading the stream. The
inline annotation form still works.

### The binary format

`+trace_format=binary` writes a 64 byte header and then one fixed size record per event, little
endian and naturally aligned. There is nothing to parse: a reader checks the header and indexes
into the rest.

```python
import numpy as np
rec = np.dtype([('cycle','<u8'), ('time_ns','<u8'), ('pc','<u8'), ...])
data = np.fromfile('trace.bin', dtype=rec, offset=64)
mispredicted = data[(data['flags'] & 0x09) == 0x09]     # resolved branch that missed
loads        = data[data['class'] & 0x01]               # no decoder needed
```

Each record carries a `class` byte saying what the instruction is — load, store, atomic, CSR,
branch, jump, compressed — written by this tracer's decoder. Without it a consumer would have to
reimplement RISC-V decoding just to count an instruction mix, which is the one thing a trace of
raw encodings cannot answer on its own. With it, a dozen lines of standard library Python
reproduce every number in the statistics report; that equivalence is checked in *Validation*.

[`util/read_trace.py`](../../util/read_trace.py) is the reference reader and documents the
layout field by field; it also converts to CSV and dumps records in a readable form:

```sh
./util/read_trace.py trace.bin --info      # header and record count
./util/read_trace.py trace.bin --csv       # csv on stdout
./util/read_trace.py trace.bin --limit 20  # readable dump
```

Retirements and traps share one record shape, told apart by `kind`; a trap puts its cause and
`tval` in the `rd_wdata` and `rs1_rdata` slots. Fields keep their offsets across versions and
new ones come out of `reserved`, so a reader that checks `version` and `record_size` keeps
working — `class` arrived that way in version 2, and the reader reports it as unavailable on a
version 1 file rather than silently counting zero loads. On `clic_smode` the binary file is 209 728 bytes against 261 772 for the same run as
text, and the difference in parsing cost is larger than the difference in size.

### Comparing against spike

[`util/cmp_spike_trace.py`](../../util/cmp_spike_trace.py) diffs a spike commit log against a
trace taken in `spike` format and reports the first place they disagree:

```sh
spike --isa=rv64imac_zicsr -l --log-commits prog.elf > spike.log 2>&1
./build/Vtest +binary=prog.hex +trace_format=spike +trace_file=rtl.log
./util/cmp_spike_trace.py spike.log rtl.log
```

```
aligned on pc 0x80000000: spike entry 5, trace entry 46
comparing 4995 instructions (spike has 4995 from here, trace has 6428)
MATCH: 4995 instructions identical (disassembly text and spike csr records ignored)
```

The two runs do not start in the same place — spike begins at its own reset vector while the
testbench goes through its boot ROM — so the logs are aligned on the first program counter they
have in common, or on `--start-pc`. The disassembly text and spike's CSR records are ignored by
default for the reasons above; `--strict` compares the text as well. A disagreement is reported
with both sides of the offending instruction:

```
#3 diverges:
  spike 0x0000000080000008 (0x6783031b)  addiw t1, t1, 1656  [x6 0x0000000000005678]
  trace 0x0000000080000008 (0x6783031b)  addiw t1, t1, 1656  [x6 0x000000000000dead]
```

The exit status is 0 when they match, so it drops into a regression script.

One caveat on what this can check: spike and the testbench only agree if the program does not
depend on anything outside the core. The tests under `sw/tests/` write to the EOC register and
the simulation console, which bare spike has no model for, so they diverge as soon as they touch
one. Self contained programs compare cleanly end to end.

### Verbosity levels

**0** — the legacy layout, nothing else. This is what you want when diffing against an older
`trace_hart_*.log`.

**1** — adds, per line: the data moved by loads and stores (`ST:`/`LD:`), the CSR touched by a
CSR instruction and the value written to it (`csr:mtvec <-…`), addresses for AMOs, the outcome
of a resolved branch (`br:taken`, `br:not-taken,mispredicted`), a `<- trap entry` /
`<- interrupt entry` marker on the first instruction of a handler, and a second line under each
trap carrying the short cause name and the raw cause register. A `#` header names the columns.

The branch annotation makes the trace answer branch questions on its own:

```sh
grep -o "br:[a-z,-]*" cva6tb_trace_hart_0.txt | sort | uniq -c
   9607 br:not-taken
   1512 br:not-taken,mispredicted
  72853 br:taken
   2951 br:taken,mispredicted
```

**2** — additionally stamps each line with the commit port and the retire counter, `[p0 #1234]`,
between the encoding and the disassembly. Useful for dual-issue questions; not diff compatible.

## End of run statistics

`+trace_stats=<path>` writes a summary when the run ends. It is off unless a path is given, and
it goes to its own file, so the trace stays a trace.

```sh
SIM_ARGS="+trace_stats=stats.txt +trace_symbols=sw/out/hello.elf"
```

The collector in [rv_stats.cc](rv_stats.cc) *observes* the same retirements the formatter is
handed, rather than reading the formatted text. That keeps the numbers independent of which
output format is in use, and a second collector counting something else hangs off the same two
`record()` calls without touching the tracer or the formatters.

It reports instruction and cycle counts with IPC, the privilege split, the instruction mix with
bytes moved, branch and mispredict rates, traps grouped by cause, and — when symbols are loaded
— which of them the instructions went to:

```
Branch prediction
  resolved control flow           86923   23.6%
  of those taken                  75804   87.2%
  of those mispredicted            4463    5.1%

Where the instructions went (38 symbols, top 15)
  vprintfmt.constprop.1           93937   25.5%
  _boot                           67670   18.3%
  hpt_init                        56085   15.2%
```

Statistics cover what was traced, so a window narrows them too. To get a summary without
keeping a trace, point the trace at `/dev/null` and only the summary survives.

## What RVFI cannot tell us

- **Mispredictions** are not part of RVFI. CVA6's branch unit resolves them into `bp_resolve_t`,
  and patch `0004` carries `is_mispredict` out alongside the `branch_valid` and `is_taken` that
  the probe already forwarded, on `rvfi_to_iti_t` rather than on RVFI itself. Leave
  `branch_valid_i` / `branch_taken_i` / `branch_mispredict_i` tied to `'0` and the column reads
  `0` everywhere, as it did before the probe existed.
- **Third source operands.** RVFI reports two source values. For the FMA instructions the third
  one prints as `----------------`. The same fallback covers the few CVA6 instructions whose
  scoreboard operand slots do not line up with the architectural `rs1`/`rs2` — mostly the FP
  ops, where the tracer matches decoded operands against the reported ones by register number
  and falls back to consuming them in order.
- **`tval`.** Not part of RVFI; taken from `cva6_rvfi`'s `rvfi_to_iti_o.tval`, which is
  registered in the same cycle as the RVFI outputs.

## A note on physical addresses

`rvfi.mem_paddr` used to be driven from [`store_buffer.sv`](../../../core/store_buffer.sv), which
made it meaningful for stores only — loads reported whatever address the last store had left
behind. It is now driven from the output of the translation stage in
[`load_store_unit.sv`](../../../core/load_store_unit.sv) and paired back up with its access
inside [`cva6_rvfi.sv`](../../../core/cva6_rvfi.sv), which delays the transaction id by the one
cycle the MMU and PMP stages take. Neither the RVFI interface nor the probe bundle changed.

## Instruction coverage

| Group | Covered |
| --- | --- |
| RV32I / RV64I, M, A, F, D, C | yes |
| Zicsr, Zifencei | yes, with CSR names |
| H (`hlv*`, `hsv*`, `hfence.*`) | yes |
| Zba, Zbb, Zbs, Zbc | yes |
| Zicbom, Zicboz (`cbo.*`) | yes |
| Zfh (half precision) | yes, via the format field |
| Zcb (`c.lbu`, `c.mul`, `c.zext.*`, …) | yes |
| Zcmt (`cm.jt`, `cm.jalt`) | yes, see below |
| V (vector), Zfa, Zk\* | no |

Zfa and the crypto extensions are not implemented by CVA6, so they are left out rather than
guessed at. Vector needs `vtype`/`vl` tracking to render sensibly and is a separate job.

Unknown encodings print as `INVALID` with the raw word still in the encoding column, so nothing
is silently dropped.

**Zcmt needs a hint.** `cm.jt` and `cm.jalt` occupy the same encoding as `c.fsdsp`, and the two
are mutually exclusive by construction — a configuration has one or the other. The decoder
cannot tell from the instruction word alone, so the SV side passes `CVA6Cfg.RVZCMT` down to the
backend as an extension mask. Zcb needs no such hint because it only uses otherwise reserved
encodings.

CSR names cover the machine, supervisor, hypervisor and CLIC registers of the
`cv64a6_imafdchsclic_sv39*` configurations, and trap causes cover the hypervisor codes. In CLIC
mode the cause register packs the saved interrupt level and privilege above the exception code;
the tracer masks those off and prints the raw value at verbosity 1.

## Core RVFI changes it depends on

Three bugs in CVA6's RVFI generation and one missing probe, found while building this tracer.
Each is a separate commit on top of `pulp-v3`, so each can be reviewed, reverted or sent
upstream on its own — `git log --oneline pulp-v3..` lists them, and `git format-patch pulp-v3..
-- core/` regenerates them as patches.

- **Per access physical address.** `rvfi.mem_paddr` was driven from the store buffer's
  speculative queue, so it only ever described stores: a load reported whatever address the
  previous store had left behind, and the first load of a program reported zero. It is now
  driven from the output of the translation stage and stored per scoreboard entry. See *A note
  on physical addresses* above.
- **Interrupt cause bit on RV64.** The test was `ex_commit_cause[31]`, but the interrupt bit of
  `mcause` is its most significant bit — bit 63 on RV64. Every interrupt was therefore reported
  as a synchronous trap, with `rvfi.intr` tagged `'b11` instead of `'b101`. RV32 was unaffected.
- **Environment call from VS-mode.** CVA6 reports an `ecall` as a retirement rather than a trap,
  because the instruction does execute; it just transfers control. The list of causes given that
  treatment named machine, supervisor and user mode but not virtual supervisor mode.
  `riscv::ENV_CALL_VSMODE` already existed and was simply never added when the hypervisor
  extension landed. On `riscv-hyp-tests` this moves 25 traps into the retirement count.
- **Branch mispredict probe.** Not a bug, a gap. `cva6_rvfi_probes` already receives the whole
  `bp_resolve_t` and already forwards `valid` and `is_taken`; `is_mispredict` sits in the same
  struct and was never copied out, so nothing downstream could tell a mispredicted branch from a
  correctly predicted one — the legacy `instr_tracer` had that column and RVFI based tracing
  could not fill it. It travels on `rvfi_to_iti_t`, not `rvfi_instr_t`: a mispredict is not part
  of RVFI.

## Validation

- **Against the legacy tracer.** Under QuestaSim the legacy `instr_tracer` can be compiled in
  alongside this one with `LEGACY_TRACE=1`, so both write a trace for the same run. Ignoring
  the time and cycle stamps, all 125 lines of
  `hello` match byte for byte, and 2181 of 2184 of `clic_smode`. Every difference is one of
  those listed under *deliberate differences* below; the cycle stamps differ by a constant 5
  (the legacy tracer counts from the core's internal reset, this one from the SoC reset, plus
  the RVFI register stage).

  To repeat the comparison, build with both tracers and strip the stamps. The two write to
  different files by default, so nothing needs renaming:

  ```sh
  make vsim-build TRACER=1 LEGACY_TRACE=1
  make vsim-run   TRACER=1 LEGACY_TRACE=1 HEXFILE=sw/out/clic_smode.hex
  cd vsim/build
  grep -vE "^Exception|tval:" trace_hart_0.log | sed -E 's/^ *[0-9]+ns +[0-9]+ //' > legacy.txt
  sed -E 's/^ *[0-9]+ns +[0-9]+ //' cva6tb_trace_hart_0.txt > new.txt
  diff legacy.txt new.txt
  ```
- **Against spike.** Of the 3548 distinct encodings the `riscv-hyp-tests` image actually
  executes, 3521 (99.2%) disassemble to the same mnemonic as `spike-dasm`. The 27 that differ
  are all spike pseudo-instructions the legacy tracer deliberately renders as the underlying
  instruction: `seqz` (19), `sext.w` (4), `snez` (3) and `ret` for a compressed `c.jr` (1).
- **The binary format against the text one.** The PC and encoding sequences read back from a
  binary trace are identical to the legacy text trace over all 2184 instructions of
  `clic_smode`, and the record count matches retirements plus traps exactly on the 368 959
  record hypervisor run. Recomputing the instruction mix and branch statistics from the binary
  file alone, using only the `class` and `flags` bytes and no decoder, reproduces every figure
  the in-tracer collector reports for the same run.
- **Against spike, instruction by instruction.** A self contained program run both on real
  spike (`spike -l --log-commits`) and on the testbench in `spike` format agrees on **4995 of
  4995 instructions**: same PCs, same encodings, same register writes, same memory effects,
  checked with `util/cmp_spike_trace.py`. Corrupting one register value in the trace is caught
  and pinpointed, so the comparison is not vacuous.
- **RV32.** `cv32a6_imac_sv32` elaborates and runs with the tracer unchanged — the boot ROM
  happens to compile to identical code for both widths, so only the test software needs
  rebuilding with RV32 flags. `hello` passes and traces correctly, with the PC and VA columns
  narrowing to the 32 bit widths. Separately, all 48 uncompressed instructions of an RV32
  coverage file match objdump exactly, including the encodings that differ between the two
  widths: `c.jal` rather than `c.addiw`, `c.flw`/`c.fsw` rather than `c.ld`/`c.sd`, and the RV32
  forms of `rev8` and `zext.h`.
- **With virtual memory.** The `riscv-hyp-tests` image passes with tracing on: 368 894
  instructions and 65 traps. 42 accesses show a physical address genuinely different from the
  virtual one, including the same VA resolving to two different PAs across a guest page table
  change, which is what the per-access address fix is there for. All hypervisor instruction
  forms appear, and the only `INVALID` in 369 000 lines is an instruction guest page fault
  whose encoding is genuinely zero because the fetch never completed.

## Deliberate differences from the legacy tracer

- `amomaxu.w` / `amomaxu.d` are named correctly; the legacy tracer printed them as `amomax`.
- The RV64M 32-bit forms print as `mulw`, `divw`, `divuw`, `remw`, `remuw`; the legacy tracer
  appended a `w` to every mnemonic in the group and produced names like `mulhw`.
- `csrs` and `csrc` are printed for the `rd == x0` forms. The legacy tracer listed both
  mnemonics but its `casez` ordering made the patterns unreachable, so it always said `csrrs`
  and `csrrc`.
- An `ecall` appears as a retired instruction. The legacy tracer printed only the
  `Exception @ ... Environment Call ...` line for it, because CVA6's RVFI reports environment
  calls as retirements.
- Interrupts produce no line at verbosity 0, where the legacy tracer printed
  `Exception @ ... Cause: ... Interrupt`. RVFI reports an interrupt on the *handler's* first
  instruction rather than on the interrupted one, so the tracer marks it there instead, as
  `<- interrupt entry` at verbosity 1. Moving that marker down to verbosity 0 is a one line
  change if the parity matters more than a clean diff.
- Exception lines stamp a plain nanosecond count instead of `%t`-formatted simulation time.
- Mnemonics and operands are derived from the encoding rather than from CVA6 scoreboard state.
  The rendering rules are the legacy ones, oddities included: `c.j` still prints as
  `c.j x0, <offset>` and `c.mv` as `c.mv rd, x0, rs2`, because that is the decompressed form the
  old tracer showed.
