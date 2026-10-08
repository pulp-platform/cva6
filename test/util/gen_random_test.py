#!/usr/bin/env python3
#
# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
#
# Enrico Zelioli <ezelioli@iis.ee.ethz.ch>
#
# Generates a random RISC-V program for co-simulation.
#
# The programs have no expected answer and do not check anything themselves --
# they always report success. That is the point: the co-simulation is the
# checker, so a generator does not have to know what the right answer is, only
# how to produce a legal program. That is a far lower bar than writing a test,
# and it is the whole reason co-simulation is worth having.
#
# What the generator must guarantee is that a divergence means a bug:
#
#   * every register and every byte of memory it reads is initialised first,
#     because the reference zero-fills and an uninitialised read would differ
#     for no interesting reason;
#   * memory accesses stay inside a scratch buffer it owns;
#   * control flow only ever goes forward, so a program always terminates;
#   * instructions with a known, understood divergence are not emitted at all
#     (see hw/cosim/README.md) -- a fuzzer that keeps rediscovering them is
#     noise.

import argparse
import random
import sys

# Registers the harness keeps for itself: x0 is hardwired, x1 holds the scratch
# base, x2 the stack, x3 the end-of-computation register address, and x4 is the
# trap handler's, so that the handler needs no save area and cannot be reached
# by a body instruction that happens to pick the same register.
RESERVED = {0, 1, 2, 3, 4}
HANDLER_REG = 4
GPRS = [r for r in range(32) if r not in RESERVED]
FPRS = list(range(32))

SCRATCH_WORDS = 256  # 2 KiB of scratch, initialised before use

RV64_REG_OPS = [
    "add", "sub", "sll", "slt", "sltu", "xor", "srl", "sra", "or", "and",
    "addw", "subw", "sllw", "srlw", "sraw",
]
RV64_IMM_OPS = ["addi", "slti", "sltiu", "xori", "ori", "andi", "addiw"]
RV64_SHIFT_IMM = ["slli", "srli", "srai"]
RV64_SHIFTW_IMM = ["slliw", "srliw", "sraiw"]
MUL_OPS = ["mul", "mulh", "mulhsu", "mulhu", "mulw"]
DIV_OPS = ["div", "divu", "rem", "remu", "divw", "divuw", "remw", "remuw"]
# Zba / Zbb / Zbs, which riscv-tests does not cover; --no-bitmanip leaves them
# out for a configuration without B.
BITMANIP_REG = ["sh1add", "sh2add", "sh3add", "add.uw", "sh1add.uw",
                "sh2add.uw", "sh3add.uw", "andn", "orn", "xnor", "max", "maxu",
                "min", "minu", "rol", "ror", "bclr", "bext", "binv", "bset"]
BITMANIP_IMM = ["bclri", "bexti", "binvi", "bseti", "rori"]
BITMANIP_UNARY = ["clz", "ctz", "cpop", "sext.b", "sext.h", "zext.h", "clzw",
                  "ctzw", "cpopw", "orc.b", "rev8"]
ZICOND = ["czero.eqz", "czero.nez"]
FP_REG_OPS_D = ["fadd.d", "fsub.d", "fmul.d", "fdiv.d", "fmin.d", "fmax.d",
                "fsgnj.d", "fsgnjn.d", "fsgnjx.d"]
FP_REG_OPS_S = ["fadd.s", "fsub.s", "fmul.s", "fdiv.s", "fmin.s", "fmax.s",
                "fsgnj.s", "fsgnjn.s", "fsgnjx.s"]
LOADS = [("lb", 1), ("lbu", 1), ("lh", 2), ("lhu", 2), ("lw", 4), ("lwu", 4),
         ("ld", 8)]
STORES = [("sb", 1), ("sh", 2), ("sw", 4), ("sd", 8)]
AMOS = ["amoadd.w", "amoadd.d", "amoand.w", "amoand.d", "amoor.w", "amoor.d",
        "amoxor.w", "amoxor.d", "amoswap.w", "amoswap.d", "amomax.w",
        "amomax.d", "amominu.w", "amominu.d"]
# Only the two scratch registers. Everything else interesting here is WARL and
# an implementation may narrow it: CVA6 keeps the interrupt bit and the low bits
# of mcause's exception code and drops the rest, masks mepc to an even address,
# and so on, while a reference model stores what it was given. Both are correct,
# so a random value written to one of those compares two legal choices; the
# WARL probing below covers them, with save and restore.
#
# Read-only counters and identification registers are absent for the same
# reason: they differ by definition and are covered by rules.
CSRS_RW = ["mscratch", "sscratch"]
# CSRs probed by writing a pattern and reading it back. The point is the WARL
# behaviour: which bits stick, which read back masked, which are hardwired. That
# is where a core's CSR legalisation lives -- the pmpaddr NAPOT readback, the
# stvec vector base, the counter enables, the FIOM bit -- which csrprobe.S also
# probes by hand.
#
# Each entry is (csr, mask): the pattern is ANDed with the mask before being
# written, for the bits that must not be set.
#
# Left out, each for a reason:
#   mstatus   MBE/SBE/UBE are a legal WARL divergence (the `endianness` rule
#             set), so a pattern write would rediscover it on every seed.
#   mcause,   CVA6 narrows these and a reference model stores what it is given;
#   scause    also a legal divergence, already documented.
#   mie, mip  arrival is structural, and enabling an interrupt here would reach
#             a handler these programs do not have.
#   misa      hardwired on CVA6 and writable on the reference (`misa-ro`).
WARL_CSRS = [
    ("pmpaddr0", None), ("pmpaddr1", None), ("pmpaddr2", None),
    ("pmpaddr3", None), ("pmpaddr4", None), ("pmpaddr5", None),
    ("pmpaddr6", None), ("pmpaddr7", None),
    # pmpcfg has its own pattern, see warl_pmpcfg_pattern().
    ("pmpcfg0", "pmpcfg"),
    # And so do menvcfg and senvcfg, see warl_envcfg_pattern().
    ("menvcfg", "envcfg"), ("senvcfg", "envcfg"),
    ("mcounteren", None), ("scounteren", None), ("mcountinhibit", None),
    ("medeleg", None), ("mideleg", None),
    ("mtvec", None), ("stvec", None),
    ("mepc", None), ("sepc", None),
    ("mtval", None), ("stval", None),
    ("satp", None),
    ("frm", None), ("fcsr", None),
]

def warl_pmpcfg_pattern(rng):
    """A pmpcfg value whose every byte is a configuration both models accept.

    Two bytes are avoided, for different reasons:

    L, the lock bit, because a locked entry cannot be unlocked before reset and
    the restore would not take -- the program would lose the PMP region its
    U-mode segments need.

    A = NA4, because with a granularity of 8 bytes it is not selectable, and the
    two models resolve that differently: CVA6 keeps the entry's previous A and a
    reference substitutes NAPOT. Both are legal for a WARL field. Leaving NA4
    unwritten keeps the address mode of every entry compared.
    """
    value = 0
    for byte in range(8):
        # R, W and X, skipping W-without-R, which is reserved and which both
        # models fold back to the previous value.
        rwx = rng.choice([0, 1, 4, 5, 7])
        a = rng.choice([0, 1, 3])  # OFF, TOR, NAPOT -- never NA4
        value |= ((a << 3) | rwx) << (byte * 8)
    return value


def warl_envcfg_pattern(rng):
    """A menvcfg or senvcfg value whose CBIE field is a legal encoding.

    CBIE, bits 5:4, reserves the encoding 10, and the two models resolve a write
    of it differently: CVA6 keeps the field's previous value and a reference
    writes 00. Both are legal for a WARL field. The pattern keeps its other bits
    and takes CBIE from the three legal encodings, so the field stays compared.
    """
    pattern = rng.choice(WARL_PATTERNS)
    cbie = rng.choice([0, 1, 3])  # disabled, flush, invalidate -- never 10
    return (pattern & ~(0x3 << 4)) | (cbie << 4)


# Patterns worth writing. All-ones is what csrprobe.S uses, because it is the
# one that shows every implemented bit at once; the rest find bits that are
# masked together.
WARL_PATTERNS = [
    0xffffffffffffffff, 0x0000000000000000, 0xaaaaaaaaaaaaaaaa,
    0x5555555555555555, 0xffffffff00000000, 0x00000000ffffffff,
]

# The unprivileged counters, readable from M always and from U only when both
# mcounteren and scounteren allow it -- see counter() below.
#
# `time` is absent. Zicntr requires the CSR, and the architecture lets the
# execution environment provide it: CVA6 takes that option -- csr_regfile.sv has
# no CSR_TIME case, so rdtime is an illegal instruction in every mode -- and this
# platform puts mtime in the CLINT at 0x0204_bff8. A reference built with Zicntr
# implements the CSR, so the two legitimately disagree on every read.
COUNTERS = ["cycle", "instret"]


class Generator:
    def __init__(self, seed, n_insns, weights):
        self.rng = random.Random(seed)
        self.seed = seed
        self.n_insns = n_insns
        self.weights = weights
        self.out = []
        # Registers no emitter may write while something outside it depends on
        # their value -- a loop counter, for the length of the loop body.
        self.locked = set()

    def emit(self, line):
        self.out.append("  " + line)

    def gpr(self, exclude=()):
        while True:
            r = self.rng.choice(GPRS)
            if r not in exclude and r not in self.locked:
                return r

    def fpr(self):
        return self.rng.choice(FPRS)

    # --- instruction shapes -------------------------------------------------

    def alu_reg(self):
        op = self.rng.choice(RV64_REG_OPS)
        self.emit(f"{op} x{self.gpr()}, x{self.gpr()}, x{self.gpr()}")

    def alu_imm(self):
        op = self.rng.choice(RV64_IMM_OPS)
        self.emit(f"{op} x{self.gpr()}, x{self.gpr()}, {self.rng.randint(-2048, 2047)}")

    def shift_imm(self):
        if self.rng.random() < 0.5:
            op, hi = self.rng.choice(RV64_SHIFT_IMM), 63
        else:
            op, hi = self.rng.choice(RV64_SHIFTW_IMM), 31
        self.emit(f"{op} x{self.gpr()}, x{self.gpr()}, {self.rng.randint(0, hi)}")

    def muldiv(self):
        # Division by zero and the signed overflow case are defined in RISC-V,
        # so they are in the mix: they are exactly the corners worth comparing.
        op = self.rng.choice(MUL_OPS + DIV_OPS)
        self.emit(f"{op} x{self.gpr()}, x{self.gpr()}, x{self.gpr()}")

    def bitmanip(self):
        kind = self.rng.random()
        if kind < 0.5:
            op = self.rng.choice(BITMANIP_REG)
            self.emit(f"{op} x{self.gpr()}, x{self.gpr()}, x{self.gpr()}")
        elif kind < 0.8:
            op = self.rng.choice(BITMANIP_IMM)
            self.emit(f"{op} x{self.gpr()}, x{self.gpr()}, {self.rng.randint(0, 63)}")
        else:
            op = self.rng.choice(BITMANIP_UNARY)
            self.emit(f"{op} x{self.gpr()}, x{self.gpr()}")

    def zicond(self):
        op = self.rng.choice(ZICOND)
        self.emit(f"{op} x{self.gpr()}, x{self.gpr()}, x{self.gpr()}")

    def mem(self):
        # Offsets are aligned and inside the scratch buffer, so every access is
        # to memory this program initialised.
        if self.rng.random() < 0.5:
            op, size = self.rng.choice(LOADS)
            off = self.rng.randrange(0, SCRATCH_WORDS * 8 - 8, size)
            self.emit(f"{op} x{self.gpr()}, {off}(x1)")
        else:
            op, size = self.rng.choice(STORES)
            off = self.rng.randrange(0, SCRATCH_WORDS * 8 - 8, size)
            self.emit(f"{op} x{self.gpr()}, {off}(x1)")

    def amo(self):
        op = self.rng.choice(AMOS)
        size = 8 if op.endswith(".d") else 4
        off = self.rng.randrange(0, SCRATCH_WORDS * 8 - 8, size)
        addr = self.gpr()
        self.emit(f"addi x{addr}, x1, {off}")
        self.emit(f"{op} x{self.gpr(exclude=(addr,))}, x{self.gpr(exclude=(addr,))}, (x{addr})")

    def lrsc(self):
        addr = self.gpr()
        off = self.rng.randrange(0, SCRATCH_WORDS * 8 - 8, 8)
        dst, src = self.gpr(exclude=(addr,)), self.gpr(exclude=(addr,))
        self.emit(f"addi x{addr}, x1, {off}")
        self.emit(f"lr.d x{dst}, (x{addr})")
        self.emit(f"sc.d x{self.gpr(exclude=(addr,))}, x{src}, (x{addr})")

    def branch(self, label_id):
        # Forward only, so the program always terminates. The branch may or may
        # not be taken; both sides are compared.
        op = self.rng.choice(["beq", "bne", "blt", "bge", "bltu", "bgeu"])
        self.emit(f"{op} x{self.gpr()}, x{self.gpr()}, {label_id}f")
        self.alu_reg()
        self.out.append(f"{label_id}:")

    def fp(self):
        kind = self.rng.random()
        if kind < 0.4:
            op = self.rng.choice(FP_REG_OPS_D)
            self.emit(f"{op} f{self.fpr()}, f{self.fpr()}, f{self.fpr()}")
        elif kind < 0.7:
            op = self.rng.choice(FP_REG_OPS_S)
            self.emit(f"{op} f{self.fpr()}, f{self.fpr()}, f{self.fpr()}")
        elif kind < 0.85:
            # Moves between the files, which is where NaN boxing shows up.
            self.emit(f"fmv.x.d x{self.gpr()}, f{self.fpr()}")
        else:
            self.emit(f"fmv.d.x f{self.fpr()}, x{self.gpr()}")

    def loop(self, label_id):
        """A backward branch with a bounded trip count.

        Control flow is otherwise forward-only so that a program always
        terminates without a watchdog, and a counted loop keeps that guarantee
        while being the only thing here that runs the same instructions twice.
        That matters because forwarding, the branch predictor and the
        non-blocking cache behave differently on the second pass -- and a
        mispredicted backward branch is a different case from a mispredicted
        forward one.

        The counter is locked for the body, so nothing inside can write it and
        turn a bounded loop into an unbounded one.
        """
        counter = self.gpr()
        trips = self.rng.randrange(2, 9)
        self.emit(f"li   x{counter}, {trips}")
        self.out.append(f"{label_id}:")
        self.locked.add(counter)
        for _ in range(self.rng.randrange(2, 7)):
            kind = self.rng.choice(LOOP_BODY_KINDS)
            if kind in self.weights:
                getattr(self, kind)()
        self.locked.discard(counter)
        self.emit(f"addi x{counter}, x{counter}, -1")
        self.emit(f"bnez x{counter}, {label_id}b")

    def counter(self):
        """A read of an unprivileged counter.

        From M mode this always succeeds. From U mode it succeeds only when
        *both* mcounteren and scounteren allow that counter: mcounteren gates
        S and U, and scounteren gates U on top of it, so a clear bit in either
        one is an illegal instruction. The preamble sets the two to independent
        masks precisely so that the mcounteren-set, scounteren-clear case
        occurs -- that is the combination a wrong implementation lets through,
        and it cannot be reached from M mode at all.

        The value itself is not comparable, since the two models count at
        different rates; the co-simulation's built-in counter rules copy the
        DUT's value into the reference before it steps, so what is compared here
        is whether the access was permitted, which is the part that can be
        wrong.
        """
        self.emit(f"csrr x{self.gpr()}, {self.rng.choice(COUNTERS)}")

    def warl(self):
        """Write a pattern to a CSR and read it back.

        What is compared is the readback, through the destination register and
        through the CSR snapshot both: a WARL field that masks differently in
        the two models shows up here and nowhere else in a generated program.

        The original value is saved first and restored after, so the program
        stays legal -- a random mtvec would send the next trap into the weeds,
        a random satp would turn on translation with no page tables, and a
        pmpcfg without its lock bits masked could not be put back at all. The
        window between write and restore is two instructions of M-mode CSR
        access, neither of which traps, so nothing can be taken inside it.

        M mode only. From U or S the first access would trap on permission
        before reaching the WARL behaviour, which is a different test.
        """
        csr, mask = self.rng.choice(WARL_CSRS)
        if mask == "pmpcfg":
            pattern = warl_pmpcfg_pattern(self.rng)
        elif mask == "envcfg":
            pattern = warl_envcfg_pattern(self.rng)
        else:
            pattern = self.rng.choice(WARL_PATTERNS)
            if mask is not None:
                pattern &= mask
        saved = self.gpr()
        value = self.gpr(exclude=(saved,))
        back = self.gpr(exclude=(saved, value))
        self.emit(f"csrr x{saved}, {csr}")
        self.emit(f"li   x{value}, {hex(pattern)}")
        self.emit(f"csrw {csr}, x{value}")
        self.emit(f"csrr x{back}, {csr}          # the readback is the comparison")
        self.emit(f"csrw {csr}, x{saved}")

    def smode(self, label_id):
        """A stretch of the body that runs in S mode.

        S mode is where delegation lives. A trap taken here goes to the S
        handler through stvec when medeleg permits that cause, and to the M
        handler when it does not, so sepc, scause, stval and sret are compared
        on one path and mepc, mcause, mtval and mret on the other -- with the
        same program and the same instruction deciding which.

        Entry and exit are the U-mode mechanism with MPP of 1 instead of 0.
        """
        tmp = self.gpr()
        self.emit(f"li   x{tmp}, (3 << 11)          # mstatus.MPP")
        self.emit(f"csrc mstatus, x{tmp}")
        self.emit(f"li   x{tmp}, (1 << 11)          # ... to S")
        self.emit(f"csrs mstatus, x{tmp}")
        self.emit(f"la   x{tmp}, {label_id}f")
        self.emit(f"csrw mepc, x{tmp}")
        self.emit("mret")
        self.out.append(f"{label_id}:")
        for _ in range(self.rng.randrange(3, 13)):
            kind = self.rng.choice(SMODE_BODY_KINDS)
            if kind in self.weights:
                getattr(self, kind)()
        self.emit("ecall                       # back to M mode")

    def umode(self, label_id):
        """A stretch of the body that runs in U mode.

        What U mode adds is not a different instruction mix -- the ALU does not
        care -- but permission. A CSR access that is legal in M is an illegal
        instruction here, and the loads and stores only work because the
        preamble opens a PMP region for them, so both the PMP match and the
        privilege check are in the comparison for the length of the segment.

        Entry is mret with mstatus.MPP cleared; the way back is ecall, which the
        handler recognises as cause 8 and returns from to M mode. Both models
        take the same path, so the mode itself is compared on every retirement
        in between -- which is the one field that was constant until now.
        """
        tmp = self.gpr()
        self.emit(f"li   x{tmp}, (3 << 11)          # mstatus.MPP")
        self.emit(f"csrc mstatus, x{tmp}            # ... to U")
        self.emit(f"la   x{tmp}, {label_id}f")
        self.emit(f"csrw mepc, x{tmp}")
        self.emit("mret")
        self.out.append(f"{label_id}:")
        for _ in range(self.rng.randrange(3, 13)):
            kind = self.rng.choice(UMODE_BODY_KINDS)
            if kind in self.weights:
                getattr(self, kind)()
        self.emit("ecall                       # back to M mode")

    def trap(self):
        """One instruction that is meant to trap, with mtvec pointing at a
        handler that steps over it.

        Every kind here traps on any conformant implementation, which is the
        whole requirement: the reference takes the same trap, so cause, tval and
        the trap CSRs are compared. All four are 32-bit encodings, which lets the
        handler add a fixed 4 to mepc.
        """
        kind = self.rng.choice(["ecall", "illegal", "misaligned_load",
                                "misaligned_store"])
        if kind == "ecall":
            # Environment call from M mode, cause 11. No tval.
            self.emit("ecall")
        elif kind == "illegal":
            # An all-zero word is not a valid encoding in any extension, so this
            # is cause 2 everywhere. mtval carries the instruction itself.
            self.emit(".word 0x00000000            # illegal instruction")
        else:
            # Misaligned, by one byte off an aligned slot so the access stays
            # inside the scratch buffer either way. CVA6 traps on a misaligned
            # access, so cause 4 or 6 with mtval the address.
            op, size = self.rng.choice(
                [("lh", 2), ("lw", 4), ("ld", 8)] if kind == "misaligned_load"
                else [("sh", 2), ("sw", 4), ("sd", 8)])
            off = self.rng.randrange(0, SCRATCH_WORDS * 8 - 16, 8) + 1
            self.emit(f"{op} x{self.gpr()}, {off}(x1)")

    def csr(self):
        csr = self.rng.choice(CSRS_RW)
        op = self.rng.choice(["csrrw", "csrrs", "csrrc"])
        self.emit(f"{op} x{self.gpr()}, {csr}, x{self.gpr()}")

    # --- program ------------------------------------------------------------

    def preamble(self):
        p = self.out.append
        p("# Generated by util/gen_random_test.py -- do not edit.")
        p(f"# seed {self.seed}, {self.n_insns} instructions")
        p("")
        p('.section .text.init, "ax"')
        p(".globl _start")
        p("_start:")
        self.emit("la   x1, scratch")
        self.emit("la   x2, stack_top")
        self.emit("li   x3, 0x10000004        # end-of-computation register")
        p("")
        self.emit("# enable the FP unit before touching any f register")
        self.emit("li   t0, (3 << 13)")
        self.emit("csrs mstatus, t0")
        self.emit("fscsr x0")
        p("")
        if "trap" in self.weights or "umode" in self.weights:
            self.emit("# take traps through the handler below, mtvec direct mode")
            self.emit("la   t0, trap_handler")
            self.emit("csrw mtvec, t0")
            p("")
        if "umode" in self.weights:
            self.emit("# One PMP region over everything, RWX. Without a matching entry a")
            self.emit("# U-mode access fails where the same access from M succeeds, so")
            self.emit("# nothing in a U-mode segment could touch the scratch buffer.")
            self.emit("li   t0, -1")
            self.emit("csrw pmpaddr0, t0")
            self.emit("li   t0, 0x1f               # NAPOT, X|W|R")
            self.emit("csrw pmpcfg0, t0")
            p("")
        if "smode" in self.weights:
            self.emit("# Delegated traps land here, in S mode.")
            self.emit("la   t0, s_trap_handler")
            self.emit("csrw stvec, t0")
            deleg = 0
            for bit, _ in DELEGABLE.items():
                if self.rng.random() < 0.5:
                    deleg |= 1 << bit
            names = ", ".join(n for b, n in DELEGABLE.items() if deleg & (1 << b))
            self.emit(f"li   t0, {hex(deleg)}" +
                      (f"              # {names}" if names else "              # nothing"))
            self.emit("csrw medeleg, t0")
            p("")
        if "counter" in self.weights:
            mc = self.rng.getrandbits(3)
            sc = self.rng.getrandbits(3)
            self.emit("# Independent counter enables, so that a U-mode read finds every")
            self.emit("# combination of the two gates.")
            self.emit(f"li   t0, {mc}")
            self.emit("csrw mcounteren, t0")
            self.emit(f"li   t0, {sc}")
            self.emit("csrw scounteren, t0")
            p("")
        self.emit("# initialise the scratch buffer: every byte read below is written here first")
        self.emit("mv   t0, x1")
        self.emit(f"li   t1, {SCRATCH_WORDS}")
        self.emit("li   t2, 0x0123456789abcdef")
        self.out.append("1:")
        self.emit("sd   t2, 0(t0)")
        self.emit("addi t0, t0, 8")
        self.emit("xor  t2, t2, t1")
        self.emit("addi t1, t1, -1")
        self.emit("bnez t1, 1b")
        p("")
        self.emit("# initialise every architectural register the body may read")
        seed_val = self.rng.getrandbits(64)
        for r in GPRS:
            self.emit(f"li   x{r}, {hex(self.rng.getrandbits(32))}")
        for f in FPRS:
            self.emit(f"li   t0, {hex(self.rng.getrandbits(64))}")
            self.emit(f"fmv.d.x f{f}, t0")
        # t0 is in GPRS and was just clobbered; give it a value again.
        self.emit(f"li   t0, {hex(seed_val & 0xffffffff)}")
        p("")
        p("  # ---- generated body ----")

    def body(self):
        choices, weights = zip(*self.weights.items())
        label_id = 10
        for _ in range(self.n_insns):
            kind = self.rng.choices(choices, weights=weights, k=1)[0]
            if kind in ("branch", "loop", "umode", "smode"):
                getattr(self, kind)(label_id)
                label_id += 1
                if label_id > 99:
                    label_id = 10
            else:
                getattr(self, kind)()

    def epilogue(self):
        p = self.out.append
        p("")
        p("  # ---- end of computation: always a pass, the co-simulation is the checker ----")
        self.emit("li   t0, 1                 # bit 0 ends the run, code 0 in bits 31:1")
        self.emit("sw   t0, 0(x3)")
        self.out.append("2:")
        self.emit("j    2b")
        p("")
        if "trap" in self.weights:
            p("  # ---- trap handler ----")
            p("  #")
            p("  # Returns to the instruction after the one that trapped. Every")
            p("  # deliberate trap is a 32-bit encoding, so the step is a fixed 4")
            p("  # and the handler never has to look at mcause. A trap the")
            p("  # generator did not intend is stepped over the same way, and is")
            p("  # still compared -- the co-simulation is what reports it, not the")
            p("  # program.")
            p("  #")
            p("  # 256-byte aligned: CVA6 requires that of a vectored trap vector")
            p("  # base, and aligning here keeps mtvec writable to the same value")
            p("  # in both models whichever mode is selected.")
            p(".balign 256")
            p("trap_handler:")
            self.emit(f"csrr x{HANDLER_REG}, mepc")
            self.emit(f"addi x{HANDLER_REG}, x{HANDLER_REG}, 4")
            self.emit(f"csrw mepc, x{HANDLER_REG}")
            if "umode" in self.weights or "smode" in self.weights:
                p("  # An ecall from a lower mode -- cause 8 from U, cause 9 from S -- is")
                p("  # how a segment asks to come back; mret alone would return to")
                p("  # mstatus.MPP, which the trap set to the mode that trapped. Every")
                p("  # other cause does return there, so a fault inside a segment does")
                p("  # not end the segment.")
                self.emit(f"csrr x{HANDLER_REG}, mcause")
                self.emit(f"addi x{HANDLER_REG}, x{HANDLER_REG}, -8")
                self.emit(f"sltiu x{HANDLER_REG}, x{HANDLER_REG}, 2")
                self.emit(f"beqz x{HANDLER_REG}, 3f")
                self.emit(f"li   x{HANDLER_REG}, (3 << 11)")
                self.emit(f"csrs mstatus, x{HANDLER_REG}   # MPP = M")
                self.out.append("3:")
            self.emit("mret")
            p("")
        if "smode" in self.weights:
            p("  # ---- S-mode trap handler ----")
            p("  #")
            p("  # Where a delegated trap lands. sret returns to sstatus.SPP, which the")
            p("  # trap set to the mode that trapped, so a delegated fault inside an S")
            p("  # or U segment returns to that segment. Nothing an ecall raises is")
            p("  # delegated, so this handler is never the one that ends a segment.")
            p(".balign 256")
            p("s_trap_handler:")
            self.emit(f"csrr x{HANDLER_REG}, sepc")
            self.emit(f"addi x{HANDLER_REG}, x{HANDLER_REG}, 4")
            self.emit(f"csrw sepc, x{HANDLER_REG}")
            self.emit("sret")
            p("")
        p('.section .data')
        p(".balign 8")
        p("scratch:")
        p(f"  .zero {SCRATCH_WORDS * 8}")
        p(".balign 16")
        p("  .zero 4096")
        p("stack_top:")

    def generate(self):
        self.preamble()
        self.body()
        self.epilogue()
        return "\n".join(self.out) + "\n"


# What a U-mode segment may contain. No control flow, and no deliberate trap:
# the segment's own ecall is how it returns to M mode, so a second trap inside
# it would end the segment early and leave the rest running in M.
#
# csr is in here because mscratch and sscratch are both privileged, so a U-mode
# access to either is an illegal instruction --
# which is the permission check this whole segment exists to exercise, and the
# handler steps over it without ending the segment.
UMODE_BODY_KINDS = ["alu_reg", "alu_imm", "shift_imm", "muldiv", "bitmanip",
                    "zicond", "mem", "fp", "csr", "counter"]

# What an S-mode segment may contain. The same set: csr is interesting here for
# a different reason -- sscratch is accessible from S and mscratch is not, so
# half the accesses are permitted and half are illegal instructions -- and a
# counter read from S is gated by mcounteren alone, which is the other branch of
# the check scounteren guards for U.
SMODE_BODY_KINDS = UMODE_BODY_KINDS

# Causes delegated to S mode: the three the deliberate traps raise. Ecalls (8,
# 9) stay with M: an ecall is how a segment asks to come back to M, and the M
# handler is what ends a segment. Which causes medeleg permits delegating is
# implementation defined, and the two models disagree on bits 0, 9 and 19 (the
# co-simulation's `deleg` rule set); these three are delegatable in both, so
# they need no rule.
DELEGABLE = {2: "illegal instruction", 4: "load misaligned", 6: "store misaligned"}

# What a loop body may contain. No branch, no loop and no trap: a nested label
# or a trap inside a counted loop is legal but makes the trip count harder to
# reason about than the coverage is worth.
LOOP_BODY_KINDS = ["alu_reg", "alu_imm", "shift_imm", "muldiv", "bitmanip",
                   "zicond", "mem", "amo", "fp", "csr"]


DEFAULT_WEIGHTS = {
    "alu_reg": 20,
    "alu_imm": 20,
    "shift_imm": 10,
    "muldiv": 8,
    "bitmanip": 12,
    "zicond": 3,
    "mem": 15,
    "amo": 4,
    "branch": 8,
    "fp": 10,
    "csr": 3,
    "trap": 4,
    "loop": 4,
    "umode": 4,
    "counter": 4,
    "smode": 4,
    "warl": 6,
}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--insns", type=int, default=2000,
                    help="instructions in the generated body (default 2000)")
    ap.add_argument("-o", "--output", default="-")
    ap.add_argument("--no-fp", action="store_true", help="omit floating point")
    ap.add_argument("--no-bitmanip", action="store_true",
                    help="omit Zba, Zbb and Zbs, for a configuration without B")
    ap.add_argument("--no-mem", action="store_true", help="omit loads and stores")
    ap.add_argument("--no-warl", action="store_true",
                    help="omit the CSR pattern writes and readback checks")
    ap.add_argument("--no-smode", action="store_true",
                    help="omit S-mode segments and trap delegation")
    ap.add_argument("--no-counters", action="store_true",
                    help="omit counter reads and the mcounteren/scounteren setup")
    ap.add_argument("--no-umode", action="store_true",
                    help="omit U-mode segments, leaving the program in M mode")
    ap.add_argument("--no-loops", action="store_true",
                    help="omit counted backward branches, leaving control flow "
                         "forward-only")
    ap.add_argument("--no-traps", action="store_true",
                    help="omit deliberate traps, leaving the program in M mode "
                         "with mtvec never taken")
    ap.add_argument("--lrsc", action="store_true",
                    help="emit LR/SC. Off by default: whether a reservation "
                         "survives is implementation defined, so the two models "
                         "disagree about whether an SC succeeded without either "
                         "being wrong")
    args = ap.parse_args()

    weights = dict(DEFAULT_WEIGHTS)
    if args.lrsc:
        weights["lrsc"] = 2
    if args.no_fp:
        weights.pop("fp")
    if args.no_bitmanip:
        weights.pop("bitmanip")
    if args.no_traps:
        weights.pop("trap")
    if args.no_loops:
        weights.pop("loop")
    if args.no_umode:
        weights.pop("umode")
    if args.no_counters:
        weights.pop("counter")
    if args.no_smode:
        weights.pop("smode")
    if args.no_warl:
        weights.pop("warl")
    if args.no_mem:
        for k in ("mem", "amo", "lrsc"):
            weights.pop(k, None)

    text = Generator(args.seed, args.insns, weights).generate()
    if args.output == "-":
        sys.stdout.write(text)
    else:
        with open(args.output, "w") as handle:
            handle.write(text)


if __name__ == "__main__":
    main()
