# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0
#
# A FORCE-RISCV test template for the cva6tb testbench.
#
# FORCE picks the registers, the addresses and the data; this only says which
# instructions to draw from and how many. As with the simpler generator next
# door, the program checks nothing itself -- the co-simulation is the checker --
# and the run ends on a retirement budget; see the end of generate().
#
# Run it with the cva6tb configuration, which restricts memory to the DRAM the
# testbench instantiates:
#
#   bin/friscv -c config/cores/cva6tb/riscv-cva6tb.config \
#              -t config/cores/cva6tb/cva6tb_random_force.py

import os

from base.Sequence import Sequence
from riscv.EnvRISCV import EnvRISCV
from riscv.GenThreadRISCV import GenThreadRISCV

# The instruction pool. FORCE names an instruction by its encoding family, and
# the same reasoning applies here as in gen_random_test.py: what is left out is
# the design work. LR/SC is absent because whether a reservation survives is
# implementation defined, and the CLIC and counter registers because they differ
# between the models by definition.
INSTRUCTIONS = [
    "ADD##RISCV", "SUB##RISCV", "SLL##RISCV", "SRL##RISCV", "SRA##RISCV",
    "SLT##RISCV", "SLTU##RISCV", "XOR##RISCV", "OR##RISCV", "AND##RISCV",
    "ADDI##RISCV", "SLTI##RISCV", "SLTIU##RISCV", "XORI##RISCV", "ORI##RISCV",
    "ANDI##RISCV", "SLLI#RV64I#RISCV", "SRLI#RV64I#RISCV", "SRAI#RV64I#RISCV",
    "ADDW##RISCV", "SUBW##RISCV", "SLLW##RISCV", "SRLW##RISCV", "SRAW##RISCV",
    "ADDIW##RISCV", "SLLIW##RISCV", "SRLIW##RISCV", "SRAIW##RISCV",
    "LUI##RISCV", "AUIPC##RISCV",
    "MUL##RISCV", "MULH##RISCV", "MULHSU##RISCV", "MULHU##RISCV", "MULW##RISCV",
    "DIV##RISCV", "DIVU##RISCV", "REM##RISCV", "REMU##RISCV",
    "DIVW##RISCV", "DIVUW##RISCV", "REMW##RISCV", "REMUW##RISCV",
    "LB##RISCV", "LBU##RISCV", "LH##RISCV", "LHU##RISCV",
    "LW##RISCV", "LWU##RISCV", "LD##RISCV",
    "SB##RISCV", "SH##RISCV", "SW##RISCV", "SD##RISCV",
    "FADD.D##RISCV", "FSUB.D##RISCV", "FMUL.D##RISCV", "FDIV.D##RISCV",
    "FADD.S##RISCV", "FSUB.S##RISCV", "FMUL.S##RISCV", "FDIV.S##RISCV",
]



class MainSequence(Sequence):
    def generate(self, **kargs):
        # From the environment, which works with whichever FORCE version is
        # checked out: its option interface differs between versions.
        count = int(os.environ.get("CVA6TB_FORCE_INSNS", "2000"))
        # How many privilege switches to spread through the body. This is the
        # part of FORCE that the simpler generator next door cannot reach: a
        # switch into S or U mode brings address translation, the xstatus
        # transitions and the trap handlers FORCE builds into the comparison,
        # and every load and store after it is a translated one.
        #
        # It is off by default because it is expensive to *generate*: with
        # translation on, FORCE's state solving makes generation slow. Use a
        # handful of seeds and a short body, and give it time.
        switches = int(os.environ.get("CVA6TB_FORCE_SWITCHES", "0"))

        if switches <= 0:
            for _ in range(count):
                self.genInstruction(self.choice(INSTRUCTIONS))
            return

        # Instructions between switches, with the switches themselves spread
        # evenly so that each privilege level gets a comparable share of the
        # body.
        chunk = max(1, count // (switches + 1))
        emitted = 0
        for i in range(switches + 1):
            n = chunk if i < switches else count - emitted
            for _ in range(n):
                self.genInstruction(self.choice(INSTRUCTIONS))
            emitted += n
            if i < switches:
                # FORCE sets up the target mode, its page tables and its return
                # path; M is in the pool so the program comes back up as well as
                # down. It also picks the interrupt mask, which is harmless here:
                # the testbench raises no interrupt at a generated program.
                self.systemCall({"PrivilegeLevel": self.choice([0, 1, 3])})

        # The program does not end itself. FORCE's own end-of-test is a branch
        # to itself -- the framework expects the simulator to notice -- and its
        # operand solver materialises its own address into the register a store
        # uses, which rules out an EOC write here. The run ends on
        # +cosim_stop_after: the comparison is the result, so "the requested
        # retirements were compared and none diverged" is the verdict. See
        # util/run-force.sh.


MainSequenceClass = MainSequence
GenThreadClass = GenThreadRISCV
EnvClass = EnvRISCV
