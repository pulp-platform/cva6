# The usual setup of the standalone testbench on IIS machines. Source it in bash
# from anywhere in the repository:
#
#   source test/util/iis-env.sh
#
# It puts RISC-V GCC 14.2, Ruby 3.4 and the SEPP tools on PATH, takes Verilator
# from the oseda container and QuestaSim from its SEPP wrapper, and activates
# the Python environment of test/, creating it on first use. Simulations and the
# regression runners then run inside the container, whose libraries the
# Verilator model is built against:
#
#   make -C test/sw all
#   make -C test verilator-build
#   $OSEDA make -C test verilator-run HEXFILE=sw/out/hello.hex
#   $OSEDA ./test/util/run-pulp-tests.sh verilator hello zicbom
#
# The architectural tests are generated on the host, with the RISC-V GCC 15,
# Sail and Ruby set here; the UDB gems go into a writable GEM_HOME:
#
#   GEM_HOME=<dir> make -C test/sw arch-test CVA6_CONFIG=<cfg>
#
# SPDX-License-Identifier: Apache-2.0

export OSEDA="oseda -2026.09"
export RISCV=/usr/pack/riscv-1.0-kgf/riscv64-gcc-14.2.0
export PATH="${RISCV}/bin:/usr/pack/ruby-3.4.13-mj/bin:/usr/sepp/bin:/usr/local/bin:${PATH}"
export ACT_RV_BINROOT=/usr/pack/riscv-1.0-kgf/riscv64-gcc-15.3.0/bin
export SAIL_DIR=/usr/pack/riscv-1.0-kgf/sail-riscv-0.14.1
export VERILATOR="${OSEDA} verilator"
export QUESTA=questa-2026.1

cva6tb_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
(cd "${cva6tb_dir}" && uv sync --locked --quiet) && . "${cva6tb_dir}/.venv/bin/activate"
unset cva6tb_dir
