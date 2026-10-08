#!/bin/bash
# Generates random programs and runs them co-simulated.
#
# The programs check nothing themselves -- they always report success. The
# co-simulation is the checker, which is what makes generated stimulus worth
# anything: a generator has to produce a *legal* program, not a correct answer,
# and that is a far lower bar.
#
#   make -C test verilator-build COSIM=1 CVA6_CONFIG=cv64a6_imafdc_sv39_wb
#   ./util/run-fuzz.sh --count 100
#
# A seed that diverges is kept, with its assembly, its hex and the simulation
# log, so it can be replayed on its own:
#
#   ./util/run-fuzz.sh --seed 1234 --keep
#
# SPDX-License-Identifier: Apache-2.0

set -uo pipefail

COUNT=${COUNT:-20}
SEED=${SEED:-}
INSNS=${INSNS:-2000}
NJOBS=${NJOBS:-8}
MAX_CYCLES=${MAX_CYCLES:-2000000}
MARCH=${MARCH:-rv64imafdc_zba_zbb_zbs_zicond}
KEEP=0
GEN_ARGS=""

while [ $# -gt 0 ]; do
    case "$1" in
        --count)  COUNT="$2"; shift 2 ;;
        --seed)   SEED="$2"; shift 2 ;;
        --insns)  INSNS="$2"; shift 2 ;;
        --jobs)   NJOBS="$2"; shift 2 ;;
        --keep)   KEEP=1; shift ;;
        --no-fp)  GEN_ARGS="${GEN_ARGS} --no-fp"; shift ;;
        --no-traps) GEN_ARGS="${GEN_ARGS} --no-traps"; shift ;;
        --no-loops) GEN_ARGS="${GEN_ARGS} --no-loops"; shift ;;
        --no-umode) GEN_ARGS="${GEN_ARGS} --no-umode"; shift ;;
        --no-counters) GEN_ARGS="${GEN_ARGS} --no-counters"; shift ;;
        --no-smode) GEN_ARGS="${GEN_ARGS} --no-smode"; shift ;;
        --no-warl) GEN_ARGS="${GEN_ARGS} --no-warl"; shift ;;
        --lrsc)   GEN_ARGS="${GEN_ARGS} --lrsc"; shift ;;
        -h|--help)
            sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        *) echo "unknown option '$1'" >&2; exit 2 ;;
    esac
done

scriptdir="$(dirname -- "${BASH_SOURCE[0]}")"
export testdir="$(realpath -m "${scriptdir}/..")"
export simbin="$(realpath -m "${testdir}/verilator/build/Vtest")"
workdir="$(realpath -m "${testdir}/verilator/fuzz")"
export workdir
export RV_BINROOT="${RV_BINROOT:-$(dirname "$(command -v riscv64-unknown-elf-gcc 2>/dev/null)")}"
export MAX_CYCLES INSNS MARCH KEEP GEN_ARGS

if [ ! -x "${simbin}" ]; then
    echo "error: ${simbin} not built -- run 'make verilator-build COSIM=1 CVA6_CONFIG=cv64a6_imafdc_sv39_wb'" >&2
    exit 1
fi

export bootrom="$(realpath -m "${testdir}/hw/bootrom/cva6tb_bootrom.bin")"
if [ ! -f "${bootrom}" ]; then
    echo "error: ${bootrom} missing -- run 'make -C test bootrom'" >&2
    exit 1
fi
# The rule sets the WARL CSR probing needs. Each is a legal, implementation
# defined divergence the probing reaches on every seed --
# medeleg's delegatable set, the vectored trap vector alignment, mstatus's
# endianness bits, the number of hpmcounters -- and each is documented in
# hw/cosim/README.md. Anything that diverges with these enabled is news.
export COSIM_RULES="${COSIM_RULES:-misa-ro,lrsc,ids,deleg,tvec,endianness,hpm}"

run_one () {
    seed="$1"
    d="${workdir}/${seed}"
    mkdir -p "${d}"
    "${testdir}/util/gen_random_test.py" --seed "${seed}" --insns "${INSNS}" \
        ${GEN_ARGS} -o "${d}/fuzz.S" || { echo "FAIL ${seed} :: generator"; return; }
    "${RV_BINROOT}/riscv64-unknown-elf-gcc" -march="${MARCH}" -mabi=lp64d \
        -nostdlib -nostartfiles -T "${testdir}/sw/fuzz/link.ld" \
        -o "${d}/fuzz.elf" "${d}/fuzz.S" > "${d}/build.log" 2>&1 \
        || { echo "FAIL ${seed} :: assembler"; return; }
    "${RV_BINROOT}/riscv64-unknown-elf-objcopy" -O verilog "${d}/fuzz.elf" "${d}/fuzz.hex"

    "${simbin}" +binary="${d}/fuzz.hex" +MaxCycles="${MAX_CYCLES}" \
        +cosim_bootrom="${bootrom}" +cosim_rules="${COSIM_RULES}" \
        > "${d}/sim.log" 2>&1

    if grep -q "SIMULATION PASSED" "${d}/sim.log"; then
        echo "PASS ${seed}"
        [ "${KEEP}" = "1" ] || rm -rf "${d}"
    else
        echo "FAIL ${seed} :: $(grep -m1 'MISMATCH on' "${d}/sim.log" | sed 's/.*MISMATCH on //')"
    fi
}
export -f run_one

mkdir -p "${workdir}"
if [ -n "${SEED}" ]; then
    seeds="${SEED}"
else
    seeds="$(seq 1 "${COUNT}")"
fi

printf '%s\n' ${seeds} | xargs -n1 -P "${NJOBS}" bash -c 'run_one "$1"' _ \
    > "${workdir}/summary.txt" || true

pass=$(grep -c '^PASS' "${workdir}/summary.txt" || true)
fail=$(grep -c '^FAIL' "${workdir}/summary.txt" || true)
grep '^FAIL' "${workdir}/summary.txt" || true
echo "---"
echo "fuzz: ${pass} passed, ${fail} diverged of $(( pass + fail ))"
echo "diverging seeds are kept under ${workdir}/<seed>/"
[ "${fail}" -eq 0 ]
