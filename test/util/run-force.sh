#!/bin/bash
# Generates programs with FORCE-RISCV and runs them co-simulated.
#
# FORCE is an instruction sequence generator: it knows the ISA, so it can build
# programs with paging, privilege switching and non-trivial handlers. The checking is the same --
# the programs report success and the co-simulation decides.
#
#   make -C test force                      # builds FORCE and installs the port
#   make -C test verilator-build COSIM=1 CVA6_CONFIG=cv64a6_imafdc_sv39_wb
#   ./util/run-force.sh --count 10
#
# SPDX-License-Identifier: Apache-2.0

set -uo pipefail

COUNT=${COUNT:-5}
SEED=${SEED:-}
INSNS=${INSNS:-2000}
MAX_CYCLES=${MAX_CYCLES:-4000000}
JOBS=${JOBS:-8}
# Privilege switches to spread through the body. Off by default: see
# sw/fuzz/force/cva6tb_random_force.py -- with translation on FORCE generates
# one to two instructions a second, so this is for a few deep programs rather
# than for volume.
SWITCHES=${SWITCHES:-0}
KEEP=0
# How many retirements to compare before stopping. A FORCE test ends in a branch
# to itself -- that is the framework's end-of-test convention, and the simulator
# is expected to notice -- so the program never writes the testbench's EOC
# register and the run ends on a budget. FORCE emits more instructions than the
# template asks for, because it materialises its own
# addresses and operands, so the default allows for that plus its boot code.
STOP_AFTER=${STOP_AFTER:-}

while [ $# -gt 0 ]; do
    case "$1" in
        --count) COUNT="$2"; shift 2 ;;
        --seed)  SEED="$2"; shift 2 ;;
        --insns) INSNS="$2"; shift 2 ;;
        --jobs)  JOBS="$2"; shift 2 ;;
        --switches) SWITCHES="$2"; shift 2 ;;
        --stop-after) STOP_AFTER="$2"; shift 2 ;;
        --keep)  KEEP=1; shift ;;
        -h|--help) sed -n '2,14p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown option '$1'" >&2; exit 2 ;;
    esac
done
[ -n "${STOP_AFTER}" ] || STOP_AFTER=$(( INSNS * 4 + 20000 ))

scriptdir="$(dirname -- "${BASH_SOURCE[0]}")"
testdir="$(realpath -m "${scriptdir}/..")"
forcedir="${FORCE_DIR:-${testdir}/tools/force-riscv}"
simbin="${testdir}/verilator/build/Vtest"
workdir="${testdir}/verilator/force"
# FORCE embeds the host's CPython, so this script runs on the host. Each
# simulation runs in $SIM_ENV, the environment the model is built in, which
# util/iis-env.sh sets, or directly when it is empty.
SIM_ENV="${SIM_ENV:-}"
RV_BINROOT="${RV_BINROOT:-$(dirname "$(command -v riscv64-unknown-elf-gcc 2>/dev/null)")}"
bootrom="${testdir}/hw/bootrom/cva6tb_bootrom.bin"
# The same three the riscv-tests runner uses, plus deleg for the same reason:
# the CSR comparison reaches medeleg, whose delegatable set is implementation
# defined. The port pins mstatus's reserved span to zero, which keeps both
# models little-endian; see sw/fuzz/force/register_field_choices-cva6tb.xml.
COSIM_RULES="${COSIM_RULES:-misa-ro,lrsc,ids,deleg}"

for f in "${forcedir}/bin/friscv" "${simbin}" "${bootrom}"; do
    [ -e "$f" ] || { echo "error: $f missing" >&2; exit 1; }
done

mkdir -p "${workdir}"

# One seed: generate, convert, simulate, and decide. The verdict is the
# co-simulation's, not the program's: a generated program reports nothing, so a
# seed passes when the requested retirements were compared and none diverged.
# Written to a status file, so that the seeds can run in parallel and still be
# reported in order.
run_seed() {
    local i="$1" d="${workdir}/$1"
    rm -rf "${d}"; mkdir -p "${d}"
    # -s pins FORCE's own RNG, so the seed number names the program: a seed that
    # diverges replays with --seed and produces the same one. Without it FORCE
    # seeds itself and a divergence is gone the moment the directory is cleaned.
    ( cd "${d}" && CVA6TB_FORCE_INSNS="${INSNS}" CVA6TB_FORCE_SWITCHES="${SWITCHES}" \
        "${forcedir}/bin/friscv" -c config/cores/cva6tb/riscv-cva6tb.config \
        -s "${i}" \
        -t "${forcedir}/config/cores/cva6tb/cva6tb_random_force.py" ) > "${d}/gen.log" 2>&1
    local elf="${d}/cva6tb_random_force.Default.ELF"
    if [ ! -f "${elf}" ]; then
        echo "GEN" > "${d}/status"; return
    fi
    "${RV_BINROOT}/riscv64-unknown-elf-objcopy" -O verilog "${elf}" "${d}/test.hex"

    ${SIM_ENV} "${simbin}" +binary="${d}/test.hex" +MaxCycles="${MAX_CYCLES}" \
        +cosim_bootrom="${bootrom}" +cosim_rules="${COSIM_RULES}" \
        +cosim_stop_after="${STOP_AFTER}" > "${d}/sim.log" 2>&1

    if grep -q 'MISMATCH on' "${d}/sim.log"; then
        echo "DIVERGED" > "${d}/status"
    elif grep -q 'retirement budget spent' "${d}/sim.log"; then
        echo "OK" > "${d}/status"
    else
        # Neither diverged nor reached the budget: the program ended early, hung
        # before it got there, or the model never started. Not a divergence, and
        # not a result either, so it is reported on its own.
        echo "SHORT" > "${d}/status"
    fi
}

# --seed runs exactly that one, which is how a divergence is replayed.
if [ -n "${SEED}" ]; then FIRST_SEED="${SEED}"; COUNT=$(( SEED )); else FIRST_SEED=1; fi

pass=0; fail=0; short=0
seed="${FIRST_SEED}"
while [ "${seed}" -le "${COUNT}" ]; do
    running=0
    batch_first="${seed}"
    while [ "${running}" -lt "${JOBS}" ] && [ "${seed}" -le "${COUNT}" ]; do
        run_seed "${seed}" &
        seed=$((seed+1)); running=$((running+1))
    done
    wait
    for i in $(seq "${batch_first}" $((seed-1))); do
        d="${workdir}/${i}"
        status="$(cat "${d}/status" 2>/dev/null || echo SHORT)"
        case "${status}" in
            OK)
                echo "PASS ${i} :: $(grep -oE '[0-9]+ instructions compared' "${d}/sim.log")"
                pass=$((pass+1))
                [ "${KEEP}" = "1" ] || rm -rf "${d}"
                ;;
            DIVERGED)
                echo "FAIL ${i} :: $(grep -m1 'MISMATCH on' "${d}/sim.log" | sed 's/.*MISMATCH on //')"
                fail=$((fail+1))
                ;;
            GEN)
                echo "FAIL ${i} :: generation"; fail=$((fail+1))
                ;;
            *)
                echo "SHORT ${i} :: $(grep -oE '[0-9]+ instructions compared' "${d}/sim.log" 2>/dev/null)"
                short=$((short+1))
                ;;
        esac
    done
done
echo "---"
printf 'force: %d passed, %d diverged' "${pass}" "${fail}"
[ "${short}" = "0" ] || printf ', %d short' "${short}"
printf ' of %d\n' "$(( pass + fail + short ))"
[ "${fail}" = "0" ] && [ "${short}" = "0" ]
[ "${fail}" -eq 0 ]
