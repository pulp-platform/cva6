#!/bin/bash
set -e
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

ci/make-tmp.sh
cd tmp
git clone https://github.com/pulp-platform/CHERI-Litmus.git riscv-litmus-tests
cd riscv-litmus-tests/frontend
# check out hash until release
git checkout 174253ec83e851cec2a7b6c10e6d40e8daf3ab07
./make.sh
cd ../binaries
# Build tests for 2 cores
./make-riscv.sh ../tests/ cva6 2
