#!/bin/bash
set -e
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

ci/make-tmp.sh
cd tmp
git clone https://github.com/pulp-platform/CHERI-Litmus.git riscv-litmus-tests
cd riscv-litmus-tests/frontend
./make.sh
cd ../binaries
# Build tests for 2 cores
./make-riscv.sh ../tests/ 2
