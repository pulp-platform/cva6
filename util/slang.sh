#!/bin/bash
# Elaborate one configuration with Slang the way the CI's Slang job checks it:
# the RTL of the configuration's Bender target under the AXI example top, which
# sets cva6_top's configuration and types from that configuration.
#
#   util/slang.sh <config>          elaborate the configuration
#   util/slang.sh --flags <config>  write <config>.f here, print Slang's flags for it
#
# The hpdcache submodule must be checked out; Bender fetches the rest.

SLANG=${SLANG:-slang}
BENDER=${BENDER:-bender}
TOP=cva6_example_axi

ROOT=$(cd "$(dirname "$0")/.." && pwd)

# Write the file list of configuration $1 to $2
flist() {
  (cd "$ROOT" && $BENDER script flist-plus -t rtl -t "$1") > "$2" &&
    echo "$ROOT/core/$TOP.sv" >> "$2"
}

if [ "$1" = --flags ] && [ $# -eq 2 ]; then
  flist "$2" "$PWD/$2.f" && echo "-f $PWD/$2.f --top $TOP"
elif [ $# -eq 1 ]; then
  f=$(mktemp)
  trap 'rm -f "$f"' EXIT
  flist "$1" "$f" && $SLANG -f "$f" --top $TOP
else
  echo "usage: $0 [--flags] <config>" >&2
  exit 1
fi
