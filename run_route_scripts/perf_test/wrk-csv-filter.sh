#! /usr/bin/env bash

# Separate CSV records into separate files from wrk-bench.sh

set -euo pipefail

file=${1--}

make_csv() {
    # Line-by-line:
    # - select header line and values that follow after
    # - Filter '--' grep result noise
    # - Remove duplicate header lines
    cat "$file" | grep -A1 "$1" | sed '/^--/d' | awk '!seen[$0]++'
}

make_csv latency > "$file.measurements.csv"
make_csv trial_no > "$file.metadata.csv"
