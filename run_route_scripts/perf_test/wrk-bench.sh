#! /usr/bin/env bash

# Benchmarking script
#
# This script runs a load testing experiment on a local or remote Valhalla
# instance. It uses wrk (https://github.com/wg/wrk) to generate latency and peak
# throughput statistics for a given number of concurrent clients, sweeping
# through a range of concurrency loads and reports a CSV to stderr.
#
# Usage:
#
#  Output to the console. URI endpoint defaults to http://localhost:8002
#
#    bash wrk-bench.sh scripts/trace_route.lua http://localhost:8002
#
#  Add a custom name to results:
#
#    TEST_NAME=custom-name bash wrk-bench.sh scripts/trace_route.lua
#
#  Redirect report data to a file:
#
#    bash wrk-bench.sh scripts/trace_route.lua 2> >(tee results.csv)

set -euo pipefail

script=$1
endpoint=${2:-http://localhost:8002}
test_name=${TEST_NAME:-benchmark}
duration_sec=${DURATION_SEC:-30}
num_trials=${NUM_TRIALS:-5}
timeout_sec=${TIMEOUT_SEC:-10}

num_threads=2

# Invoke the benchmark
benchmark () {
    threads="$1"
    connections="$2"
    trial_no="$3"
    echo "event=benchmark threads=$threads connections=$connections duration_sec=$duration_sec trial_no=$trial_no"
    # Need to add the Lua module import path
    export LUA_PATH="scripts/?.lua;"
    wrk --script "$script"  \
        --threads "$threads" \
        --latency \
        --timeout "$timeout_sec"s \
        --connections "$connections" \
        --duration "$duration_sec"s \
        "$endpoint"
    # Log out some parameters of the test, which we will use for plotting.
    # These apparently can't be passthroughed through wrk into its Lua API
    # Note, logged out for redirecting with tabular output
    >&2 echo "concurrency,test_name,trial_no,duration(sec)"
    >&2 echo "$connections,$test_name,$trial_no,$duration_sec"
    echo "--END--"
}

trials=$(seq "$num_trials")

# Run a single-threaded benchmark
for i in $trials; do benchmark 1 1 "$i"; done

# Run a concurrent benchmark up to 100 callers
concurrency=( 2 4 8 10 20 30 40 50 60 70 80 90 100)
for i in "${concurrency[@]}"
do
    for j in $trials; do benchmark "$num_threads" "$i" "$j"; done
done
