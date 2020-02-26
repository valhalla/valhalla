# Performance Benchmarking

This directory contains some scripts for performing HTTP-based benchmarking.

## `url_de_benchmark_routes.txt`

The following command will run `siege`, a multi-threaded http load testing and benchmarking utility,
with 4 concurrent simulated users, 2550 repetitions per user, and the `url_de_benchmark_routes.txt`
requests file.

```
$ siege -c 4 -b -r 2550 -f url_de_benchmark_routes.txt
```

## Load-based Benchmarking

The [`wrk-bench.sh`](https://github.com/wg/wrk) script measures Valhalla's query
throughput under varying concurrency load.

Installation:

```bash
# On MacOS
$ brew install wrk

# On Ubuntu/Debian
$ sudo apt-get install build-essential libssl-dev git -y
$ git clone https://github.com/wg/wrk.git wrk
$ cd wrk
$ make
$ sudo cp wrk /usr/local/bin

# If plotting/analyzing results
$ pip3 install pandas click
```

Assuming your local Valhalla is running and listening at `localhost:8002`:

```bash
# Start benchmark
$ TEST_NAME=bencmark-name bash wrk-bench.sh scripts/trace_route.lua http://localhost:8002 2> >(tee results.csv)

# Split files into results.csv.measurements.csv and results.csv.metadata.csv
$ bash wrk-csv-filter.sh results.csv

# Plot metadata
$ python3 wrk-analysis.py plot-throughput results.csv.measurements.csv results.csv.metadata.csv
```

`wrk-bench.sh` relies on Lua scripts defined in `scripts/` to perform
benchmarking. To update or modify these scripts, check out wrk's scripting API
[documentation][lua_wrk_docs].

[lua_wrk_docs]: https://github.com/wg/wrk/blob/master/SCRIPTING
