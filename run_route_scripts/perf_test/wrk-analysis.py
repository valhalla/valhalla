#! /usr/bin/env python3
"""
Output benchmarking analysis graphs.
"""

import sys

import click
import matplotlib.pyplot as plt
import pandas as pd


def analyze_benchmark(measurements, metadata):
    measurements_df = pd.read_csv(measurements)
    metadata_df = pd.read_csv(metadata)
    combined = measurements_df.join(metadata_df)
    combined['throughput_qps'] \
      = combined['summary.requests']/(combined['summary.duration(usec)']/10**6)
    # Plot separate performance envelopes for different benchmark tests.
    # groupby handles the situation where multiple tests appear in the same
    # file, which may happen if you concatenate results from multiple benchmark
    # tests into the single file. If you concatenate multiple CSVs together
    # from separate runs of wrk-bench, this will plot
    for test_name, df in combined.groupby('test_name'):
        qps_mean = df.groupby('concurrency').mean()['throughput_qps']
        qps_stddev = df.groupby('concurrency').std()['throughput_qps']
        ax = qps_mean.plot(yerr=qps_stddev, label='test_name')
    plt.legend()
    ax.set_title('Query Throughput Under Load')
    ax.set_xlabel('Number of Concurrent Clients')
    ax.set_ylabel('Throughput (Queries/Second)')
    ax.get_figure().savefig(measurements + '.png')
    combined.to_csv(measurements + '-combined.csv')


@click.group()
def main():
    pass


@main.command()
@click.argument('measurements',
                type=click.Path(file_okay=True, dir_okay=False))
@click.argument('metadata', type=click.Path(file_okay=True, dir_okay=False))
def plot_measurements(measurements, metadata):
    analyze_benchmark(measurements, metadata)


if __name__ == '__main__':
    sys.exit(main())
