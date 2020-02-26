#! /usr/bin/env python3
"""
Output benchmarking analysis graphs.
"""

import sys

import click
import matplotlib.pyplot as plt
import pandas as pd


def analyze_benchmark(measurements, metadata):
    # Parse
    measurements_df = pd.read_csv(measurements)
    metadata_df = pd.read_csv(metadata)
    combined = measurements_df.join(metadata_df)

    # Plot throughput
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
        ax = qps_mean.plot(yerr=qps_stddev, label=test_name)
    plt.legend()
    ax.set_title('Query Throughput Under Load')
    ax.set_xlabel('Number of Concurrent Clients')
    ax.set_ylabel('Throughput (Queries/Second)')
    ax.get_figure().savefig(measurements + '-throughput.png')

    # Plot mean latency
    # NOTE: Pandas boxplots apparently don't support labels :(
    if 'latency.mean(usec)' in combined:
        combined['latency.mean(sec)'] = combined['latency.mean(usec)'] / 10**6
        df = combined[combined['latency.mean(usec)'] != 0]
        plt.figure()
        for test_name, df in combined.groupby('test_name'):
            ax = df.boxplot(
                by='concurrency',
                column='latency.mean(sec)',
                grid=False,
            )
        ax.set_title('Latency (mean) Under Load')
        ax.set_xlabel('Number of Concurrent Clients')
        ax.set_ylabel('Latency (Seconds)')
        ax.get_figure().savefig(measurements + '-latency-mean.png')

    # Plot p99 latency
    if 'latency:percentile(99.0)(usec)' in combined:
        combined['latency:percentile(99.0)(sec)'] \
          = combined['latency:percentile(99.0)(usec)'] / 10**6
        df = combined[combined['latency:percentile(99.0)(usec)'] != 0]
        plt.figure()
        for test_name, df in df.groupby('test_name'):
            ax = df.boxplot(
                by='concurrency',
                column='latency:percentile(99.0)(sec)',
                grid=False,
            )
        ax.set_title('Latency (p99) Under Load')
        ax.set_xlabel('Number of Concurrent Clients')
        ax.set_ylabel('Latency (Seconds)')
        ax.get_figure().savefig(measurements + '-latency-p99.png')

    # Plot p50 latency
    if 'latency:percentile(50.0)(usec)' in combined:
        combined['latency:percentile(50.0)(sec)'] \
          = combined['latency:percentile(50.0)(usec)'] / 10**6
        df = combined[combined['latency:percentile(50.0)(usec)'] != 0]
        plt.figure()
        for test_name, df in df.groupby('test_name'):
            ax = df.boxplot(
                by='concurrency',
                column='latency:percentile(50.0)(sec)',
                grid=False,
            )
        ax.set_title('Latency (p50) Under Load')
        ax.set_xlabel('Number of Concurrent Clients')
        ax.set_ylabel('Latency (Seconds)')
        ax.get_figure().savefig(measurements + '-latency-p50.png')
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
