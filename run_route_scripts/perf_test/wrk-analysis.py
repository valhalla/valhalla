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
    qps_mean = combined.groupby('concurrency').mean()['throughput_qps']
    qps_stddev = combined.groupby('concurrency').std()['throughput_qps']
    ax = qps_mean.plot(yerr=qps_stddev, label='QPS')
    plt.legend()
    ax.set_title('Throughput(QPS) Under Load')
    ax.set_xlabel('Number of Concurrent Users')
    ax.set_ylabel('Throughput (Queries/Second)')
    ax.get_figure().savefig(measurements + '.png')


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
