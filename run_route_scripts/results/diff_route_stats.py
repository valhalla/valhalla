#!/usr/bin/env python3

import csv
import argparse


STATS_TO_DIFF = ['#Passes', 'runtime', 'trip time', 'length', '#Manuevers']


def main(old_stats_file, new_stats_file, output_file):

    with open(old_stats_file, 'r') as old_file, \
         open(new_stats_file, 'r') as new_file, \
         open(output_file, 'w', newline='') as output_csv:
        old_csv_reader = csv.reader(old_file)
        new_csv_reader = csv.reader(new_file)

        # Store header, stripping any whitespace that might be present
        headers = list(map(str.strip, next(old_csv_reader)))
        # Skip header row in the second csv
        next(new_csv_reader)

        cols_to_diff = []
        stats_diff_fieldnames = ['routeID']

        # Collect indexes of cols we're going to generate diff stats of and
        # generate fieldnames for stats diff
        for col in STATS_TO_DIFF:
            cols_to_diff.append(headers.index(col))
            # each field generates the following field names in the diff:
            # - <field name>_old
            # - <field name>_new
            # - <field name>_diff
            # - <field name>_%diff_
            stats_diff_fieldnames.append(f'{col}_old')
            stats_diff_fieldnames.append(f'{col}_new')
            stats_diff_fieldnames.append(f'{col}_diff')
            stats_diff_fieldnames.append(f'{col}_%diff')

        csv_writer = csv.writer(output_csv)
        csv_writer.writerow(stats_diff_fieldnames)

        route_num = 1
        # Assume same number of rows in both csv
        for old_row, new_row in zip(old_csv_reader, new_csv_reader):
            diff_row = []
            diff_row.append(route_num)
            for col_index in cols_to_diff:
                # Treat everything as float
                old_stat, new_stat = (float(old_row[col_index]),
                                      float(new_row[col_index]))
                diff = old_stat - new_stat
                pct_diff = diff/old_stat * 100
                diff_row.append(old_stat)
                diff_row.append(new_stat)
                diff_row.append(f'{diff}')
                diff_row.append(f'{pct_diff:.2f}')

            csv_writer.writerow(diff_row)
            route_num += 1
        print(f'Combined statistics generated: {output_file}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
                description='Compare 2 RAD statistics and '
                            'write output as a csv')
    parser.add_argument('old_stats_file', help='Old statistics.csv')
    parser.add_argument('new_stats_file', help='New statistics.csv')
    parser.add_argument('output_file', help='Output CSV filename')
    args = parser.parse_args()
    main(args.old_stats_file, args.new_stats_file, args.output_file)
