#!/usr/bin/env python3

import argparse
import csv
import sys


STATS_TO_DIFF = ['#Passes', 'runtime', 'trip time', 'length', '#Maneuvers', 'elapsedCostSeconds', 'elapsedCostCost']


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

        result_col_index = headers.index('result')
        # Collect indexes of cols we're going to generate diff stats of and
        # generate fieldnames for stats diff
        for col in STATS_TO_DIFF:
            cols_to_diff.append(headers.index(col))
            # each field generates the following field names in the diff:
            # - <field name>_old
            # - <field name>_new
            # - <field name>_diff
            # - <field name>_%diff_
            stats_diff_fieldnames.append('{}_old'.format(col))
            stats_diff_fieldnames.append('{}_new'.format(col))
            stats_diff_fieldnames.append('{}_diff'.format(col))
            stats_diff_fieldnames.append('{}_%diff'.format(col))

        csv_writer = csv.writer(output_csv,quoting=csv.QUOTE_ALL)
        csv_writer.writerow(stats_diff_fieldnames)

        route_num = 1
        failed_routes = 0
        mismatched_result = 0
        # Assume same number of rows in both csv
        for old_row, new_row in zip(old_csv_reader, new_csv_reader):
            diff_row = []
            diff_row.append(route_num)
            route_num += 1

            # Compare status of old & new stat
            old_result = old_row[result_col_index]
            new_result = new_row[result_col_index]
            # skip failed routes
            if old_result != new_result:
                print('WARNING: Result mismatch for route #{}! '
                      'Old result: {}, new result: {}'.format(route_num,
                                                              old_result,
                                                              new_result))
                mismatched_result += 1
                continue
            elif old_result == 'fail_no_route':
                failed_routes += 1
                continue

            for col_index in cols_to_diff:
                # Treat everything as float
                old_stat, new_stat = (float(old_row[col_index]),
                                      float(new_row[col_index]))
                # if old stat is 0, add small epsilon value to avoid divide by
                # zero
                # Should we skip these rows instead?
                if old_stat == 0:
                    old_stat = sys.float_info.epsilon
                diff = new_stat - old_stat
                pct_diff = diff/old_stat * 100
                diff_row.append(old_stat)
                diff_row.append(new_stat)
                diff_row.append('{}'.format(diff))
                diff_row.append('{:.2f}'.format(pct_diff))

            csv_writer.writerow(diff_row)
        print('Found {} failed routes'.format(failed_routes))
        print('Found {} mismatched results'.format(mismatched_result))
        print('Combined statistics generated: {}'.format(output_file))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
                description='Compare 2 RAD statistics and '
                            'write output as a csv')
    parser.add_argument('old_stats_file', help='Old statistics.csv')
    parser.add_argument('new_stats_file', help='New statistics.csv')
    parser.add_argument('output_file', help='Output CSV filename')
    args = parser.parse_args()
    main(args.old_stats_file, args.new_stats_file, args.output_file)
