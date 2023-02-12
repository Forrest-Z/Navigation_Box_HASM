#!/usr/bin/env python3

# Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

""" Compares sets of run statistics output by analysis node

The ndt analysis node outputs csv files with the found statistics.
To compare different methods, we can look at the results of these csv files
for multiple runs over different methods.
By displaying the difference between these methods, we can then decide which one works best.

It is meant to be ran as a standalone application:
Example (assuming the runs are named master*.csv and branch_name*.csv):
        $ python compare_runs.py --reference "./master" --compare "./branch_name"
"""

import argparse
import copy
import csv
import glob
import os
from tabulate import tabulate


def read_runs(file_prefix):
    """ Reads and returns a list of csv files matching the file_prefix """
    filenames_list = glob.glob(file_prefix + '*.csv')
    runs = []
    for filename in filenames_list:
        run = []
        with open(filename) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for row in csv_reader:
                run.append(row)
        runs.append(run)
    return runs


def combine_runs(runs):
    """ Combines a list of runs with 2d string data into a single 2d table """
    num_runs = len(runs)
    num_rows = len(runs[0])
    num_columns = len(runs[0][0])

    # By default python makes a shallow copy, which is not what we want
    table = copy.deepcopy(runs[0])
    # Index starts at 1 to ignore header row
    for r in range(1, num_rows):
        # Index starts at 1 to ignore name column
        for c in range(1, num_columns):
            sum = 0.0
            for i in range(0, num_runs):
                sum += float(runs[i][r][c])
            table[r][c] = sum / num_runs
    return table


def compare_sets(A, B):
    """ Subtracts the data in set A from the data in set B while ignoring the headers and names """
    # By default python makes a shallow copy, which is not what we want
    C = copy.deepcopy(A)
    for r in range(1, len(A)):
        for c in range(1, len(A[0])):
            relative_diff = float(B[r][c] - float(A[r][c])) /(float(A[r][c])) 
            C[r][c] = relative_diff
    return C


def print_table(table):
    """ Prints the data in a fancy grid table """
    print(
        tabulate(
            table[1:],
            headers=table[0],
            tablefmt="fancy_grid",
            floatfmt=".4f"))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--reference',
        dest='reference',
        help='Runs for reference')
    parser.add_argument(
        '--compare',
        dest='compare',
        help='Runs to compare with reference')

    args = parser.parse_args()

    # Process data
    ref_data = combine_runs(read_runs(args.reference))
    comp_data = combine_runs(read_runs(args.compare))
    diff_data = compare_sets(ref_data, comp_data)

    # Print fancy tables
    print("Reference:")
    print_table(ref_data)
    print("Comparison:")
    print_table(comp_data)
    print("Relative Change:")
    print_table(diff_data)
