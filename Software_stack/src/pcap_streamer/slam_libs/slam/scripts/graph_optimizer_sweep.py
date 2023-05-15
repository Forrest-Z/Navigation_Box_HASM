#!/usr/bin/python3
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

import argparse
from csv import DictReader as csv_reader
from glob import glob
import matplotlib.pyplot as plt
import os

from core import Executor, get_files_list


def parse_percentages(string):
    return [float(p.strip()) for p in string.split(',')]


def run_optimizer(arguments, inlier_percentages):
    executor = Executor(arguments.applications_path)
    for perc in inlier_percentages:
        executor.run([
            'graph_optimizer',
            f'-in_files_estimates={arguments.in_file_pose}',
            f'-out_directory_ba_graph={arguments.out_dir}',
            '-output_mahalanobis_distances',
            '-double_pass',
            '-mahalanobis_threshold=-1.',
            f'-inlier_percentage={perc}',
            '-logtostderr',
            '-v=1'])


def read_file(file_name):
    data = []
    with open(file_name, 'r') as file_handle:
        reader = csv_reader(file_handle, delimiter = ';')
        for line in reader:
            data.append(float(line['mahalanobis_distance']))
    return data


def read_files(folder_name):
    file_names = sorted(glob(os.path.join(folder_name, "*[0-9].csv")))
    assert len(file_names) > 0, "No csv files found."

    data = list()
    for file_name in file_names:
        data.append((float(file_name[-13:-4]), sorted(read_file(file_name))))
    return data


def make_plot(data, filename, normalize = False):
    plt.figure()
    for _, values in data:
        size = len(values)
        x = range(size)
        if normalize:
            x = [i / (size-1) for i in x]
        assert(len(x) == len(values))
        plt.plot(x, values)
    plt.legend([d[0] for d in data])
    plt.title("Mahalanobis distance of loop edges after double pass\nSorted distance after filtering with different inlier percentages")
    plt.xlabel("Counter of sorted Mahalanobis distances")
    plt.ylabel("Mahalanobis distance")
    plt.savefig(filename)


def main(in_args=None):
    # Define and parse command line parameters
    parser = argparse.ArgumentParser()
    parser.add_argument("--in_file_pose", type=str,
                        help="Path to the input pose file containing the problem to be optimized.", required=True)
    parser.add_argument("--out_dir", type=str,
                        help="Path to the output directory.", required=True)
    parser.add_argument("--inlier_percentages", type=str,
                        help="Path of SLAM executables used in the script",
                        default="50, 55, 60, 65, 70, 75, 80, 85, 90, 95")
    parser.add_argument("--applications_path", type=str,
                        help="Path of SLAM executables used in the script",
                        default=os.path.dirname(os.path.abspath(__file__)))
    args = parser.parse_args(in_args)

    inlier_percentages = parse_percentages(args.inlier_percentages)
    run_optimizer(args, inlier_percentages)
    data = list(reversed(read_files(args.out_dir)))
    make_plot(data, os.path.join(args.out_dir, 'Filtering.png'))


if __name__ == "__main__":
    main()
