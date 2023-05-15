#!/usr/bin/python3
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

""" Pose Benchmarking Pipeline

This script can be used to benchmarking two .pose files with each other.

It will find the delta between the two pose files and generate a pose_difference.csv in the output directory.
This file will be used to generate two plots, one for the absolute and one for the relative pose errors.
It will also output the found metrics to the console.

The plots can be used to determine how well both .pose files match, and if there are any issues that have to be resolved.

The plots are generated using the benchmark_poses_visualizer.py script

It is meant to be used as a standalone application:
Example:
    Either using python:
        $ python benchmark_poses_pipeline.py [ARGUMENTS]

    Or if set as executable (using chmod +x ./benchmark_poses_pipeline.py):
        $ ./benchmark_poses_pipeline.py [ARGUMENTS]

Required Arguments:             Type    Purpose
    --in_file_reference         [PATH]  Path to the .pose file that serves as reference (ground truth).
    --in_file_benchmarking      [PATH]  Path to the .pose file to be benchmarked.
    --out_dir                   [PATH]  Path to the output directory.

Note:
    * Requires matplotlib to be installed
    * Utilizes the pose_benchmarking app
    * Only supports directly comparing two .pose files
    * The yaw plots and printed values will be the same for both relative and absolute
"""

import argparse
import os

from nie_utils import arg_checks, pipeline_tools
from benchmark_poses_visualizer import visualize_benchmarking_results

def benchmark_poses():
    # Define and parse command line parameters
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--in_file_reference",
        type=arg_checks.file_exists,
        help="Path to the input reference .pose file.",
        required=True)
    parser.add_argument(
        "--in_file_benchmarking",
        type=arg_checks.file_exists,
        help="Path to the input benchmarking .pose file.",
        required=True)
    parser.add_argument(
        "--out_dir",
        type=arg_checks.dir_exists,
        help="Path to the output directory.",
        required=True)
    parser.add_argument("--applications_path",
                        type=arg_checks.dir_exists,
                        help="Path of SLAM executables used in the script",
                        default=os.path.dirname(os.path.abspath(__file__)))
    args = parser.parse_args()

    executor = pipeline_tools.Executor(args.applications_path)

    input_benchmark_stem = os.path.splitext(os.path.basename(args.in_file_benchmarking))[0]
    benchmark_output_prefix = os.path.join(args.out_dir, input_benchmark_stem)
    csv_filename = benchmark_output_prefix + "_pose_difference.csv"

    print("=== Running pose benchmarking ===")
    executor.run([
        'pose_benchmarker',
        f'-reference_file={args.in_file_reference}',
        f'-benchmark_file={args.in_file_benchmarking}',
        f'-out_file_prefix={benchmark_output_prefix}'])

    print("=== Reading from differences csv ===")
    visualize_benchmarking_results(csv_filename)


if __name__ == '__main__':
    benchmark_poses()
