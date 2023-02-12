#!/usr/bin/python3
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

import argparse
import os
import subprocess

DEFAULT = "default"
KITTI = "kitti"
WEIYA = "weiya"

INTRINSICS_FILE = "intrinsics.json"

# Get the current script location
SCRIPT_PATH = os.getcwd()


def run_command(command_args):
    # Prepend the script/executable with the given bin directory
    command_args[0] = f'{os.path.join(SCRIPT_PATH, command_args[0])}'

    results = subprocess.run(command_args)
    results.check_returncode()


def main():
    # Define and parse command line parameters
    parser = argparse.ArgumentParser()
    parser.add_argument("-format", type=str, default=DEFAULT,
                        choices=[DEFAULT, KITTI, WEIYA],
                        help="Input format setting, required when input conversion is required.")
    parser.add_argument("-in_file_intrinsics", type=str, required=True,
                        help="Path to an input intrinsics file.")
    parser.add_argument("-in_dir_images", type=str, required=True,
                        help="Path to a directory containing input images.")
    parser.add_argument("-in_file_ground_truth", type=str, required=True,
                        help="Path to the ground truth poses in kitti pose format.")
    parser.add_argument("-out_dir_results", type=str, required=True,
                        help="Path to a directory where the results will be stored.")
    parser.add_argument("-out_file_name", type=str, default="",
                        help="Benchmark result files will get this name. [optional]")
    args = parser.parse_args()

    # Define google log specific parameters for all applications of the pipeline
    general_args = ['-alsologtostderr',
                    '-v', '2']

    intrinsics_file = args.in_file_intrinsics
    if args.format != DEFAULT:
        print("=== Converting the input data ===")
        intrinsics_file = os.path.join(args.out_dir_results, INTRINSICS_FILE)
        convert_input_command = []
        if args.format in [KITTI, WEIYA]:
            convert_input_command = [
                'convert_calibration.py',
                f'--in_file_intrinsics={args.in_file_intrinsics}',
                f'--in_dir_images={args.in_dir_images}',
                f'--format={args.format}',
                f'--out_file_intrinsics={intrinsics_file}'
            ]
        else:
            raise Exception(
                f"Unknown value '{args.format}' given for argument 'format'")
        run_command(convert_input_command)
        print("")

    print("=== Running visual odometry application ===")
    visual_odometry_command = [
        'visual_odometry_mono',
        f'-in_file_intrinsics={intrinsics_file}',
        f'-in_dir_images={args.in_dir_images}',
        f'-out_dir={args.out_dir_results}']
    run_command(visual_odometry_command + general_args)
    print("")

    print("=== Performing the benchmark ===")
    benchmark_command = [
        'kitti_benchmarker',
        f'-in_file_reference={args.in_file_ground_truth}',
        f'-in_file_test={os.path.join(args.out_dir_results, "poses.pose")}',
        f'-correct_scale',
        f'-out_dir_result={args.out_dir_results}']
    if args.out_file_name:
        benchmark_command.append(f'-out_file_name={args.out_file_name}')
    run_command(benchmark_command + general_args)
    print("")


if __name__ == "__main__":
    main()
