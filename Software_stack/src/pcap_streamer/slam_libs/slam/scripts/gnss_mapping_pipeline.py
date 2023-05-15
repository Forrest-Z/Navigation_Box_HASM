#!/usr/bin/python3
# Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
""" GNSS Mapping Pipeline

This is the main script that calls all the relevant processing applications to convert the recorded data into a localization point cloud.

It is meant to be used as a standalone application:
Example:
    Either using python:
        $ python gnss_mapping_pipeline.py [ARGUMENTS]

    Or if set as executable (using chmod +x ./gnss_post_processing.py):
        $ ./gnss_mapping_pipeline.py [ARGUMENTS]

Required Arguments:             Type    Purpose
    --gnss_post_processing_py   [PATH]  Path to the gnss post processing python script.
    --in_file_gnss              [PATH]  Path to the input gnss .anpp file.
    --in_file_basestation       [PATH]  Path to the input basestation .20o file.
    --in_file_calib_extr        [PATH]  Path to the input lidar calibration extrinsics file.
    --in_file_calib_intr        [PATH]  Path to the input lidar calibration intrinsics file.
    --in_dir_pcap               [PATH]  Path to a directory containing input PCAP files.
    --out_dir                   [PATH]  Path to the output directory.

Optional Arguments:             Type    Purpose
    --downsampling_leaf_size    [FLOAT] Leaf size of the Downsampling done on output cloud.
    --grid_size                 [FLOAT] Size of the output map grid size in meters.
    --debug_level               [BOOL]  The level of debug information to output. 0 = Nothing; 1 = Input of the Viewer Apps; 2 = All the Intermediate Files.
    --applications_path         [PATH]  The root folder from where binary executables can be found.

    gnss_pipeline()                         The main pipeline script that implements the gnss mapping.
"""

import argparse
import glob
import os
from pathlib import Path
import subprocess
import shutil
import re

from nie_utils import arg_checks, pipeline_tools


def gnss_pipeline():
    # Define and parse command line parameters
    parser = argparse.ArgumentParser()
    # Note: We cannot use "execute_app" for the GNSS Post Processing because the script is contained in a different repository.
    # Thus, the path to this script has to be supplied.
    # Usually, it should be located in
    # {NIE_DIR}/gnss_post_processing/src/gnss_post_processing.py
    parser.add_argument(
        "--gnss_post_processing_py",
        type=arg_checks.file_exists,
        required=True,
        help="Path to the gnss post processing python script.")
    parser.add_argument(
        "--in_file_gnss",
        type=arg_checks.file_exists,
        help="Path to the input gnss .anpp file.",
        required=True)
    parser.add_argument(
        "--in_file_basestation",
        type=arg_checks.file_exists,
        help="Path to the input basestation .20o file.",
        required=True)
    parser.add_argument(
        "--in_file_calib_extr",
        type=arg_checks.file_exists,
        help="Path to the input lidar calibration extrinsics file.",
        required=True)
    parser.add_argument(
        "--in_file_calib_intr",
        type=arg_checks.file_exists,
        help="Path to the input lidar calibration intrinsics file.",
        required=True)
    parser.add_argument(
        "--in_dir_pcap",
        type=arg_checks.dir_exists,
        help="Path to a directory containing input PCAP files.",
        required=True)
    parser.add_argument(
        "--out_dir",
        type=arg_checks.dir_exists,
        help="Path to the output directory.",
        required=True)
    parser.add_argument(
        "--downsampling_leaf_size",
        type=float,
        help="Downsampling leaf size.",
        default=0.15)
    parser.add_argument(
        "--grid_size",
        type=float,
        help="Size of output map grid size in meters.",
        default=50.0)
    parser.add_argument(
        "--shift_time",
        type=int,
        help="Time shift in microseconds to be applied to the input poses.",
        default=0.0)
    parser.add_argument(
        "--debug_level",
        type=int,
        help="The level of debug information to output. 0 = Nothing; "
        "1 = Input of the Viewer Apps; 2 = All the Intermediate Files.",
        default=0)
    parser.add_argument("--applications_path",
                        type=arg_checks.dir_exists,
                        help="Path of SLAM executables used in the script",
                        default=os.path.dirname(os.path.abspath(__file__)))
    args = parser.parse_args()

    executor = pipeline_tools.Executor(args.applications_path)

    # Base output folders
    out_dir_int = os.path.join(args.out_dir, "intermediate")

    # Intermediate output
    out_dir_kinematica = os.path.join(out_dir_int, "gnss_pose_optimized")
    out_file_kinematica = os.path.join(out_dir_kinematica, "PostProcessed.csv")
    utm_pose = os.path.join(out_dir_int, "utm_pose.pose")
    shifted_pose = os.path.join(out_dir_int, "utm_pose_shifted.pose")
    single_trajectory_file = os.path.join(out_dir_int, 'single_pcap.txt')

    # User-facing output
    out_dir_las = os.path.join(args.out_dir, "las")
    out_dir_pcd = os.path.join(args.out_dir, "pcd")

    # Create output subfolders
    Path(out_dir_las).mkdir(parents=True, exist_ok=True)
    Path(out_dir_pcd).mkdir(parents=True, exist_ok=True)

    # Post process the gnss files
    # Please note that this costs credits, do not run twice for same input.
    print("=== Running gnss pose post processing ===")
    # Note: We cannot use execute_app here because it is in a different
    # repository.
    gnss_processing_args = [args.gnss_post_processing_py,
                            '--anpp_path',
                            f'{args.in_file_gnss}',
                            '--basestation',
                            f'{args.in_file_basestation}',
                            '--output_dir',
                            f'{out_dir_kinematica}',
                            '-v']
    subprocess.check_output(gnss_processing_args)

    # Convert Advanced Navigation .csv file into posecollection .pose files
    print("=== Running gnss pose converter ===")
    executor.run([
        'pose_tool',
        '-convert_from',
        '-conversion_format=kinematica',
        f'-in_paths={out_file_kinematica}',
        f'-out_paths={utm_pose}'])

    print("=== Shifting pose file ===")
    executor.run([
        'pose_tool',
        '-shift',
        f'-shift_time={args.shift_time}',
        f'-in_paths={utm_pose}',
        f'-out_paths={shifted_pose}'])

    # Output one las for every pcap file given
    print("=== Creating las files ===")
    # Pcap files generated by tcpdump do not have the .pcap extension.
    pcap_files = glob.glob(args.in_dir_pcap + '/*')
    for pcap_file in pcap_files:
        print(f"== Processing pcap file '{os.path.basename(pcap_file)}' ==")
        with open(single_trajectory_file, 'w') as trajectory_file:
            trajectory_file.write(pcap_file)
        base = os.path.splitext(os.path.basename(pcap_file))[0]
        executor.run([
            'las_creator',
            f'-pose_file={shifted_pose}',
            f'-source_list_file={single_trajectory_file}',
            f'-lidar_extrinsics_file={args.in_file_calib_extr}',
            f'-lidar_intrinsics_file={args.in_file_calib_intr}',
            '-traveled_distance=100000',
            '-min_ray_length=3',
            '-max_ray_length=300',
            f'-output_dir={out_dir_las}',
            f'-output_prefix={base}',
            '-lidar_id=ouster'])

    # Convert to autoware pcd format
    print("=== Converting las to autoware pcd ===")
    executor.run([
        'localization_pcl_converter',
        f'-dir_in={out_dir_las}',
        f'-pcd_out={out_dir_pcd}',
        f'-voxel_grid_distance={args.downsampling_leaf_size}',
        f'-map_grid_size={args.grid_size}'])

    print("=== Finished ===")


if __name__ == '__main__':
    gnss_pipeline()

