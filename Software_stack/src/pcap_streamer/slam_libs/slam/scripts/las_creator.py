#!/usr/bin/python3
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

import argparse
from glob import glob
import multiprocessing
import os
from pathlib import Path

# local import
from nie_utils.pipeline_tools import Executor
from nie_utils import arg_checks

""" LAS creator

Will read all .pcap files from a directory and generate a directory full of .las files with corresponding .iref files.

Call as:

las_creator.py <arguments>

Argument                       Example

  --in_file_pose        =      "/data/aiim/lidar_slam_pipeline_results/latest/shanghai_190928/intermediate/gps.pose" 
  --in_file_calib_extr  =      "/data/aiim/CHINA_20191205/shanghai/1002-1-014-190928/HDLCalPara-14-190717.EP" 
  --in_file_calib_intr  =      "/data/aiim/lidar_calibration_files/HDL-32E.xml" 
  --in_dir_pcap         =      "/data/aiim/CHINA_20191205/shanghai/1002-1-014-190928/Pointcloud" 
  --out_dir_las         =      "/data/aiim/CHINA_20191205/shanghai/1002-1-014-190928/las"
"""

def las_creator(args, pcap_file):
    print(f"== Processing pcap file '{os.path.basename(pcap_file)}' ==")

    executor = Executor(args.applications_path)
    single_trajectory_file = Path(args.out_dir_las) / Path(pcap_file).with_suffix(".txt").name

    with open(single_trajectory_file, 'w') as trajectory_file:
        trajectory_file.write(pcap_file)
    base = os.path.splitext(os.path.basename(pcap_file))[0]
    executor.run([
        'las_creator',
        f'-lidar_extrinsics_file={args.in_file_calib_extr}',
        f'-lidar_intrinsics_file={args.in_file_calib_intr}',
        f'-pose_file={args.in_file_pose}',
        f'-source_list_file={single_trajectory_file}',
        '-traveled_distance=100000',
        f'-output_dir={args.out_dir_las}',
        f'-output_prefix={base}'])

    if os.path.exists(single_trajectory_file):
        os.remove(single_trajectory_file)

def main(in_args=None):
    # Define and parse command line parameters
    parser = argparse.ArgumentParser()
    parser.add_argument("--in_file_pose", type=str,
                        help="Path to the input pose file.", required=True)
    parser.add_argument("--in_file_calib_extr",
                        type=arg_checks.file_exists,
                        help="Path to the input lidar calibration extrinsics file.",
                        required=True)
    parser.add_argument("--in_file_calib_intr",
                        type=arg_checks.file_exists,
                        help="Path to the input lidar calibration intrinsics file.",
                        required=True)
    parser.add_argument("--in_dir_pcap",
                        type=arg_checks.dir_exists,
                        help="Path to a directory containing input PCAP files.",
                        required=True)
    parser.add_argument("--out_dir_las",
                        type=arg_checks.dir_exists,
                        help="Path to the output directory.", required=True)
    parser.add_argument("--debug_level", type=int,
                        help="The level of debug information to output. 0 = Nothing; "
                             "1 = Input of the Viewer Apps; 2 = All the Intermediate Files.",
                        default=0)
    parser.add_argument("--applications_path",
                        type=arg_checks.dir_exists,
                        help="Path of SLAM executables used in the script",
                        default=os.path.dirname(os.path.abspath(__file__)))
    args = parser.parse_args(in_args)

    # Output one las for every pcap file
    print(f"=== Creating las files with {multiprocessing.cpu_count()} cpus ===")

    pcap_files = glob(os.path.join(args.in_dir_pcap, '*.pcap'))
    pcap_files.sort()

    pool = multiprocessing.Pool(multiprocessing.cpu_count())

    for pcap_file in pcap_files:
        pool.apply_async(las_creator, args=(args, pcap_file))

    pool.close()
    pool.join()

if __name__ == '__main__':
    main()
