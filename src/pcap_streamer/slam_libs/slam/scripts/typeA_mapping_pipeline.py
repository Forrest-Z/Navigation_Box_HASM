#!/usr/bin/python3
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
'''
Script for generating .las and .pcd files from a set of pcaps and a PosT/AIIM file.
PCD conversion enables grid generation for NDT optimization.
'''
import argparse
from glob import glob
import os
import shutil

from nie_utils import arg_checks, pipeline_tools


def main(in_args=None):
    # Define and parse command line parameters
    parser = argparse.ArgumentParser()
    parser.add_argument("--in_file_post",
                        type=arg_checks.file_exists,
                        help="Path to the input PosT/AIIM file.", required=True)
    parser.add_argument("--in_file_calib_extr",
                        help="Path to the input lidar calibration extrinsics file.",
                        type=arg_checks.file_exists,
                        required=True)
    parser.add_argument("--in_file_calib_intr",
                        type=arg_checks.file_exists,
                        help="Path to the input lidar calibration intrinsics file.",
                        required=True)
    parser.add_argument("--in_dir_pcap",
                        type=arg_checks.dir_exists,
                        help="Path to a directory containing input PCAP files.",
                        required=True)
    parser.add_argument("--out_dir",
                        type=arg_checks.dir_exists,
                        help="Path to the output directory.", required=True)
    parser.add_argument("--verbosity", type=int,
                        help="The level of log messages verbosity to be used in the apps.",
                        default=3)
    parser.add_argument("--applications_path",
                        type=arg_checks.dir_exists,
                        help="Path of SLAM executables used in the script",
                        default=os.path.dirname(os.path.abspath(__file__)))
    parser.add_argument("--map_grid_size", type=float,
                        help="Size of the grid of the PCD map.",
                        default=100.0)
    parser.add_argument("--voxel_grid_distance", type=float,
                        help="Downsampling distance for output map.",
                        default=0.15)

    args = parser.parse_args(in_args)

    executor = pipeline_tools.Executor(args.applications_path)

    # Base output folders
    out_dir_int = os.path.join(args.out_dir, "intermediate")

    # Intermediate output
    out_dir_int_trajectory = os.path.join(out_dir_int, "trajectory")
    single_trajectory_file = os.path.join(out_dir_int, 'single_pcap.txt')
    gps_pose = os.path.join(args.out_dir, "gps.pose")
    gps_stop = os.path.join(args.out_dir,
                            "gps.stop")  # csv file with list of intervals when the vehicle was not moving
    # User-facing output
    out_dir_las = os.path.join(args.out_dir, "las")
    out_dir_pcd = os.path.join(args.out_dir, "pcd")

    # Create output subfolders
    os.makedirs(out_dir_int, exist_ok=True)
    os.makedirs(out_dir_int_trajectory, exist_ok=True)
    os.makedirs(out_dir_las, exist_ok=True)
    os.makedirs(out_dir_pcd, exist_ok=True)

    print("=== Running PosT converter ===")
    executor.run([
        'pose_tool',
        '-convert_from',
        '-conversion_format=post',
        f'-in_paths={args.in_file_post}',
        f'-out_paths={gps_pose},{gps_stop}'])

    print("=== Running pcap trajectory tool ===")
    executor.run([
        'pcap_trajectory_tool',
        f'-in_dir={args.in_dir_pcap}',
        f'-lidar_intrinsics_file={args.in_file_calib_intr}',
        f'-out_dir={out_dir_int_trajectory}',
        f'-in_stationary_intervals={gps_stop}'])

    pcaps_skipped_filename = os.path.join(out_dir_int_trajectory,
                                          'skipped.txt')
    with open(pcaps_skipped_filename, 'r') as pcaps_skipped_file:
        pcaps_skipped_lines = pcaps_skipped_file.readlines()
    pcaps_skipped = {line.strip() for line in pcaps_skipped_lines}
    pcap_files = [pcap for pcap in
                  glob(os.path.join(args.in_dir_pcap, "*.pcap")) if
                  pcap not in pcaps_skipped]
    pcap_files.sort()

    # Output one las for every pcap file given
    print("=== Creating LAS files ===")
    for pcap_file in pcap_files:
        print(f"== Processing pcap file '{os.path.basename(pcap_file)}' ==")
        with open(single_trajectory_file, 'w') as trajectory_file:
            trajectory_file.write(pcap_file)
        base = os.path.splitext(os.path.basename(pcap_file))[0]
        executor.run([
            'las_creator',
            f'-lidar_extrinsics_file={args.in_file_calib_extr}',
            f'-lidar_intrinsics_file={args.in_file_calib_intr}',
            f'-pose_file={gps_pose}',
            f'-source_list_file={single_trajectory_file}',
            '-traveled_distance=100000',
            f'-output_dir={out_dir_las}',
            f'-output_prefix={base}',
            '-output_iref',
            f'-filter_ground_plane={True}',
            f'-v={args.verbosity}'])
    print(f"LAS files generated in: {out_dir_las}")

    print("=== Creating PCD files ===")
    executor.run([
        'localization_pcl_converter',
        f'-dir_in={out_dir_las}',
        f'-pcd_out={out_dir_pcd}',
        f'-map_grid_size={args.map_grid_size}',
        f'-voxel_grid_distance={args.voxel_grid_distance}',
        f'-v={args.verbosity}'])

    print(f"PCD files generated in: {out_dir_pcd}")
    # Clean up
    if os.path.exists(single_trajectory_file):
        os.remove(single_trajectory_file)
    for file in glob(os.path.join(out_dir_las, "*.iref")):
        os.remove(file)
    shutil.rmtree(out_dir_int)


if __name__ == '__main__':
    main()
