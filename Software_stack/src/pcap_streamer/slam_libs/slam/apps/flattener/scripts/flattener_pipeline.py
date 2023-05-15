#!/usr/bin/python3
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

import argparse
from glob import glob
import os

from nie_utils import arg_checks, pipeline_tools


def main(in_args=None):
    # Define and parse command line parameters
    parser = argparse.ArgumentParser()
    parser.add_argument("--in_file_post", type=arg_checks.file_exists,
                        help="Path to the input PosT file.", required=True)
    parser.add_argument("--in_file_post_classic_mode", dest='in_file_post_classic_mode', action='store_true',
                        help="Flag to indicate that the PosT file format is expected.")
    parser.set_defaults(in_file_post_classic_mode=False)
    parser.add_argument("--in_file_calib_extr", type=arg_checks.file_exists,
                        help="Path to the input lidar calibration extrinsics file.",
                        required=True)
    parser.add_argument("--in_file_calib_intr", type=arg_checks.file_exists,
                        help="Path to the input lidar calibration intrinsics file.",
                        required=True)
    parser.add_argument("--in_dir_pcap", type=arg_checks.dir_exists,
                        help="Path to a directory containing input PCAP files.",
                        required=True)
    parser.add_argument("--out_dir", type=arg_checks.dir_exists,
                        help="Path to the output directory.", required=True)
    parser.add_argument("--applications_path", type=arg_checks.dir_exists,
                        help="Path of SLAM executables used in the script",
                        default=os.path.dirname(os.path.abspath(__file__)))
    args = parser.parse_args(in_args)

    executor = pipeline_tools.Executor(args.applications_path)

    # Base output folders
    out_dir_lo = os.path.join(args.out_dir, "lo")
    out_dir_las = os.path.join(args.out_dir, "las")
    out_dir_int = os.path.join(args.out_dir, "intermediate")
    out_dir_int_trajectory = os.path.join(out_dir_int, "trajectory")

    gps_pose = os.path.join(out_dir_int, "gps.pose")
    # csv file with list of intervals when the vehicle was not moving
    gps_stop = os.path.join(out_dir_int, "gps.stop")
    gps_pose_resampled = os.path.join(out_dir_int, "gps_resampled.pose")

    loam_jump_tracks_txt = os.path.join(out_dir_int, "loam_jump_tracks.txt")
    fuse_problem_pose = os.path.join(out_dir_int, "fuse_problem_pose.pose")
    ba_pose = os.path.join(out_dir_int, "ba.pose")  # Specified in graph optimizer app
    ba_filtered_pose = os.path.join(args.out_dir, "ba_filtered.pose")
    single_trajectory_file = os.path.join(out_dir_int, 'single_pcap.txt')

    # Create output subfolders
    os.makedirs(out_dir_lo, exist_ok=True)
    os.makedirs(out_dir_las, exist_ok=True)
    os.makedirs(out_dir_int, exist_ok=True)
    os.makedirs(out_dir_int_trajectory, exist_ok=True)

    print("=== Running PosT converter ===")
    executor.run([
        'pose_tool',
        '-convert_from',
        '-conversion_format=post',
        f'-classic_post={args.in_file_post_classic_mode}',
        f'-in_paths={args.in_file_post}',
        f'-out_paths={gps_pose},{gps_stop}'])

    print("=== Running pose resampler ===")
    executor.run([
        'pose_tool',
        '-resample',
        '-sample_interval=0.1',
        f'-in_paths={gps_pose}',
        f'-out_paths={gps_pose_resampled}'])

    print("=== Running trajectory tool ===")
    executor.run([
        'stop_trajectory_tool',
        f'-in_dir={args.in_dir_pcap}',
        f'-in_stationary_intervals={gps_stop}',
        f'-lidar_intrinsics_file={args.in_file_calib_intr}',
        f'-out_dir={out_dir_int_trajectory}'])

    print("=== Determine lidar odometry ===")
    # The directory also contains which pcaps have been skipped. So only reading files that have a sequence number in
    # the filename.
    trajectory_files = glob(os.path.join(out_dir_int_trajectory, "*[0-9].txt"))
    print(f"Identified {len(trajectory_files)} trajectories.")
    trajectory_files.sort()
    for trajectory_file in trajectory_files:
        seq_nr = os.path.splitext(trajectory_file)[0][-6:]
        base = "traject_seq_" + seq_nr
        print(f"== Processing traject '{base}' ==")

        print("= Calculating odometry =")
        executor.run([
            'lidar_odometry_estimator',
            f'-in_lidar_intrinsics_file={args.in_file_calib_intr}',
            f'-in_source_list_file={trajectory_file}',
            f'-in_stationary_intervals={gps_stop}',
            f'-output_dir={out_dir_lo}',
            f'-output_prefix={base}'])

    lo_pose_files = glob(os.path.join(out_dir_lo, "*[0-9].pose"))
    with open(loam_jump_tracks_txt, 'w') as out_file:
        out_file.writelines([f + '\n' for f in lo_pose_files])

    print("=== Running fuse problem builder ===")
    executor.run([
        'fuse_problem_builder',
        f'-in_file_lidar_extrinsics={args.in_file_calib_extr}',
        f'-in_file_jump_tracks={loam_jump_tracks_txt}',
        f'-in_file_gps_pose={gps_pose_resampled}',
        f'-in_stationary_intervals={gps_stop}',
        f'-out_file_fuse_problem_pose={fuse_problem_pose}'])

    print("=== Running graph optimizer ===")
    executor.run([
        'graph_optimizer',
        f'-in_files_estimates={fuse_problem_pose}',
        f'-out_directory_ba_graph={out_dir_int}'])

    print("=== Running pose tool to filter the optimized graph ===")
    executor.run([
        'pose_tool',
        '-filter',
        '-selection=gps',
        f'-in_paths={ba_pose}',
        f'-out_paths={ba_filtered_pose}'])

    # Output one las for every pcap file given
    print("=== Create final las files ===")
    for pcap_file in sorted(glob(os.path.join(args.in_dir_pcap, "*.pcap"))):
        print(f"== Processing pcap file '{os.path.basename(pcap_file)}' ==")
        with open(single_trajectory_file, 'w') as trajectory_file:
            trajectory_file.write(pcap_file)
        base = os.path.splitext(os.path.basename(pcap_file))[0]
        executor.run([
            'las_creator',
            f'-lidar_extrinsics_file={args.in_file_calib_extr}',
            f'-lidar_intrinsics_file={args.in_file_calib_intr}',
            f'-pose_file={ba_filtered_pose}',
            f'-source_list_file={single_trajectory_file}',
            '-traveled_distance=100000',
            f'-output_dir={out_dir_las}',
            f'-output_prefix={base}'])


if __name__ == "__main__":
    main()
