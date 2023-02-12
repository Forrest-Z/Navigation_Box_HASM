#!/usr/bin/python3
# Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

# This script uses the gps information and combines it with the lidar data to
# result an improved gps trajectory and corresponding las files.
# Inputs:
#   - PosT file containing the trajectory data
#   - A folder that contains the lidar data in the form of pcap files
#   - The lidar intrinsics and extrinsics file
# Rough processing steps:
#   - lidar odometry using loam
#   - loop closer using icp
#   - graph optimization combining gps, lidar odometry and loop closure
#     information
#   - las creator to create las files using lidar data and optimized trajectory
# Outputs:
#   - Csv file containing the poses of the optimized trajectory
#   - A folder that contains the lidar data in the form of las files based on
#     the optimized trajectory

import argparse
from glob import glob
import os
import shutil
from pathlib import Path

from nie_utils import arg_checks, pipeline_tools


# TODO: Some improvements in the pipeline script by the car team can be reused
# here. When the python shared library is in place, then this common
# functionality can be put there and reused elsewhere.


# Check if the argument is a list of pairs of strings
def check_is_list_of_string_pairs(pairs):
    error_message = "Expecting argument to be a list of string pairs."
    if isinstance(pairs, list):
        for pair in pairs:
            if isinstance(pair, list) and len(pair) == 2:
                if not all(isinstance(elem, str) for elem in pair):
                    raise Exception(error_message)
            else:
                raise Exception(error_message)
    else:
        raise Exception(error_message)


def concat_files(executor, directory, pairs):
    check_is_list_of_string_pairs(pairs)

    pairs_size = len(pairs)
    concatenated_files = [glob(os.path.join(directory, pair[0])) for pair in
                          pairs]
    output_files = [pair[1] for pair in pairs]

    num_files = len(concatenated_files[0])
    if pairs_size > 1:
        for i, files in enumerate(concatenated_files[1:]):
            if len(files) != num_files:
                raise Exception(
                    f"Expecting to find just as many files for every expression, which is not the case for {pairs[0][0]} and {pairs[i][0]}.")

    if num_files > 1:
        # Very important to sort as the trajectories are chronological and so are
        # the pose files. When they will be combined/concatenated, then also the
        # final pose file will be chronologically ordered. This is required for
        # reading the las files (inside the point filter function to remove the
        # ground plane).
        [files.sort() for files in concatenated_files]
        executor.run([
            'pose_tool',
            '-concat',
            f'-in_paths={",".join([file for files in concatenated_files for file in files])}',
            f'-out_paths={",".join(output_files)}'])
    elif num_files == 1:
        for in_files, out_file in zip(concatenated_files, output_files):
            shutil.copy2(in_files[0], out_file)


def main(in_args=None):
    # Define and parse command line parameters
    parser = argparse.ArgumentParser()
    parser.add_argument("--in_file_post",
                        type=arg_checks.file_exists,
                        help="Path to the input PosT file.", required=True)
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
    parser.add_argument("--debug_level", type=int,
                        help="The level of debug information to output. 0 = Nothing; "
                             "1 = Input of the Viewer Apps; 2 = All the Intermediate Files.",
                        default=0)
    parser.add_argument("--applications_path",
                        type=arg_checks.dir_exists,
                        help="Path of SLAM executables used in the script",
                        default=os.path.dirname(os.path.abspath(__file__)))
    args = parser.parse_args(in_args)

    executor = pipeline_tools.Executor(args.applications_path)

    # Base output folders
    out_dir_int = os.path.join(args.out_dir, "intermediate")
    out_dir_debug = os.path.join(args.out_dir, "debug")

    # Intermediate output
    out_dir_int_trajectory = os.path.join(out_dir_int, "trajectory")
    single_trajectory_file = os.path.join(out_dir_int, 'single_pcap.txt')
    gps_pose = os.path.join(out_dir_int, "gps.pose")
    gps_stop = os.path.join(out_dir_int,
                            "gps.stop")  # csv file with list of intervals when the vehicle was not moving
    lo_pose = os.path.join(out_dir_int, "lo.pose")
    lo_abs_pose = os.path.join(out_dir_int, "lo_abs.pose")

    cart_bbox_bbox = os.path.join(out_dir_int, "cart_bbox.bbox")
    cart_bbox_pose = os.path.join(out_dir_int, "cart_bbox.pose")
    cart_bbox_iref = os.path.join(out_dir_int, "cart_bbox.iref")
    orien_bbox_bbox = os.path.join(out_dir_int, "orien_bbox.bbox")
    orien_bbox_pose = os.path.join(out_dir_int, "orien_bbox.pose")
    orien_bbox_iref = os.path.join(out_dir_int, "orien_bbox.iref")
    cart_bbox_abs_pose = os.path.join(out_dir_int, "cart_bbox_abs.pose")
    orien_bbox_abs_pose = os.path.join(out_dir_int, "orien_bbox_abs.pose")

    loops_pose = os.path.join(out_dir_int, "loops.pose")
    loops_filtered_pose = os.path.join(out_dir_int, "loops_filtered.pose")

    graph_nodes_csv = os.path.join(out_dir_int, "graph_nodes.csv")
    bbox_loops_pose = os.path.join(out_dir_int, "bbox_loops.pose")
    lo_loops_pose = os.path.join(out_dir_int, "lo_loops.pose")
    partial_problem_pose = os.path.join(out_dir_int, "partial_problem.pose")
    ba_pose = os.path.join(
        out_dir_int, "ba.pose")  # Specified in graph optimizer app

    # User-facing output
    out_dir_las = os.path.join(args.out_dir, "las")
    out_dir_lo = os.path.join(args.out_dir, "lo")
    lo_iref = os.path.join(args.out_dir, "lo.iref")
    full_problem_pose = os.path.join(args.out_dir, "full_problem.pose")
    ba_filtered_pose = os.path.join(args.out_dir, "ba_filtered.pose")
    slam_iref = os.path.join(args.out_dir, "slam.iref")
    slam_trace_csv = os.path.join(
        args.out_dir,
        Path(args.in_file_post).with_suffix(".csv").name)

    # Create output subfolders
    os.makedirs(out_dir_int, exist_ok=True)
    os.makedirs(out_dir_lo, exist_ok=True)
    os.makedirs(out_dir_int_trajectory, exist_ok=True)
    os.makedirs(out_dir_las, exist_ok=True)
    os.makedirs(out_dir_debug, exist_ok=True)

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
        f'-in_lidar_intrinsics_file={args.in_file_calib_intr}',
        f'-out_dir={out_dir_int_trajectory}',
        f'-in_stationary_intervals={gps_stop}'])

    print("=== Determine lidar odometry and create las files ===")
    # The directory also contains which pcaps have been skipped. So only reading files that have a sequence number in
    # the filename.
    trajectory_files = glob(os.path.join(out_dir_int_trajectory, "*[0-9].txt"))
    print(f"Identified {len(trajectory_files)} trajectories.")
    trajectory_files.sort()
    for trajectory_file in trajectory_files:
        seq_nr = os.path.splitext(trajectory_file)[0][-6:]
        base = "traject_seq_" + seq_nr
        full_base = os.path.join(out_dir_lo, base)  # path + base
        print(f"== Processing traject '{base}' ==")
        rel_pose = full_base + '.pose'
        abs_pose = full_base + '_abs.pose'

        print("= Calculating odometry =")
        executor.run([
            'lidar_odometry_estimator',
            f'-in_lidar_intrinsics_file={args.in_file_calib_intr}',
            f'-in_lidar_extrinsics_file={args.in_file_calib_extr}',
            f'-in_source_list_file={trajectory_file}',
            f'-in_stationary_intervals={gps_stop}',
            f'-output_dir={out_dir_lo}',
            f'-output_prefix={base}'])

        print("= Converting relative odometry to absolute =")
        executor.run([
            'loam_pose_rel_to_abs',
            f'-in_file_pose_gps={gps_pose}',
            f'-in_file_pose_odom_rel={rel_pose}',
            f'-out_file_pose_odom_abs={abs_pose}'])

    print(
        "=== Running pose tool to concatenate the relative odometry of the trajectories ===")
    concat_files(executor, out_dir_lo, [["*[0-9].pose", lo_pose],
                                        ["*[0-9].iref", lo_iref]])

    print(
        "=== Running pose tool to concatenate the absolute odometry of the trajectories ===")
    concat_files(executor, out_dir_lo, [["*[0-9]_abs.pose", lo_abs_pose]])

    print(
        "=== Running pose tool to concatenate the cartesian bounding boxes of trajectories ===")
    concat_files(executor, out_dir_lo,
                 [["*_cart_bbox.bbox", cart_bbox_bbox],
                  ["*_cart_bbox.pose", cart_bbox_pose],
                  ["*_cart_bbox.iref",
                   cart_bbox_iref]])

    print(
        "=== Running pose tool to concatenate the oriented bounding boxes of trajectories ===")
    concat_files(executor, out_dir_lo,
                 [["*_orien_bbox.bbox", orien_bbox_bbox],
                  ["*_orien_bbox.pose", orien_bbox_pose],
                  ["*_orien_bbox.iref", orien_bbox_iref]])

    print("= Converting relative cartesian bounding boxes to absolute =")
    executor.run([
        'bbox_pose_rel_to_abs',
        f'-in_file_pose_gps={gps_pose}',
        f'-in_file_pose_odom={lo_pose}',
        f'-in_file_pose_bbox_odom={cart_bbox_pose}',
        f'-out_file_pose_bbox_gps={cart_bbox_abs_pose}'])

    print("= Converting relative oriented bounding boxes to absolute =")
    executor.run([
        'bbox_pose_rel_to_abs',
        f'-in_file_pose_gps={gps_pose}',
        f'-in_file_pose_odom={lo_pose}',
        f'-in_file_pose_bbox_odom={orien_bbox_pose}',
        f'-out_file_pose_bbox_gps={orien_bbox_abs_pose}'])

    print("=== Running loop finder ===")
    # Cartesian bounding boxes are used as the naive edge transformation should
    # be based on these coordinates (as they are also used for the las
    # generation).
    executor.run([
        'loop_finder',
        f'-in_file={cart_bbox_abs_pose}',
        f'-out_file={loops_pose}',
        '-max_distance=250',
        '-min_time=60'])

    print("=== Running loop filter ===")
    executor.run([
        'lidar_loop_filter',
        f'-in_file_loops={loops_pose}',
        f'-in_file_bbox_bbox={orien_bbox_bbox}',
        f'-in_file_bbox_pose={orien_bbox_abs_pose}',
        f'-in_file_bbox_iref={orien_bbox_iref}',
        f'-in_file_trace_pose={lo_abs_pose}',
        f'-in_file_trace_iref={lo_iref}',
        f'-out_file_loops={loops_filtered_pose}'])

    print("=== Running pose tool to identify graph nodes ===")
    executor.run([
        'pose_tool',
        '-subsample',
        f'-in_paths={lo_abs_pose}',
        '-vertex_distance_threshold=4',
        f'-out_paths={graph_nodes_csv}'])

    print("=== Running loop closer ===")
    command = [
        'lidar_loop_closer',
        f'-in_file_pose_trace={lo_pose}',
        f'-in_file_pose_edges={loops_filtered_pose}',
        f'-in_file_bbox_bbox={orien_bbox_bbox}',
        f'-in_file_pose_bbox={orien_bbox_pose}',
        f'-in_file_iref_bbox={orien_bbox_iref}',
        f'-out_file_pose_edges={bbox_loops_pose}']
    if args.debug_level >= 2:
        command += [
            '-debug',
            f'-out_dir_debug={out_dir_debug}']
    executor.run(command)

    print(
        "=== Running tool to convert the loop closer edges to loam trace edges ===")
    executor.run([
        'bbox_to_pose_edge_converter',
        f'-in_file_bbox_loops={bbox_loops_pose}',
        f'-in_file_aa_bbox_pose={cart_bbox_pose}',
        f'-in_file_aa_bbox_iref={cart_bbox_iref}',
        f'-in_file_oa_bbox_pose={orien_bbox_pose}',
        f'-in_file_oa_bbox_bbox={orien_bbox_bbox}',
        f'-in_file_trace_pose={lo_pose}',
        f'-in_file_trace_iref={lo_iref}',
        f'-in_file_trace_ids={graph_nodes_csv}',
        f'-out_file_trace_loops={lo_loops_pose}'])

    print("=== Running pose tool to merge the loam trace with the edges ===")
    executor.run([
        'pose_tool',
        '-merge',
        f'-in_paths={lo_abs_pose},{lo_loops_pose}',
        f'-out_paths={partial_problem_pose}'])

    print("=== Running graph builder ===")
    executor.run([
        'pgo_problem_builder',
        f'-in_file_gps_pose={gps_pose}',
        f'-in_file_lo_pose={partial_problem_pose}',
        f'-in_file_pose_ids_csv={graph_nodes_csv}',
        f'-out_file_pgo_pose={full_problem_pose}'])

    print("=== Running graph optimizer ===")
    executor.run([
        'graph_optimizer',
        f'-in_files_estimates={full_problem_pose}',
        f'-out_directory_ba_graph={out_dir_int}',
        '-double_pass',
        '-mahalanobis_threshold=500'])

    print("=== Running pose tool to filter the optimized graph ===")
    executor.run([
        'pose_tool',
        '-filter',
        '-selection=odom',
        f'-in_paths={ba_pose}',
        f'-out_paths={ba_filtered_pose}'])

    # Output one las for every pcap file given
    print("=== Create final las files ===")

    pcaps_skipped_filename = os.path.join(out_dir_int_trajectory,
                                          'skipped.txt')
    with open(pcaps_skipped_filename, 'r') as pcaps_skipped_file:
        pcaps_skipped_lines = pcaps_skipped_file.readlines()
    pcaps_skipped = {line.strip() for line in pcaps_skipped_lines}
    pcap_files = [pcap for pcap in
                  glob(os.path.join(args.in_dir_pcap, "*.pcap")) if
                  pcap not in pcaps_skipped]
    pcap_files.sort()

    for pcap_file in pcap_files:
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
            f'-output_prefix={base}',
            '-output_iref'])

    if os.path.exists(single_trajectory_file):
        os.remove(single_trajectory_file)

    iref_files = glob(os.path.join(out_dir_las, "*.iref"))
    if len(iref_files) > 1:
        iref_files.sort()
        print("=== Running pose tool to merge the irefs ===")
        executor.run([
            'pose_tool',
            '-merge',
            f'-in_paths={",".join(iref_files)}',
            f'-out_paths={slam_iref}'])
    elif len(iref_files) == 1:
        shutil.copy2(iref_files[0], slam_iref)

    print("=== Running pose tool to convert the trajectory to csv ===")
    executor.run([
        'pose_tool',
        '-convert_to',
        '-conversion_format=csv',
        f'-in_paths={ba_filtered_pose}',
        f'-out_paths={slam_trace_csv}'])

    print("=== Running las file renamer ===")
    executor.run([
        'nie_china_las_namer',
        f'-in_file_aiim={args.in_file_post}',
        f'-in_file_iref={slam_iref}',
        f'-out_file_iref={slam_iref}'])

    # Conditional clean up
    if args.debug_level < 2:
        shutil.rmtree(out_dir_int)
        shutil.rmtree(out_dir_debug)
        for file in glob(os.path.join(out_dir_las, "*.iref")):
            os.remove(file)
    if args.debug_level < 1:
        shutil.rmtree(out_dir_lo)
        os.remove(lo_iref)
        os.remove(full_problem_pose)
        os.remove(ba_filtered_pose)
        os.remove(slam_iref)


if __name__ == '__main__':
    main()
