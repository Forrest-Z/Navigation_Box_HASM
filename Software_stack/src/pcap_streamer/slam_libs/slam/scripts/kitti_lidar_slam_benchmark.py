#!/usr/bin/python3
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

import argparse
from glob import glob
import os
import shutil
from pathlib import Path

from nie_utils import arg_checks
from nie_utils.pipeline_tools import *


def create_debug_files(executor, in_file, txt_only=False):
    print("= Creating debug pose files ===")
    name = Path(in_file)
    convert_to_txt(executor, name)
    if txt_only:
        return
    convert_to_kml(executor, name, add_suffix(name, '.kml'))
    convert_to_kml(executor, name, add_suffix(name, '.kml_rel'),
                   KmlMode.RelEdges)
    convert_to_kml(executor, name, add_suffix(name, '.kml_gmt'),
                   KmlMode.GeomEdges)


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


def filter_lines(in_path, selection, out_path):
    with open(in_path, 'r') as in_file, open(out_path, 'w') as out_file:
        out_file.writelines(in_file.readlines()[selection[0]:selection[1]])


def benchmark(executor, args, in_pose, out_pose, out_dir, out_name):
    print(
        "=== Running pose tool to resample the optimized graph for benchmarking ===")
    executor.run([
        'kitti_pose_interpolator',
        f'-in_file_pose={in_pose}',
        f'-in_file_times={args.in_file_ground_truth_times}',
        f'-in_file_calib_imu_to_lidar={args.in_file_calib_imu_to_lidar}',
        f'-in_file_calib_lidar_to_cam={args.in_file_calib_lidar_to_cam}',
        f'-in_file_ground_truth={args.in_file_ground_truth_poses}',
        f'-range={args.range}',
        f'-out_file_pose={out_pose}'])
    create_debug_files(executor, out_pose)

    print("=== Performing the benchmark ===")
    executor.run([
        'kitti_benchmarker',
        f'-in_file_reference={args.in_file_ground_truth_poses}',
        f'-in_file_test={out_pose}',
        f'-out_dir_result={out_dir}',
        f'-out_file_name={out_name}'])


def main(in_args=None):
    # Define and parse command line parameters
    parser = argparse.ArgumentParser()
    parser.add_argument("--in_dir",
                        type=arg_checks.dir_exists,
                        help="Path to the input folder containing the oxts and velodyne_points folders.",
                        required=True)
    parser.add_argument("--in_file_calib_imu_to_lidar",
                        help="Path to the input lidar calibration extrinsics file.",
                        type=arg_checks.file_exists,
                        required=True)
    parser.add_argument("--in_file_calib_lidar_to_cam",
                        help="Path to the input lidar calibration extrinsics file.",
                        type=arg_checks.file_exists,
                        required=True)
    parser.add_argument("--in_file_ground_truth_poses",
                        help="Path to the kitti ground truth pose file.",
                        type=arg_checks.file_exists,
                        required=True)
    parser.add_argument("--in_file_ground_truth_times",
                        help="Path to the kitti ground truth times file.",
                        type=arg_checks.file_exists,
                        required=True)
    parser.add_argument("--out_dir",
                        type=arg_checks.dir_exists,
                        help="Path to the output directory.", required=True)
    parser.add_argument("--range", type=str, default="",
                        help="Subselection of data, '0-1000' will only process the first till the 1000th sweep.")
    parser.add_argument("--generate_las", action='store_true',
                        help="Boolean parameter whether final lasses should be created.")
    parser.add_argument("--debug_level", type=int, default=0,
                        help="The level of debug information to output. 0 = Nothing; "
                             "1 = Input of the Viewer Apps; 2 = All the Intermediate Files.")
    parser.add_argument("--applications_path",
                        type=arg_checks.dir_exists,
                        help="Path of SLAM executables used in the script",
                        default=os.path.dirname(os.path.abspath(__file__)))
    args = parser.parse_args(in_args)

    executor = Executor(args.applications_path)

    # input folders and files
    in_file_oxts = os.path.join(args.in_dir, "oxts", "timestamps.txt")
    in_dir_pnts = os.path.join(args.in_dir, "velodyne_points")
    in_dir_pnts_data = os.path.join(in_dir_pnts, "data")
    in_file_pnts_start = os.path.join(in_dir_pnts, "timestamps_start.txt")
    in_file_pnts_end = os.path.join(in_dir_pnts, "timestamps_end.txt")

    # Base output folders
    out_dir_debug = os.path.join(args.out_dir, "debug")
    out_dir_int = os.path.join(args.out_dir, "intermediate")
    out_dir_benchmark = os.path.join(args.out_dir, "benchmark")
    out_dir_las = os.path.join(args.out_dir, "las")

    # Intermediate output
    source_list_file = os.path.join(out_dir_int, 'source_list_file.txt')
    gps_orig_pose = os.path.join(out_dir_int, "gps_orig.pose")
    gps_pose = os.path.join(out_dir_int, "gps.pose")
    gps_resampled_pose = os.path.join(out_dir_int, "gps_resampled.pose")
    lo_pose = os.path.join(out_dir_int, "lo.pose")
    lo_abs_pose = os.path.join(out_dir_int, "lo_abs.pose")
    lo_abs_reweighted_pose = os.path.join(out_dir_int, "lo_abs_reweighted.pose")
    lo_abs_resampled_pose = os.path.join(out_dir_int, "lo_abs_resampled.pose")

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
    ba_pose = os.path.join(out_dir_int,
                           "ba.pose")  # Specified in graph optimizer app

    # User-facing output
    out_dir_lo = os.path.join(args.out_dir, "lo")
    lo_iref = os.path.join(args.out_dir, "lo.iref")
    full_problem_pose = os.path.join(args.out_dir, "full_problem.pose")
    ba_filtered_pose = os.path.join(args.out_dir, "ba_filtered.pose")
    ba_filtered_resampled_pose = os.path.join(args.out_dir,
                                              "ba_filtered_resampled.pose")
    slam_iref = os.path.join(args.out_dir, "slam.iref")

    # Create output subfolders
    os.makedirs(out_dir_int, exist_ok=True)
    os.makedirs(out_dir_lo, exist_ok=True)
    os.makedirs(out_dir_debug, exist_ok=True)
    os.makedirs(out_dir_benchmark, exist_ok=True)
    os.makedirs(out_dir_las, exist_ok=True)

    print("=== Running PosT converter ===")
    executor.run([
        'pose_tool',
        '-convert_from',
        '-conversion_format=kitti',
        f'-in_paths={in_file_oxts}',
        f'-out_paths={gps_orig_pose}'])
    create_debug_files(executor, gps_orig_pose)

    print("=== Running pose distorter ===")
    executor.run([
        'pose_distorter',
        f'-in_file_pose={gps_orig_pose}',
        f'-settings=drift rel,10,-1,1,0;noise,.1,.05,.05;uncertain,30',
        f'-out_file_pose={gps_pose}'])
    create_debug_files(executor, gps_pose)

    print("=== Calculating lidar odometry ===")

    points_files = glob(os.path.join(in_dir_pnts_data, "*.txt"))
    print(f"Identified {len(points_files)} sweeps.")
    points_files.sort()
    open(source_list_file, 'w').writelines(
        '\n'.join([in_file_pnts_start, in_file_pnts_end, *points_files]))

    executor.run([
        'lidar_odometry_estimator',
        '-lidar_id=kitti',
        f'-in_lidar_extrinsics_file={args.in_file_calib_imu_to_lidar}',
        f'-in_source_list_file={source_list_file}',
        '-pose_trim_size=0.0',
        '-traveled_distance=100.0',
        f'-output_dir={out_dir_lo}',
        '-output_prefix=lo'])

    shutil.move(os.path.join(out_dir_lo, "lo.pose"), lo_pose)
    create_debug_files(executor, lo_pose)
    shutil.move(os.path.join(out_dir_lo, "lo.iref"), lo_iref)

    print("=== Converting relative odometry to absolute ===")
    executor.run([
        'loam_pose_rel_to_abs',
        f'-in_file_pose_gps={gps_pose}',
        f'-in_file_pose_odom_rel={lo_pose}',
        f'-out_file_pose_odom_abs={lo_abs_pose}'])
    create_debug_files(executor, lo_abs_pose)

    shutil.move(os.path.join(out_dir_lo, "lo_cart_bbox.bbox"), cart_bbox_bbox)
    shutil.move(os.path.join(out_dir_lo, "lo_cart_bbox.pose"), cart_bbox_pose)
    shutil.move(os.path.join(out_dir_lo, "lo_cart_bbox.iref"), cart_bbox_iref)

    shutil.move(os.path.join(out_dir_lo, "lo_orien_bbox.bbox"), orien_bbox_bbox)
    shutil.move(os.path.join(out_dir_lo, "lo_orien_bbox.pose"), orien_bbox_pose)
    shutil.move(os.path.join(out_dir_lo, "lo_orien_bbox.iref"), orien_bbox_iref)

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
    create_debug_files(executor, loops_pose, True)

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
    create_debug_files(executor, loops_filtered_pose, True)

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

    print("=== Running pose tool to identify graph nodes ===")
    executor.run([
        'pose_tool',
        '-subsample',
        f'-in_paths={lo_abs_pose}',
        '-vertex_distance_threshold=4',
        f'-out_paths={graph_nodes_csv}'])

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

    print("=== Running tool to reweight the odometry edges ===")
    executor.run([
        'odometry_edge_reweighter',
        f'-in_file_pose_gps={gps_pose}',
        f'-in_file_pose_odom={lo_abs_pose}',
        f'-out_file_pose={lo_abs_reweighted_pose}'])
    create_debug_files(executor, lo_abs_reweighted_pose)

    print("=== Running pose tool to merge the loam trace with the edges ===")
    executor.run([
        'pose_tool',
        '-merge',
        f'-in_paths={lo_abs_reweighted_pose},{lo_loops_pose}',
        f'-out_paths={partial_problem_pose}'])

    print("=== Running graph builder ===")
    executor.run([
        'pgo_problem_builder',
        f'-in_file_gps_pose={gps_pose}',
        f'-in_file_lo_pose={partial_problem_pose}',
        f'-in_file_pose_ids_csv={graph_nodes_csv}',
        f'-out_file_pgo_pose={full_problem_pose}'])
    create_debug_files(executor, full_problem_pose)

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
    create_debug_files(executor, ba_pose)
    create_debug_files(executor, ba_filtered_pose)

    benchmark(executor, args, gps_pose, gps_resampled_pose, out_dir_benchmark,
              "gps")
    benchmark(executor, args, lo_abs_pose, lo_abs_resampled_pose,
              out_dir_benchmark, "lo")
    benchmark(executor, args, ba_filtered_pose, ba_filtered_resampled_pose,
              out_dir_benchmark, "slam")

    if args.generate_las:
        # Output one las for every pcap file given
        print("=== Create final las files ===")
        executor.run([
            'las_creator',
            '-lidar_id=kitti',
            f'-pose_file={ba_filtered_pose}',
            f'-lidar_extrinsics_file={args.in_file_calib_imu_to_lidar}',
            f'-source_list_file={source_list_file}',
            '-traveled_distance=100',
            f'-output_dir={out_dir_las}',
            f'-output_prefix=las'])

    # Conditional clean up
    if args.debug_level < 2:
        shutil.rmtree(out_dir_int)
        shutil.rmtree(out_dir_debug)
    if args.debug_level < 1:
        shutil.rmtree(out_dir_lo)
        os.remove(lo_iref)
        os.remove(full_problem_pose)
        os.remove(ba_filtered_pose)


if __name__ == '__main__':
    main()
