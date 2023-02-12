#!/bin/python3
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

import argparse
import os
import subprocess

VO_OUTPUT = "vo"
BA_OUTPUT = "ba"  # hard-coded in graph optimizer app
SIMPLE_OUTPUT = "poses.txt"
SIMPLE_OUTPUT_HEADER = ' '.join([
    'R_00', 'R_01', 'R_02', 't_0',
    'R_10', 'R_11', 'R_12', 't_1',
    'R_20', 'R_21', 'R_22', 't_2'])


LOOP_CLOSURE = True


def PrintKml(app_path, file_name, out_path, out_file_name=None):
    subprocess.run([
        app_path + 'pose_tool',
        '-convert_to',
        '-conversion_format=kml',
        f'-in_paths={out_path + os.sep + file_name}',
        f'-out_paths={out_path}'
    ]).check_returncode()
    if not out_file_name:
        out_file_name = file_name + ".kml"
    subprocess.run([
        'bash',
        '-c',
        f'cd "{out_path}"; mv pose.kml "{out_file_name}"'
    ]).check_returncode()


def main():
    # Define and parse command line parameters
    parser = argparse.ArgumentParser()
    parser.add_argument("--visual_odometry_exe", type=str, required=True,
                        help="Path to the visual odometry executable.")
    parser.add_argument("--loop_closer_exe", type=str, required=True,
                        help="Path to the loop closer executable.")
    parser.add_argument("--in_dir_images", type=str, required=True,
                        help="Path to a directory containing the images.")
    parser.add_argument("--in_file_extrinsics", type=str, required=True,
                        help="Path to the input extrinsics xml file containing the calibration information.")
    parser.add_argument("--in_file_boresight", type=str, required=True,
                        help="Path to the input boresight yaml file containing the calibration information.")
    parser.add_argument("--in_file_post", type=str, required=True,
                        help="Path to the input PosT file containing the GPS information.")
    parser.add_argument("--out_dir", type=str, required=True,
                        help="Path to the output directory.")
    parser.add_argument("--keep_intermediates", default=False, action="store_true",
                        help="Keep the intermediate results.")

    args = parser.parse_args()

    # Define application directory
    application_path = os.path.dirname(os.path.abspath(__file__)) + os.sep

    # Define google log specific parameters for all applications of the pipeline
    # application_log_args = ['-alsologtostderr', '-v', '5']
    application_log_args = ['-alsologtostderr']

    # Construct all files path and tool arguments
    output_vo_g2o_file = os.sep.join([args.out_dir, VO_OUTPUT + '.g2o'])
    output_vo_csv_file = os.sep.join([args.out_dir, VO_OUTPUT + '.csv'])

    gps_pose_file = os.sep.join([args.out_dir, 'gps.pose'])

    # pose tool
    iref_file = os.sep.join([args.out_dir, "info_refs.iref"])
    pose_before_file = os.sep.join([args.out_dir, "estimates.pose"])

    iref_txt_file = iref_file + ".txt"

    candidates_loops_file = os.sep.join([args.out_dir, "candidate_loops.csv"])
    verified_loops_file = os.sep.join([args.out_dir, "verified_loops.csv"])
    # pose interpolator
    merge_info_file = os.sep.join([args.out_dir, "merge_info.csv"])

    # graph optimizer
    pose_after_file = os.sep.join([args.out_dir, BA_OUTPUT + ".pose"])

    simple_pose_file = os.sep.join([args.out_dir, SIMPLE_OUTPUT])

    files_to_keep = [simple_pose_file]

    #
    # Visual odometry section
    #
    print("=== Running visual odometry ===")
    visual_odometry_args = [
        args.visual_odometry_exe,
        args.in_dir_images,
        'weiya',
        args.in_file_extrinsics,
        output_vo_g2o_file]
    subprocess.run(
        visual_odometry_args + application_log_args).check_returncode()

    #
    # Data conversion section
    #
    print("=== Converting g2o to internal format ===")
    g2o_to_pose_args = [
        application_path + 'pose_tool',
        '-convert_from',
        '-conversion_format=g2o_tue',
        f'-in_paths={output_vo_g2o_file}',
        f'-in_file_csv={output_vo_csv_file}',
        f'-in_dir_images={args.in_dir_images}',
        f'-out_paths={args.out_dir}']
    subprocess.run(g2o_to_pose_args + application_log_args).check_returncode()

    PrintKml(application_path, "estimates.pose", args.out_dir)

    print("=== Converting PosT to internal format ===")
    post_to_pose_args = [
        application_path + 'pose_tool',
        '-convert_from',
        '-conversion_format=post',
        f'-in_paths={args.in_file_post}',
        f'-out_paths={gps_pose_file}']
    subprocess.run(post_to_pose_args + application_log_args).check_returncode()

    PrintKml(application_path, "gps.pose", args.out_dir)

    print("=== Combining GPS data with VO output ===")
    combine_poses_args = [
        application_path + 'pose_interpolator',
        f'-in_file_pose_abs={gps_pose_file}',
        f'-in_out_paths_rel={pose_before_file}']
    subprocess.run(combine_poses_args + application_log_args).check_returncode()

    PrintKml(application_path, "estimates.pose", args.out_dir, "interpolated_estimates.pose.kml")

    #
    # Loop closure section
    #
    if LOOP_CLOSURE:
        print("=== Printing information references ===")
        find_loops_args = [
            application_path + 'pose_tool',
            '-print',
            f'-in_paths={iref_file}']
        with open(iref_txt_file, "w") as f:
            subprocess.run(find_loops_args, stdout=f).check_returncode()

        print("=== Finding loop candidates ===")
        find_loops_args = [
            application_path + 'loop_finder',
            f'-in_file={pose_before_file}',
            f'-out_file={candidates_loops_file}',
            f'-max_distance=10',
            f'-min_time=60']
        subprocess.run(find_loops_args + application_log_args).check_returncode()

        print("=== Verifying loop candidates ===")
        loop_closer_args = [
            args.loop_closer_exe,
            iref_txt_file,
            args.in_file_extrinsics,
            candidates_loops_file,
            verified_loops_file]
        subprocess.run(loop_closer_args + application_log_args).check_returncode()

        print("=== Adding loops ===")
        add_loops_args = [
            application_path + 'csv_to_pose_edge',
            f'-in_out_file_pose={pose_before_file}',
            f'-in_file_csv_loops={verified_loops_file}',
            f'-in_file_csv_merge_info={merge_info_file}']
        subprocess.run(add_loops_args + application_log_args).check_returncode()

    #
    # Bundle adjustment and finalizing section
    #
    print("=== Running graph optimizer ===")
    graph_optimizer_args = [
        application_path + 'graph_optimizer',
        f'-in_files_estimates={pose_before_file}',
        f'-out_directory_ba_graph={args.out_dir}']
    subprocess.run(
        graph_optimizer_args + application_log_args).check_returncode()

    PrintKml(application_path, "ba.pose", args.out_dir)

    print("=== Converting internal format result to simple ===")
    pose_to_kitti_args = [
        application_path + 'pose_tool',
        '-convert_to',
        '-conversion_format=kitti',
        f'-in_paths={pose_after_file}',
        f'-out_paths={simple_pose_file}',
        f'-in_file_csv_merge_info={merge_info_file}']
    subprocess.run(
        pose_to_kitti_args + application_log_args).check_returncode()
    subprocess.run([
        'bash',
        '-c',
        f'sed -i 1i"{SIMPLE_OUTPUT_HEADER}" "{simple_pose_file}"'
    ]).check_returncode()

    # Clean up of intermediate files
    if not args.keep_intermediates:
        print("=== Cleaning up of files ===")
        for root, _, files in os.walk(args.out_dir):
            for f in files:
                f = os.sep.join([root, f])
                if f not in files_to_keep:
                    os.remove(f)


if __name__ == '__main__':
    main()
