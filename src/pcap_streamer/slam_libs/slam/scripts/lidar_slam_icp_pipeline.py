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
#   - las creator to create las files using lidar data and given trajectory data
#   - loop closer using icp
#   - graph optimization combining gps and loop closure information
#   - las creator to create las files using lidar data and optimized trajectory
# Outputs:
#   - Csv file containing the poses of the optimized trajectory
#   - A folder that contains the lidar data in the form of las files based on
#     the optimized trajectory

import argparse
from glob import glob
import shutil
from typing import NamedTuple
from pathlib import Path

from nie_utils import arg_checks
from nie_utils.pipeline_tools import *


class Pipeline():
    class Configuration(NamedTuple):
        in_file_post: Path
        classic_post: bool
        in_file_lidar_intr: Path
        in_file_lidar_extr: Path
        in_dir_pcap: Path
        out_dir: Path
        debug_level: int
        output_txt: bool = False
        output_kml: bool = False

    def __init__(self, apps_path, conf):
        self.__executor = Executor(apps_path)
        self.__conf = conf

        self.__define_paths()

    # First part of the pipeline runs up and including the lidar_loop_closer
    def part_1(self):
        self.__create_output_directory()

        print("=== Running PosT converter ===")
        self.__executor.run([
            'pose_tool',
            '-convert_from',
            '-conversion_format=post',
            f'-in_paths={self.__conf.in_file_post}',
            f'-out_paths={self.__gps_orig_pose},{self.__gps_stop}'])
        self.__create_txt(self.__gps_orig_pose)
        self.__create_kmls(self.__gps_orig_pose)

        print("=== Running pose tool to resample the trace ===")
        self.__executor.run([
            'pose_tool',
            '-resample',
            '-sample_interval=0.1',
            '-max_edge_distance=5.',
            f'-in_paths={self.__gps_orig_pose}',
            f'-out_paths={self.__gps_pose}'])
        self.__create_txt(self.__gps_pose)
        self.__create_kmls(self.__gps_pose)

        print("=== Running pcap trajectory tool ===")
        self.__executor.run([
            'pcap_trajectory_tool',
            f'-in_dir={self.__conf.in_dir_pcap}',
            f'-in_lidar_intrinsics_file={self.__conf.in_file_lidar_intr}',
            f'-out_dir={self.__out_dir_int_trajectory}',
            f'-in_stationary_intervals={self.__gps_stop}'])

        print("=== Create las files per trajectory ===")
        # The directory also contains which pcaps have been skipped. So only reading files that have a sequence number in
        # the filename.
        trajectory_files = glob(
            str(self.__out_dir_int_trajectory / "*[0-9].txt"))
        print(f"Identified {len(trajectory_files)} trajectories.")
        trajectory_files.sort()
        for trajectory_file in trajectory_files:
            seq_nr = os.path.splitext(trajectory_file)[0][-6:]
            base = "traject_seq_" + seq_nr
            print(f"== Processing traject '{base}' ==")

            self.__executor.run([
                'las_creator',
                f'-lidar_intrinsics_file={self.__conf.in_file_lidar_intr}',
                f'-lidar_extrinsics_file={self.__conf.in_file_lidar_extr}',
                f'-pose_file={self.__gps_pose}',
                f'-source_list_file={trajectory_file}',
                '-traveled_distance=200',
                f'-output_dir={self.__out_dir_int_las}',
                f'-output_prefix={base}',
                '-filter_ground_plane',
                '-output_iref',
                '-output_bbox'])

        print("=== Running pose tool to merge iref files ===")
        self.__merge_files(self.__out_dir_int_las, "*[0-9].iref",
                           self.__gps_iref)

        print(
            "=== Running pose tool to concatenate the cartesian bounding boxes of trajectories ===")
        self.__concat_files(self.__out_dir_int_las,
                            [["*_cart_bbox.bbox", self.__cart_bbox_bbox],
                             ["*_cart_bbox.pose", self.__cart_bbox_pose],
                             ["*_cart_bbox.iref", self.__cart_bbox_iref]])

        print(
            "=== Running pose tool to concatenate the oriented bounding boxes of trajectories ===")
        self.__concat_files(self.__out_dir_int_las,
                            [["*_orien_bbox.bbox", self.__orien_bbox_bbox],
                             ["*_orien_bbox.pose", self.__orien_bbox_pose],
                             ["*_orien_bbox.iref", self.__orien_bbox_iref]])

        print("=== Running loop finder ===")
        # Cartesian bounding boxes are used as the naive edge transformation should
        # be based on these coordinates (as they are also used for the las
        # generation).
        self.__executor.run([
            'loop_finder',
            f'-in_file={self.__cart_bbox_pose}',
            f'-out_file={self.__loops_pose}',
            '-max_distance=250',
            '-min_time=60'])

        print("=== Running loop filter ===")
        self.__executor.run([
            'lidar_loop_filter',
            f'-in_file_loops={self.__loops_pose}',
            f'-in_file_bbox_bbox={self.__orien_bbox_bbox}',
            f'-in_file_bbox_pose={self.__orien_bbox_pose}',
            f'-in_file_bbox_iref={self.__orien_bbox_iref}',
            f'-in_file_trace_pose={self.__gps_pose}',
            f'-in_file_trace_iref={self.__gps_iref}',
            f'-out_file_loops={self.__loops_filtered_pose}'])

        print("=== Running loop closer ===")
        command = [
            'lidar_loop_closer',
            f'-in_file_pose_trace={self.__gps_pose}',
            f'-in_file_pose_edges={self.__loops_filtered_pose}',
            f'-in_file_bbox_bbox={self.__orien_bbox_bbox}',
            f'-in_file_pose_bbox={self.__orien_bbox_pose}',
            f'-in_file_iref_bbox={self.__orien_bbox_iref}',
            f'-out_file_pose_edges={self.__bbox_loops_pose}',
            '-v=3']
        if self.__conf.debug_level >= 2:
            command += [
                '-debug',
                f'-out_dir_debug={self.__out_dir_debug}']
        self.__executor.run(command)
        self.__create_txt(self.__bbox_loops_pose)

    # Second part of the pipeline starting after the lidar_loop_closer
    def part_2(self):
        self.__check_output_directory()

        print("=== Running pose tool to identify graph nodes ===")
        self.__executor.run([
            'pose_tool',
            '-subsample',
            f'-in_paths={self.__gps_pose}',
            '-vertex_distance_threshold=4',
            f'-out_paths={self.__graph_nodes_csv}'])

        print(
            "=== Running tool to convert the loop closer edges to loam trace edges ===")
        self.__executor.run([
            'bbox_to_pose_edge_converter',
            f'-in_file_bbox_loops={self.__bbox_loops_pose}',
            f'-in_file_aa_bbox_pose={self.__cart_bbox_pose}',
            f'-in_file_aa_bbox_iref={self.__cart_bbox_iref}',
            f'-in_file_oa_bbox_pose={self.__orien_bbox_pose}',
            f'-in_file_oa_bbox_bbox={self.__orien_bbox_bbox}',
            f'-in_file_trace_pose={self.__gps_pose}',
            f'-in_file_trace_iref={self.__gps_iref}',
            f'-in_file_trace_ids={self.__graph_nodes_csv}',
            f'-out_file_trace_loops={self.__gps_loops_pose}',
            '-info_translation=0.01'])
        self.__create_txt(self.__gps_loops_pose)

        print("=== Adding edges to gps poses ===")
        self.__executor.run([
            'pose_tool',
            '-add_edges',
            f'-in_paths={self.__gps_pose}',
            f'-out_paths={self.__half_problem_pose}'])

        print(
            "=== Running pose tool to merge the trace with the loop edges ===")
        self.__executor.run([
            'pose_tool',
            '-merge',
            f'-in_paths={self.__half_problem_pose},{self.__gps_loops_pose}',
            f'-out_paths={self.__full_problem_pose}'])
        self.__create_txt(self.__full_problem_pose)
        self.__create_kmls(self.__full_problem_pose)

        print("=== Running graph optimizer ===")
        self.__executor.run([
            'graph_optimizer',
            f'-in_files_estimates={self.__full_problem_pose}',
            f'-out_directory_ba_graph={self.__conf.out_dir}'])
        self.__create_txt(self.__ba_pose)
        self.__create_kmls(self.__ba_pose)

        # Output one las for every pcap file given
        print("=== Create final las files ===")

        pcaps_skipped_filename = os.path.join(self.__out_dir_int_trajectory,
                                              'skipped.txt')
        with open(pcaps_skipped_filename, 'r') as pcaps_skipped_file:
            pcaps_skipped_lines = pcaps_skipped_file.readlines()
        pcaps_skipped = {line.strip() for line in pcaps_skipped_lines}
        pcap_files = [pcap for pcap in
                      glob(str(self.__conf.in_dir_pcap / "*.pcap")) if
                      pcap not in pcaps_skipped]
        pcap_files.sort()

        for pcap_file in pcap_files:
            print(f"== Processing pcap file '{os.path.basename(pcap_file)}' ==")
            with open(self.__single_trajectory_file, 'w') as trajectory_file:
                trajectory_file.write(pcap_file)
            base = os.path.splitext(os.path.basename(pcap_file))[0]
            self.__executor.run([
                'las_creator',
                f'-lidar_extrinsics_file={self.__conf.in_file_lidar_extr}',
                f'-lidar_intrinsics_file={self.__conf.in_file_lidar_intr}',
                f'-pose_file={self.__ba_pose}',
                f'-source_list_file={self.__single_trajectory_file}',
                '-traveled_distance=100000',
                f'-output_dir={self.__out_dir_las}',
                f'-output_prefix={base}',
                '-output_iref'])

        if os.path.exists(self.__single_trajectory_file):
            os.remove(self.__single_trajectory_file)

        print("=== Running pose tool to merge iref files ===")
        self.__merge_files(self.__out_dir_las, "*.iref", self.__slam_iref)

        print("=== Running pose tool to convert the trajectory to csv ===")
        self.__executor.run([
            'pose_tool',
            '-convert_to',
            '-conversion_format=csv',
            f'-in_paths={self.__ba_pose}',
            f'-out_paths={self.__slam_trace_csv}'])

        print("=== Running las file renamer ===")
        self.__executor.run([
            'nie_china_las_namer',
            f'-in_file_aiim={self.__conf.in_file_post}',
            f'-in_file_iref={self.__slam_iref}',
            f'-out_file_iref={self.__slam_iref}'])

    def clean_up(self):
        # Conditional clean up
        if self.__conf.debug_level < 2:
            shutil.rmtree(self.__out_dir_int)
            shutil.rmtree(self.__out_dir_debug)
            for file in glob(str(self.__out_dir_las / "*.iref")):
                os.remove(file)
            os.remove(self.__gps_iref)
            os.remove(self.__full_problem_pose)
            os.remove(self.__ba_pose)
            os.remove(self.__slam_iref)

    def __create_txt(self, in_file):
        if self.__conf.output_txt:
            print("= Converting pose file to txt =")
            convert_to_txt(self.__executor, in_file)

    def __create_kmls(self, in_file):
        if self.__conf.output_kml:
            print("= Converting pose file to kml files =")
            convert_to_kml(self.__executor, in_file,
                           add_suffix(in_file, '.kml'))
            convert_to_kml(self.__executor, in_file,
                           add_suffix(in_file, '.kml_rel'), KmlMode.RelEdges)
            convert_to_kml(self.__executor, in_file,
                           add_suffix(in_file, '.kml_gmt'), KmlMode.GeomEdges)

    def __define_paths(self):
        # Base output folders
        self.__out_dir_int = self.__conf.out_dir / "intermediate"
        self.__out_dir_debug = self.__conf.out_dir / "debug"

        # Intermediate output
        self.__out_dir_int_trajectory = self.__out_dir_int / "trajectory"
        self.__out_dir_int_las = self.__out_dir_int / "las"

        self.__gps_orig_pose = self.__out_dir_int / "gps_orig.pose"
        self.__gps_pose = self.__out_dir_int / "gps.pose"
        self.__gps_stop = self.__out_dir_int / "gps.stop"

        self.__cart_bbox_bbox = self.__out_dir_int / "cart_bbox.bbox"
        self.__cart_bbox_pose = self.__out_dir_int / "cart_bbox.pose"
        self.__cart_bbox_iref = self.__out_dir_int / "cart_bbox.iref"
        self.__orien_bbox_bbox = self.__out_dir_int / "orien_bbox.bbox"
        self.__orien_bbox_pose = self.__out_dir_int / "orien_bbox.pose"
        self.__orien_bbox_iref = self.__out_dir_int / "orien_bbox.iref"

        self.__loops_pose = self.__out_dir_int / "loops.pose"
        self.__loops_filtered_pose = self.__out_dir_int / "loops_filtered.pose"

        self.__graph_nodes_csv = self.__out_dir_int / "graph_nodes.csv"
        self.__bbox_loops_pose = self.__out_dir_int / "bbox_loops.pose"
        self.__gps_loops_pose = self.__out_dir_int / "gps_loops.pose"
        self.__ba_pose = self.__conf.out_dir / "ba.pose"

        self.__single_trajectory_file = self.__out_dir_int / 'single_pcap.txt'

        # User-facing output
        self.__out_dir_las = self.__conf.out_dir / "las"

        self.__gps_iref = self.__conf.out_dir / "gps.iref"
        self.__half_problem_pose = self.__out_dir_int / "half_problem.pose"
        self.__full_problem_pose = self.__conf.out_dir / "full_problem.pose"
        self.__slam_iref = self.__conf.out_dir / "slam.iref"
        self.__slam_trace_csv = (
                self.__conf.out_dir / self.__conf.in_file_post.name).with_suffix(
            ".csv")

        self.__sub_dirs = [
            self.__out_dir_int,
            self.__out_dir_int_las, self.__out_dir_int_trajectory,
            self.__out_dir_las, self.__out_dir_debug]

    def __create_output_directory(self):
        for dir in self.__sub_dirs:
            dir.mkdir(exist_ok=True)

    def __check_output_directory(self):
        for dir in self.__sub_dirs:
            if not dir.exists():
                raise Exception(f"Folder '{dir}' does not exists.")

    # Check if the argument is a list of pairs of strings
    @staticmethod
    def __check_is_list_of_string_pairs(pairs):
        error_message = "Expecting argument to be a list of string pairs."
        if isinstance(pairs, list):
            for pair in pairs:
                if not (isinstance(pair, list) and len(pair) == 2):
                    raise Exception(error_message)
        else:
            raise Exception(error_message)

    def __concat_files(self, directory, pairs):
        Pipeline.__check_is_list_of_string_pairs(pairs)

        pairs_size = len(pairs)
        concatenated_files = [glob(str(directory / pair[0])) for pair in pairs]
        output_files = [str(pair[1]) for pair in pairs]

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
            self.__executor.run([
                'pose_tool',
                '-concat',
                f'-in_paths={",".join([file for files in concatenated_files for file in files])}',
                f'-out_paths={",".join(output_files)}'])
        elif num_files == 1:
            for in_files, out_file in zip(concatenated_files, output_files):
                shutil.copy2(in_files[0], out_file)

    def __merge_files(self, directory, pattern, output):
        files = glob(str(directory / pattern))
        if len(files) > 1:
            files.sort()
            print("=== Running pose tool to merge the files ===")
            self.__executor.run([
                'pose_tool',
                '-merge',
                f'-in_paths={",".join(files)}',
                f'-out_paths={output}'])
        elif len(files) == 1:
            shutil.copy2(files[0], output)


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
    parser.add_argument("--mode", type=int,
                        help="The mode in which the pipeline will be ran."
                             "0 = Full pipeline, icp results only filtered by graph optimizer double pass; "
                             "1 = First part of pipeline up to and including loop_closer, such that verification can be done; "
                             "2 = Second part of pipeline using the verification ICP results.",
                        default=0)
    parser.add_argument("--debug_level", type=int,
                        help="The level of debug information to output. 0 = Nothing; "
                             "2 = All the Intermediate Files.",
                        default=0)
    parser.add_argument("--applications_path",
                        type=arg_checks.dir_exists,
                        help="Path of SLAM executables used in the script",
                        default=os.path.dirname(os.path.abspath(__file__)))
    args = parser.parse_args(in_args)

    configuration = Pipeline.Configuration(
        in_file_post=Path(args.in_file_post),
        classic_post=False,
        in_file_lidar_intr=Path(args.in_file_calib_intr),
        in_file_lidar_extr=Path(args.in_file_calib_extr),
        in_dir_pcap=Path(args.in_dir_pcap),
        out_dir=Path(args.out_dir),
        debug_level=args.debug_level,
        output_txt=(args.debug_level == 2),
        output_kml=(args.debug_level == 2)
    )
    pipeline = Pipeline(args.applications_path, configuration)

    if args.mode in [0, 1]:
        pipeline.part_1()
    if args.mode in [0, 2]:
        pipeline.part_2()
        pipeline.clean_up()


if __name__ == '__main__':
    main()
