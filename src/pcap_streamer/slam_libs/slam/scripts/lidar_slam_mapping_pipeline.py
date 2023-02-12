#!/usr/bin/python3
# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

# This script uses only lidar data to determine the trajectory and corresponding
# las files.
# Inputs:
#   - A folder that contains the lidar data in the form of pcap files
#   - The lidar intrinsics and extrinsics file
# Rough processing steps:
#   - lidar odometry using loam
#   - loop closer using icp
#   - graph optimization combining lidar odometry and loop closure information
#   - las creator to create las files using lidar data and optimized trajectory
# Outputs:
#   - Pose file containing the poses of the optimized trajectory
#   - A folder that contains the lidar data in the form of las files based on
#     the optimized trajectory

import argparse
from glob import glob
import os
import shutil
from pathlib import Path
from typing import NamedTuple

from nie_utils import arg_checks
from nie_utils.pipeline import Pipeline


class LidarSlamMappingPipeline(Pipeline):
    class Configuration(NamedTuple):
        in_file_lidar_intr: Path
        in_file_lidar_extr: Path
        in_dir_pcap: Path
        in_file_loops: str
        out_dir: Path
        debug_level: int
        output_txt: bool = False
        output_kml: bool = False

    def __init__(self, apps_path, conf):
        assert isinstance(conf, self.Configuration)

        super().__init__(apps_path)
        self.__conf = conf

        self.__define_paths()

    # First part of the pipeline runs up and including the lidar_loop_closer
    def part_1(self):
        # Create output subfolders
        self._create_output_directory([
            self.__out_dir_debug, self.__out_dir_int,
            self.__out_dir_lo, self.__out_dir_las])

        print("=== Calculating lidar odometry ===")

        files = glob(str(self.__conf.in_dir_pcap / "*"))
        print(f"Identified {len(files)} pcap files.")
        files.sort()
        open(self.__source_list_file, 'w').writelines('\n'.join([*files]))

        self._executor.run([
            'lidar_odometry_estimator',
            '-lidar_id=ouster',
            f'-in_lidar_intrinsics_file={self.__conf.in_file_lidar_intr}',
            f'-in_lidar_extrinsics_file={self.__conf.in_file_lidar_extr}',
            f'-in_source_list_file={self.__source_list_file}',
            '-pose_trim_size=0.0',
            '-traveled_distance=20.0',
            '-min_ray_length=2.0',
            '-max_ray_length=50.0',
            f'-output_dir={self.__out_dir_lo}',
            '-output_prefix=lo'])

        # Note that the merging of files is strange as there is only one file
        # that will be "merged" and so that file in practive will just be
        # copied. The reason for this is to keep the logic in line with the
        # other lidar slam pipeline scripts.
        self._merge_files_pattern(self.__out_dir_lo, "lo.pose", self.__lo_pose)
        self.__create_debug_files(self.__lo_pose)
        self._merge_files_pattern(self.__out_dir_lo, "lo.iref", self.__lo_iref)

        self._merge_files_pattern(self.__out_dir_lo, "lo_cart_bbox.bbox",
                                  self.__cart_bbox_bbox)
        self._merge_files_pattern(self.__out_dir_lo, "lo_cart_bbox.pose",
                                  self.__cart_bbox_pose)
        self._merge_files_pattern(self.__out_dir_lo, "lo_cart_bbox.iref",
                                  self.__cart_bbox_iref)

        self._merge_files_pattern(self.__out_dir_lo, "lo_orien_bbox.bbox",
                                  self.__orien_bbox_bbox)
        self._merge_files_pattern(self.__out_dir_lo, "lo_orien_bbox.pose",
                                  self.__orien_bbox_pose)
        self._merge_files_pattern(self.__out_dir_lo, "lo_orien_bbox.iref",
                                  self.__orien_bbox_iref)

        print("=== Running loop finder ===")
        # Cartesian bounding boxes are used as the naive edge transformation
        # should be based on these coordinates (as they are also used for the
        # las generation).
        self._executor.run([
            'loop_finder',
            f'-in_file={self.__cart_bbox_pose}',
            f'-out_file={self.__loops_pose}',
            '-max_distance=250',
            '-min_time=20'])
        self.__create_debug_files(self.__loops_pose, True)

        print("=== Running loop filter ===")
        self._executor.run([
            'lidar_loop_filter',
            f'-in_file_loops={self.__loops_pose}',
            f'-in_file_bbox_bbox={self.__orien_bbox_bbox}',
            f'-in_file_bbox_pose={self.__orien_bbox_pose}',
            f'-in_file_bbox_iref={self.__orien_bbox_iref}',
            f'-in_file_trace_pose={self.__lo_pose}',
            f'-in_file_trace_iref={self.__lo_iref}',
            f'-out_file_loops={self.__loops_filtered_pose}',
            '-intersection_area_threshold=20'])
        self.__create_debug_files(self.__loops_filtered_pose, True)

    # Second part of the pipeline only running the lidar_loop_closer
    def part_2(self):
        self._check_output_directory([
            self.__out_dir_debug, self.__out_dir_int,
            self.__out_dir_lo, self.__out_dir_las])

        print("=== Running loop closer ===")
        command = [
            'lidar_loop_closer',
            f'-in_file_pose_trace={self.__lo_pose}',
            f'-in_file_pose_edges={self.__loops_filtered_pose}',
            f'-in_file_bbox_bbox={self.__orien_bbox_bbox}',
            f'-in_file_pose_bbox={self.__orien_bbox_pose}',
            f'-in_file_iref_bbox={self.__orien_bbox_iref}',
            f'-out_file_pose_edges={self.__bbox_loops_pose}']
        if self.__conf.debug_level >= 2:
            command += [
                '-debug',
                f'-out_dir_debug={self.__out_dir_debug}']
        self._executor.run(command)
        self.__create_debug_files(self.__bbox_loops_pose, True)

    # Third part of the pipeline starting after the lidar_loop_closer
    def part_3(self):
        self._check_output_directory([
            self.__out_dir_debug, self.__out_dir_int,
            self.__out_dir_lo, self.__out_dir_las])

        loops_files = [self.__bbox_loops_pose]
        if self.__conf.in_file_loops:
            loops_files.append(self.__conf.in_file_loops)
            self.__create_debug_files(self.__conf.in_file_loops, True)
        self._merge_files_list(loops_files, self.__bbox_loops_combined_pose)
        self.__create_debug_files(self.__bbox_loops_combined_pose, True)

        print("=== Running pose tool to identify graph nodes ===")
        self._executor.run([
            'pose_tool',
            '-subsample',
            f'-in_paths={self.__lo_pose}',
            '-vertex_distance_threshold=4',
            f'-out_paths={self.__graph_nodes_csv}'])

        print(
            "=== Running tool to convert the loop closer edges to loam trace edges ===")
        self._executor.run([
            'bbox_to_pose_edge_converter',
            f'-in_file_bbox_loops={self.__bbox_loops_pose}',
            f'-in_file_aa_bbox_pose={self.__cart_bbox_pose}',
            f'-in_file_aa_bbox_iref={self.__cart_bbox_iref}',
            f'-in_file_oa_bbox_pose={self.__orien_bbox_pose}',
            f'-in_file_oa_bbox_bbox={self.__orien_bbox_bbox}',
            f'-in_file_trace_pose={self.__lo_pose}',
            f'-in_file_trace_iref={self.__lo_iref}',
            f'-in_file_trace_ids={self.__graph_nodes_csv}',
            f'-out_file_trace_loops={self.__lo_loops_pose}'])

        print(
            "=== Running pose tool to merge the loam trace with the edges ===")
        self._executor.run([
            'pose_tool',
            '-merge',
            f'-in_paths={self.__lo_pose},{self.__lo_loops_pose}',
            f'-out_paths={self.__partial_problem_pose}'])
        self.__create_debug_files(self.__partial_problem_pose)

        print("=== Running pose tool to fix the first loam pose ===")
        self._executor.run([
            'pose_tool',
            '-add_fixture',
            f'-in_paths={self.__partial_problem_pose}',
            f'-out_paths={self.__full_problem_pose}'])
        self.__create_debug_files(self.__full_problem_pose)

        print("=== Running graph optimizer ===")
        self._executor.run([
            'graph_optimizer',
            f'-in_files_estimates={self.__full_problem_pose}',
            f'-out_directory_ba_graph={self.__conf.out_dir}',
            '-double_pass',
            '-mahalanobis_threshold=500'])
        self.__create_debug_files(self.__ba_pose)

        # Output one las for every pcap file given
        print("=== Create final las files ===")
        self._executor.run([
            'las_creator',
            '-lidar_id=ouster',
            f'-pose_file={self.__ba_pose}',
            '-lidar_odometry',
            f'-lidar_intrinsics_file={self.__conf.in_file_lidar_intr}',
            f'-lidar_extrinsics_file={self.__conf.in_file_lidar_extr}',
            f'-source_list_file={self.__source_list_file}',
            '-traveled_distance=20',
            '-min_ray_length=2.0',
            '-max_ray_length=300.0',
            f'-output_dir={self.__out_dir_las}',
            f'-output_prefix=las',
            '-output_iref'])

        self._merge_files_pattern(self.__out_dir_las, "las.iref",
                                  self.__las_iref)

    def clean_up(self):
        # Conditional clean up
        if self.__conf.debug_level < 2:
            shutil.rmtree(self.__out_dir_debug)
            shutil.rmtree(self.__out_dir_int)
        if self.__conf.debug_level < 1:
            shutil.rmtree(self.__out_dir_lo)
            os.remove(self.__lo_iref)
            os.remove(self.__full_problem_pose)

    def __define_paths(self):
        # Base output folders
        self.__out_dir_debug = self.__conf.out_dir / "debug"
        self.__out_dir_int = self.__conf.out_dir / "intermediate"
        self.__out_dir_lo = self.__conf.out_dir / "lo"
        self.__out_dir_las = self.__conf.out_dir / "las"

        # Intermediate output
        self.__source_list_file = self.__out_dir_int / 'source_list_file.txt'
        self.__lo_pose = self.__out_dir_int / "lo.pose"

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
        self.__bbox_loops_combined_pose = self.__out_dir_int / "bbox_loops_combined.pose"
        self.__lo_loops_pose = self.__out_dir_int / "lo_loops.pose"
        self.__partial_problem_pose = self.__out_dir_int / "partial_problem.pose"

        # User-facing output
        self.__lo_iref = self.__conf.out_dir / "lo.iref"
        self.__las_iref = self.__conf.out_dir / "las.iref"
        self.__full_problem_pose = self.__conf.out_dir / "full_problem.pose"
        self.__ba_pose = self.__conf.out_dir / "ba.pose"  # Specified in graph optimizer app

    def __create_debug_files(self, in_file, txt_only=False):
        print("= Creating debug pose files ===")
        name = Path(in_file)
        if self.__conf.output_txt:
            self._create_txt(name)
        if txt_only:
            return
        if self.__conf.output_kml:
            self._create_kmls(name)


def main(in_args=None):
    # Define and parse command line parameters
    parser = argparse.ArgumentParser()
    parser.add_argument("--in_file_lidar_intr", type=arg_checks.file_exists,
                        required=True,
                        help="Path to the input lidar calibration intrinsics file.")
    parser.add_argument("--in_file_lidar_extr", type=arg_checks.file_exists,
                        required=True,
                        help="Path to the input lidar calibration extrinsics file.")
    parser.add_argument("--in_dir_pcap", type=arg_checks.dir_exists,
                        required=True,
                        help="Path to a directory containing input PCAP files.")
    parser.add_argument("--in_file_loops", type=arg_checks.file_exists,
                        required=False,
                        help="Path to pose file with manual loop closure situations.")
    parser.add_argument("--out_dir", type=arg_checks.dir_exists, required=True,
                        help="Path to the output directory.")
    parser.add_argument("--mode", type=str, default="0",
                        help="The mode indicating which parts of the pipeline will be ran."
                             "0 = Full pipeline, icp results only filtered by graph optimizer double pass; "
                             "1 = First part of pipeline up to the loop_closer; "
                             "2 = Second part of pipeline, only running the loop_closer."
                             "3 = Third part of pipeline, after loop_closer and potentially using the provided manual loop closures.")
    parser.add_argument("--debug_level", type=int, default=0,
                        help="The level of debug information to output. 0 = Nothing; "
                             "1 = Input of the Viewer Apps; 2 = All the Intermediate Files.")
    parser.add_argument("--applications_path", type=arg_checks.dir_exists,
                        default=os.path.dirname(os.path.abspath(__file__)),
                        help="Path of SLAM executables used in the script")
    args = parser.parse_args(in_args)

    if not all(o in "0123" for o in args.mode):
        raise Exception(
            "Unknown mode supplied, can only consist of 0, 1, 2 and/or 3.")

    conf = LidarSlamMappingPipeline.Configuration(
        in_file_lidar_intr=Path(args.in_file_lidar_intr),
        in_file_lidar_extr=Path(args.in_file_lidar_extr),
        in_dir_pcap=Path(args.in_dir_pcap),
        in_file_loops=args.in_file_loops,
        out_dir=Path(args.out_dir),
        debug_level=args.debug_level,
        output_txt=(args.debug_level == 2),
        output_kml=(args.debug_level == 2)
    )

    pipeline = LidarSlamMappingPipeline(args.applications_path, conf)

    all_parts = "0" in args.mode
    if all_parts or "1" in args.mode:
        pipeline.part_1()
    if all_parts or "2" in args.mode:
        pipeline.part_2()
    if all_parts or "3" in args.mode:
        pipeline.part_3()
        pipeline.clean_up()


if __name__ == '__main__':
    main()
