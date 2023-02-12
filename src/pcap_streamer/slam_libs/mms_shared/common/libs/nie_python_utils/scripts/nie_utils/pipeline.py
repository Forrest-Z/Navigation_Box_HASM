# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

__all__ = ['Pipeline']

from glob import glob
from pathlib import Path
from shutil import copy2 as copy2
from typing import List, Union

from nie_utils.pathlib import add_suffix
from nie_utils.pipeline_tools import *


class Pipeline():
    """ Base class to be used for setting up pipelines """

    def __init__(self, apps_path):
        self._executor = Executor(apps_path)

    def _create_txt(self, in_file):
        convert_to_txt(self._executor, in_file)

    def _create_kmls(self, in_file):
        convert_to_kml(self._executor, in_file,
                       add_suffix(in_file, '.kml'))
        convert_to_kml(self._executor, in_file,
                       add_suffix(in_file, '.kml_rel'), KmlMode.RelEdges)
        convert_to_kml(self._executor, in_file,
                       add_suffix(in_file, '.kml_gmt'), KmlMode.GeomEdges)

    @staticmethod
    def _create_output_directory(dirs):
        Pipeline.__str_or_list(dirs, lambda d: d.mkdir(exist_ok=True))

    @staticmethod
    def _check_output_directory(dirs):
        def action(dir):
            if not dir.exists():
                raise Exception(f"Folder '{dir}' does not exists.")

        Pipeline.__str_or_list(dirs, action)

    # Call function with the argument if the argument is a string, other wise do
    # it for all elements in the argument as it is expected to be an iterable.
    @staticmethod
    def __str_or_list(objects, function):
        if isinstance(objects, str):
            function(objects)
        else:
            for object in objects:
                function(object)

    def _concat_files(self, directory: str, pairs):
        Pipeline.__check_is_list_of_string_pairs(pairs)

        pairs_size = len(pairs)
        concatenated_files = [glob(str(Path(directory) / pair[0])) for pair in pairs]
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
            self._executor.run([
                'pose_tool',
                '-concat',
                f'-in_paths={",".join([file for files in concatenated_files for file in files])}',
                f'-out_paths={",".join(output_files)}'])
        elif num_files == 1:
            for in_files, out_file in zip(concatenated_files, output_files):
                copy2(in_files[0], out_file)

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

    # Call the merge option of the pose_tool with all files in the supplied
    # directory that match the given pattern and name the merged pose file as
    # indicated.
    def _merge_files_pattern(self, directory: Union[str, Path], pattern: str, output: Union[str, Path]):
        self._merge_files_list(glob(str(Path(directory) / pattern)), output)

    # Call the merge option of the pose_tool with given files list and name the
    # merged pose file as indicated.
    def _merge_files_list(self, files: List, output: Union[str, Path]):
        if len(files) > 1:
            # Convert files to string as they can be Paths as well
            files = sorted([str(f) for f in files])
            print("=== Running pose tool to merge the files ===")
            self._executor.run([
                'pose_tool',
                '-merge',
                f'-in_paths={",".join(files)}',
                f'-out_paths={output}'])
        elif len(files) == 1:
            copy2(files[0], output)
