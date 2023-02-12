# Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

""" Utilitary free functions to be used in python based pipelines """

__all__ = ['Executor', 'convert_to_txt', 'KmlMode', 'convert_to_kml']

from enum import Enum
from pathlib import Path
import subprocess
from typing import List

from nie_utils.pathlib import add_suffix


class Executor:
    def __init__(self, applications_path: str):
        self.__applications_path = Path(applications_path)

    def run(self, args: List):
        # Define application directory
        args[0] = self.__applications_path / args[0]

        # Define google log specific parameters for all applications of the pipeline
        args += ['-alsologtostderr']

        # Actually run the command
        subprocess.check_output(args)


def convert_to_txt(executor, in_file: str, out_file: str = None):
    if out_file is None:
        out_file = add_suffix(in_file, '.txt')
    executor.run([
        'pose_tool',
        '-print',
        '-to_file',
        f'-in_paths={in_file}',
        f'-out_paths={out_file}'])


class KmlMode(Enum):
    Normal = 0
    RelEdges = 1
    GeomEdges = 2


def convert_to_kml(executor: Executor, in_file: str, out_file: str,
                   mode: KmlMode = KmlMode.Normal):
    assert isinstance(executor, Executor)
    assert isinstance(mode, KmlMode)

    working_dir = Path(out_file).parent

    executor.run([
        'pose_tool',
        f'-convert_to',
        f'-conversion_format=kml',
        '-kml_edges' if mode in [KmlMode.RelEdges, KmlMode.GeomEdges] else '',
        '-kml_edges_relational=false' if mode == KmlMode.GeomEdges else '',
        f'-in_paths={in_file}',
        f'-out_paths={working_dir}'])

    (working_dir / 'pose.kml').rename(out_file)
