# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
""" Argument check functions following the contract of the argparse module """

import argparse
import os

def file_exists(file_path):
    """ Checks if a file exists; returns the path or throws exception """
    if not os.path.isfile(file_path):
        raise argparse.ArgumentTypeError(f"No file under path <{file_path}>")
    # if it is file, just return the file_path as is
    return file_path


def dir_exists(dir_path):
    """ Checks if a directory exists; returns the path or throws exception """
    if not os.path.isdir(dir_path):
        raise argparse.ArgumentTypeError(
            f"No directory under path <{dir_path}>")
    # if it is a directory, just return the dir_path as is
    return dir_path