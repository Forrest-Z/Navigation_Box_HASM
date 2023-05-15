#!/usr/bin/python3
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

import pytest
import sys
import os
import argparse

from nie_utils import arg_checks

def test_arg_file_exists():
    parser = argparse.ArgumentParser()
    parser.add_argument("--in_file", type=arg_checks.file_exists,
                        help="Input file", required=True)
    
    parser.parse_args(['--in_file', __file__]) #shouldn't give any exceptions

    with pytest.raises(SystemExit): # Arg parsing should fail for inexisting file
        parser.parse_args(['--in_file', '/tmp/blabla'])


def test_arg_dir_exists():
    parser = argparse.ArgumentParser()
    parser.add_argument("--in_dir", type=arg_checks.dir_exists,
                        help="Input dir", required=True)

    parser.parse_args(['--in_dir', os.path.dirname(os.path.abspath(__file__))])
    
    with pytest.raises(SystemExit): # Arg parsing should fail for inexisting dir
        parser.parse_args(['--in_dir', '/blabla/'])