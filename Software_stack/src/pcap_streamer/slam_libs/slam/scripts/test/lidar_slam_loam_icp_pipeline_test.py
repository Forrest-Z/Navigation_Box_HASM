#!/usr/bin/python3
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

import pytest
import sys
import os
import argparse
import pytest_check as check

sys.path.append(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), "..")) #to discover the tested files

import lidar_slam_loam_icp_pipeline as pipeline

def test_smoke(tmpdir):
    applications_path = os.path.join(os.getcwd(), "bin")
    pipeline.main(['--in_file_post=/data/aiim/unit_tests_data/slam_smoke_test/1002-1-010-181212.aiim',
                   '--in_file_calib_extr=/data/aiim/unit_tests_data/slam_smoke_test/HDLCalPara-10-181201.EP',
                   '--in_file_calib_intr=/data/aiim/unit_tests_data/slam_smoke_test/HDL-32E-10-181201.xml',
                   '--in_dir_pcap=/data/aiim/unit_tests_data/slam_smoke_test/Pointcloud/',
                   f'--out_dir={tmpdir}',
                   '--debug_level=2',
                   f'--applications_path={applications_path}'])
                                
    #Check output files are correct
    debug_folder = os.path.join(tmpdir, 'debug')
    check.is_true(os.path.isdir(debug_folder))
    check.is_true(os.listdir(debug_folder)) # i.e. not empty
    
    intermediate_folder = os.path.join(tmpdir, 'intermediate')
    check.is_true(os.path.isdir(intermediate_folder))
    check.is_true(os.listdir(intermediate_folder))
    
    las_folder = os.path.join(tmpdir, 'las')
    check.is_true(os.path.isdir(las_folder))
    check.is_true(os.listdir(las_folder))
    
    lo_folder = os.path.join(tmpdir, 'lo')
    check.is_true(os.path.isdir(lo_folder))
    check.is_true(os.listdir(lo_folder))

    file = os.path.join(tmpdir, 'ba_filtered.pose')
    check.is_true(os.path.isfile(file))
    check.greater(os.path.getsize(file), 0)
    
    file = os.path.join(tmpdir, 'full_problem.pose')
    check.is_true(os.path.isfile(file))
    check.greater(os.path.getsize(file), 0)
    
    file = os.path.join(tmpdir, 'lo.iref')
    check.is_true(os.path.isfile(file))
    check.greater(os.path.getsize(file), 0)
    
    file = os.path.join(tmpdir, 'slam.iref')
    check.is_true(os.path.isfile(file))
    check.greater(os.path.getsize(file), 0)
    
    file = os.path.join(tmpdir, '1002-1-010-181212.csv')
    check.is_true(os.path.isfile(file))
    check.greater(os.path.getsize(file), 0)
    
