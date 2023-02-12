#!/bin/bash
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

# Halt on error or undefined variables
set -e
set -u


declare -A sequences
sequences[00]="2011_10_03_drive_0027"
sequences[01]="2011_10_03_drive_0042"
sequences[02]="2011_10_03_drive_0034"
sequences[04]="2011_09_30_drive_0016"
sequences[05]="2011_09_30_drive_0018"
sequences[06]="2011_09_30_drive_0020"
sequences[07]="2011_09_30_drive_0027"
sequences[08]="2011_09_30_drive_0028"
sequences[09]="2011_09_30_drive_0033"
sequences[10]="2011_09_30_drive_0034"

declare -A sequences_ranges
sequences_ranges[00]="000006-004542"
sequences_ranges[01]="000006-001100"
sequences_ranges[02]="000006-004662"
sequences_ranges[04]="000004-000272"
sequences_ranges[05]="000006-002762"
sequences_ranges[06]="000007-001102"
sequences_ranges[07]="000004-001100"
sequences_ranges[08]="001109-005172"
sequences_ranges[09]="000012-001592"
sequences_ranges[10]="000008-001202"


BASE_DATA_DIR="/volumes2/Data sets/KITTI"
BASE_WORK_DIR="/volumes1/workarea/MMS-1731/pipeline_final"

for SEQ in ${!sequences[*]}; do
    now=$(date +"%T")
    echo "========================================"
    echo "=== Starting sequence $SEQ at $now ==="
    echo "========================================"

    WORK_DIR="${BASE_WORK_DIR}/${SEQ}"
    WORK_DIR_INT="${WORK_DIR}/intermediate"

    echo "Creating output directory structure in '${WORK_DIR}'."
    mkdir -p "${BASE_WORK_DIR}/${SEQ}"

    # Define and create default output directory structure
    NAME="${sequences[$SEQ]}"
    DATE="${NAME:0:10}"

    pushd "$(dirname "$0")" >> /dev/null

    ./kitti_pose_interpolator \
        --in_file_pose="${WORK_DIR_INT}/gps.pose" \
        --in_file_times="${BASE_DATA_DIR}/raw/${DATE}/${NAME}_extract/image_00/timestamps.txt" \
        --in_file_calib_imu_to_lidar="${BASE_DATA_DIR}/raw/${DATE}/calib_imu_to_velo.txt" \
        --in_file_calib_lidar_to_cam="${BASE_DATA_DIR}/raw/${DATE}/calib_velo_to_cam.txt" \
        --in_file_ground_truth="${BASE_DATA_DIR}/dataset/poses/${SEQ}.txt" \
        --range="${sequences_ranges[$SEQ]}" \
        --ground_truth=true \
        --out_file_pose="${WORK_DIR_INT}/gps_resampled.pose" \
        --logtostderr \
        --v=3

    ./pose_tool \
        --print \
        --to_file \
        --in_paths="${WORK_DIR_INT}/gps_resampled.pose" \
        --out_paths="${WORK_DIR_INT}/gps_resampled.pose.txt" \
        --logtostderr \
        --v=3

    ./kitti_benchmarker \
        --in_file_reference="${BASE_DATA_DIR}/dataset/poses/${SEQ}.txt" \
        --in_file_test="${WORK_DIR_INT}/gps_resampled.pose" \
        --out_dir_result="${WORK_DIR}/benchmark_gps" \
        --out_file_name="gps" \
        --logtostderr \
        --v=3

    popd >> /dev/null
done
