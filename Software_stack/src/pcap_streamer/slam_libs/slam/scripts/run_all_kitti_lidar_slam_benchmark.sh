#!/bin/bash
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

# Halt on error or undefined variables
set -u

BASE_DATA_DIR="/data/aiim/KITTI"
WORK_DIR="/volumes1/workarea/MMS-1731/pipeline_final"
GENERATE_LAS=1  # 0 = false, 1 = true

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
sequences_ranges[00]="000004-004541"
sequences_ranges[01]="000005-001100"
sequences_ranges[02]="000004-004661"
sequences_ranges[04]="000002-000271"
sequences_ranges[05]="000004-002761"
sequences_ranges[06]="000005-001101"
sequences_ranges[07]="000002-001100"
sequences_ranges[08]="001105-005171"
sequences_ranges[09]="000010-001591"
sequences_ranges[10]="000006-001201"


RAW_DATA_DIR="${BASE_DATA_DIR}/raw_data/extract"

for SEQ in ${!sequences[*]}; do
    now=$(date +"%T")
    echo "========================================"
    echo "=== Starting sequence $SEQ at $now ==="
    echo "========================================"

    # Define and create default output directory structure
    NAME="${sequences[$SEQ]}"
    DATE="${NAME:0:10}"

    OUT_DIR="${WORK_DIR}/${SEQ}"
    echo "Creating output directory structure in '${OUT_DIR}'."
    mkdir -p "${OUT_DIR}"

    # Run the lidar slam benchmark
    pushd "$(dirname "$0")" >> /dev/null

    # Build command to execute
    command=(python3 \
        ./kitti_lidar_slam_benchmark.py \
        --in_dir="${RAW_DATA_DIR}/${DATE}/${NAME}_extract" \
        --in_file_calib_imu_to_lidar="${RAW_DATA_DIR}/${DATE}/calib_imu_to_velo.txt" \
        --in_file_calib_lidar_to_cam="${RAW_DATA_DIR}/${DATE}/calib_velo_to_cam.txt" \
        --in_file_ground_truth_times="${RAW_DATA_DIR}/${DATE}/${NAME}_extract/image_00/timestamps.txt" \
        --in_file_ground_truth_poses="${BASE_DATA_DIR}/odometry/dataset/poses/${SEQ}.txt" \
        --range="${sequences_ranges[$SEQ]}" \
        --out_dir="${OUT_DIR}" \
        --debug_level=2)
    if (( GENERATE_LAS )); then
        command+=(--generate_las)
    fi
    # Actually execute command
    "${command[@]}" &> "${OUT_DIR}/log.txt"

    popd >> /dev/null
done
