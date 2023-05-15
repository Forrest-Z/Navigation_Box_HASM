#!/bin/bash
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

# Halt on error or undefined variables
set -e
set -u

# Ensure the correct number of parameters was specified
if [[ $# -ne 3 ]]; then
    echo "Usage: run_kitti_vo_mono_benchmark <input directory> <output directory> <sequence number>"
    echo ""
    echo "This script runs the visual odometry mono benchmark for a single sequence in the kitti benchmark data set."
    echo "The 3 mandatory positional parameters:"
    echo "  * input directory: the directory that is expected to contain"
    echo "        <input dir>/poses/<sequence number>.txt: the ground truth poses"
    echo "        <input dir>/sequences/<sequence number>/...: the calibration file and images"
    echo "  * output directory: the directory where the results will be written to"
    echo "  * sequence number: The two digits that represent the kitti sequence number"
    exit 2
fi

# Define input and output directory variables for readability
IN_DIR="$1"
OUT_DIR="$2"
SEQ="$3"

# Define and create default output directory structure
IN_FILE_POSES="${IN_DIR}/poses/${SEQ}.txt"
IN_DIR_SEQ="${IN_DIR}/sequences/${SEQ}"
OUT_DIR="${OUT_DIR}/${SEQ}"

echo Creating output directory structure in ${OUT_DIR}
mkdir -p "${OUT_DIR}"

# Run the mono vo benchmark
pushd "$(dirname "$0")" >> /dev/null
python3 ./vo_mono_benchmark.py \
    -format kitti \
    -in_file_intrinsics="${IN_DIR_SEQ}/calib.txt" \
    -in_dir_images="${IN_DIR_SEQ}/image_0" \
    -in_file_ground_truth="${IN_FILE_POSES}" \
    -out_dir_results="${OUT_DIR}" \
    -out_file_name="${SEQ}"
popd >> /dev/null
