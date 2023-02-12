#!/bin/bash
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

WORK_DIR="/volumes1/workarea/MMS-897/test_full_kitti"

for SEQ in {00..10}; do
    now=$(date +"%T")
    echo "========================================"
    echo "=== Starting sequence $SEQ at $now ==="
    echo "========================================"

    mkdir -p "${WORK_DIR}/${SEQ}"

    ./run_kitti_vo_mono_benchmark.sh \
	      "/volumes2/Data sets/KITTI/dataset" "${WORK_DIR}"  "${SEQ}" \
	      &> "${WORK_DIR}/${SEQ}/log.txt"
done
