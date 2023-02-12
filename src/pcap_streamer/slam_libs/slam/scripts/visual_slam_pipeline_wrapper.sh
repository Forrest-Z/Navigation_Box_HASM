#!/bin/bash
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

python3 /usr/bin/visual_slam_pipeline.py \
  --visual_odometry_exe="/usr/bin/mai_visual_odometry" \
  --loop_closer_exe="/usr/bin/mai_loop_closer" \
  --in_dir_images="/input" \
  --out_dir="/output" \
  $@
