# This is a modified version of the original NDTStat.msg from Autoware.
# The original is licensed under the Apache License, Version 2.0. For more information
# see the LICENSE file in the repository root
#
# The changes made in this file, of which a summary is listed below, are copyrighted:
#
# Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
#
# List of changes:
# * Removed use_predict_pose
# * Added converged, align_time, fitness_time, response_time, latency, tf_probability, map_time
# * Renamed score to fitness_score
# * Renamed exe_time to processing_time
# * Upgraded to ROS2

std_msgs/Header header
bool converged
float32 response_time
float32 map_time
float32 latency
float32 processing_time
float32 align_time
float32 fitness_time
int32 iteration
float32 tf_probability
float32 fitness_score
float32 velocity
float32 yaw_rate
float32 acceleration
