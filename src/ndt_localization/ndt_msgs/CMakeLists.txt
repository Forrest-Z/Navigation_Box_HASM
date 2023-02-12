# This is a modified version of the original ndt_msgs CMakeLists.txt from Autoware.
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
# * Renamed module from ndt_msgs to ndt_msgs
# * Removed NDTConfig.msg
# * Upgraded to ROS2

cmake_minimum_required(VERSION 3.5)
project(ndt_msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/NdtStat.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}    
    DEPENDENCIES 
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)


ament_package()