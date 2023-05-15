# Copyright 2019 the Autoware Foundation
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# This file has been changed and the changes are copyrighted and listed below
#
# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
#
# List of changes:
# * Removing autoware auto cmake dependency
cmake_minimum_required(VERSION 3.5)

### Export headers
project(pure_pursuit_nodes)

## dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  include/pure_pursuit_nodes/pure_pursuit_node.hpp
  include/pure_pursuit_nodes/visibility_control.hpp
  src/pure_pursuit_node.cpp
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${tf2_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

set(PURE_PURSUIT_NODE_EXE pure_pursuit_node_exe)
ament_auto_add_executable(${PURE_PURSUIT_NODE_EXE} src/pure_pursuit_node_main.cpp)

if(BUILD_TESTING)
  # run common linters
  find_package(ament_lint_auto)
  ament_lint_auto_find_test_dependencies()

  set(PURE_PURSUIT_NODE_TEST pure_pursuit_integration_test_exe)
  ament_auto_add_executable(${PURE_PURSUIT_NODE_TEST} test/src/pure_pursuit_integration_test.cpp)

  add_ros_test(test/src/pure_pursuit_node.test.py)
  add_ros_test(
    test/src/pure_pursuit_node_launch.test.py
    TIMEOUT "30"
  )
endif()

## install stuff
ament_auto_package(
  INSTALL_TO_SHARE param
)
