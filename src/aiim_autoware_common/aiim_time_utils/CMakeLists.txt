# The changes made in this file, of which a summary is listed below, are copyrighted:
#
# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
#
# List of changes:
# * Add aiim prefix to time_utils
cmake_minimum_required(VERSION 3.6)

project(aiim_time_utils)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Build
ament_auto_add_library(${PROJECT_NAME} SHARED src/aiim_time_utils/time_utils.cpp)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Linters
  ament_lint_auto_find_test_dependencies()
  # TODO(c.ho) unit tests
endif()

ament_auto_package()
