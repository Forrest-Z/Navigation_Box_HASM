# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
#
# List of changes:
# * Added aiim prefix
# * Replaced test generation function
cmake_minimum_required(VERSION 3.6)

project(aiim_motion_testing)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Build
ament_auto_add_library(${PROJECT_NAME} SHARED src/aiim_motion_testing/motion_testing.cpp)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Linters
  ament_lint_auto_find_test_dependencies()
  # Unit test
  ament_add_gtest(aiim_motion_testing_unit_tests
    test/gtest_main.cpp
    test/constant_trajectory.cpp
    test/trajectory_checks.cpp)
  target_compile_options(aiim_motion_testing_unit_tests PRIVATE -Wno-sign-conversion)
  target_link_libraries(aiim_motion_testing_unit_tests ${PROJECT_NAME})
  add_dependencies(aiim_motion_testing_unit_tests ${PROJECT_NAME})
endif()

ament_auto_package()
