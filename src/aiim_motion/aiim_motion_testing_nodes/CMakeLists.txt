# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
#
# List of changes:
# * Added aiim prefix
cmake_minimum_required(VERSION 3.6)

project(aiim_motion_testing_nodes)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### Build
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/aiim_motion_testing_nodes/motion_testing_publisher.cpp
  src/aiim_motion_testing_nodes/wait_for_matched.cpp)

### Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Linters
  ament_lint_auto_find_test_dependencies()
  # Unit test
  ament_add_gtest(motion_testing_nodes_unit_tests
    test/gtest_main.cpp
    test/periodic_publisher.cpp
    test/sanity_check.cpp)
  target_compile_options(motion_testing_nodes_unit_tests PRIVATE -Wno-sign-conversion -Wno-ignored-qualifiers)
  target_link_libraries(motion_testing_nodes_unit_tests ${PROJECT_NAME} gtest)
  add_dependencies(motion_testing_nodes_unit_tests ${PROJECT_NAME})
endif()

ament_auto_package()
