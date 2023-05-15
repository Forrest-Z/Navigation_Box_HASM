# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
#
# List of changes:
# * Added aiim prefix
# * Replaced test generation function
cmake_minimum_required(VERSION 3.6)

project(aiim_motion_common)

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Build library
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/aiim_motion_common/config.cpp
  src/aiim_motion_common/motion_common.cpp
)

# Test
if(BUILD_TESTING)
  # Linters
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Unit test
  ament_add_gtest(aiim_motion_common_unit_tests
          test/gtest_main.cpp
          test/sanity_checks.cpp
          test/interpolation.cpp)
  target_compile_options(aiim_motion_common_unit_tests PRIVATE -Wno-sign-conversion)
  target_link_libraries(aiim_motion_common_unit_tests ${PROJECT_NAME})
  add_dependencies(aiim_motion_common_unit_tests ${PROJECT_NAME})
  ament_target_dependencies(aiim_motion_common_unit_tests
    "geometry_msgs"
  )
endif()

# Install snippets for code generation
install(DIRECTORY scripts/autogeneration_code_snippets DESTINATION share/)

ament_auto_package()
