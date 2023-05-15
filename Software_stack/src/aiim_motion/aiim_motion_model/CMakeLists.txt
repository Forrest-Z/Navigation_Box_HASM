# Copyright 2021 Apex.AI, Inc.
# All rights reserved.
#
# This file has been changed and the changes are copyrighted and listed below
#
# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
#
# List of changes:
# * Adding aiim prefix
cmake_minimum_required(VERSION 3.5)
project(aiim_motion_model)

#dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

# build library
ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/linear_motion_model.cpp
  src/differential_drive_motion_model.cpp)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${EIGEN3_INCLUDE_DIR})

if(BUILD_TESTING)
  set(MOTION_MODEL_GTEST motion_model_gtest)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # gtest
  ament_add_gtest(${MOTION_MODEL_GTEST}
                  test/test_linear_motion_model.cpp
                  test/test_differential_motion_model.cpp)
  target_include_directories(${MOTION_MODEL_GTEST} PRIVATE "test/include" "include")
  target_link_libraries(${MOTION_MODEL_GTEST} ${PROJECT_NAME})
  ament_target_dependencies(${MOTION_MODEL_GTEST} "aiim_autoware_common")
endif()

# Ament Exporting
ament_export_include_directories(${EIGEN3_INCLUDE_DIR})
ament_auto_package()
