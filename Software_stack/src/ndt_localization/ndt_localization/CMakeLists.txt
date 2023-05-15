# This is a modified version of the original ndt_matching CMakeLists.txt from Autoware.
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
# * Renamed modules
# * Restructured CMake structure to one main and childs for the libraries
# * Added conditional building for OpenMP and CUDA if found
# * Require exact matches for PCL and CUDA versions to avoid errors
# * Added console output to notify the user about the status
# * Removed any unnecessary includes/links
# * Complete list of dependent catkin packages
# * Upgraded to ROS2.0

cmake_minimum_required(VERSION 3.12)
project(ndt_localization)

# PCL
find_package(PCL REQUIRED QUIET)
if(PCL_FOUND)
    message(STATUS "PCL library status:")
    message(STATUS "    version: ${PCL_VERSION}")
    message(STATUS "    modules: ${PCL_COMPONENTS}")
    message(STATUS "    include path: ${PCL_INCLUDE_DIRS}")
endif()

# Eigen
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES}) # Not strictly necessary as Eigen is
                                             # header only
    # Possibly map additional variables to the EIGEN3_ prefix.
else()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
message(
    WARNING
        "Adding 'EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT' macro to prevent ndt_matching's runtime error in debug mode.
  The bug reasons and solutions are written in http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html .
  This workaround was discussed on https://gitlab.com/autowarefoundation/autoware.ai/core_perception/merge_requests/57 ."
)
add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

# ROS2 Dependencies
find_package(ament_cmake REQUIRED)
find_package(aiim_ros_cmake REQUIRED)

# ##############################################################################
# Optimization libraries
# ##############################################################################
# NDT PCL Generic is included in PCL by default

# NDT ANH CPU
add_subdirectory(ndt_anh_cpu)
set(OPTIMIZATION_LIBRARIES "${OPTIMIZATION_LIBRARIES};ndt_anh_cpu")

# NDT ANH GPU
find_package(CUDA 11.2 EXACT)
if(CUDA_FOUND)
    message("CUDA found")
    add_definitions(-DUSE_CUDA)
    add_subdirectory(ndt_anh_gpu)
    set(OPTIMIZATION_LIBRARIES "${OPTIMIZATION_LIBRARIES};ndt_anh_gpu")
    set(OPTIMIZATION_DIRECTORIES "${OPTIMIZATION_DIRECTORIES};${CUDA_INCLUDE_DIRS}")
    
    message("Adding ndt_anh_gpu to the optimization libraries.")
else()
    message(WARNING "ndt_anh_gpu will not be built, CUDA was not found.")
endif()

# NDT PCL OpenMP
find_package(OpenMP)
if(OPENMP_FOUND)
    add_definitions(-DUSE_PCL_OPENMP)
    add_subdirectory(ndt_pcl_openmp)
    set(OPTIMIZATION_LIBRARIES "${OPTIMIZATION_LIBRARIES};ndt_pcl_openmp")
    message("Adding ndt_pcl_openmp to the optimization libraries.")
else()
    message(WARNING "ndt_pcl_openmp will not be built, OpenMP was not found.")
endif()

# ##############################################################################
# Build
# ##############################################################################

set(srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/ndt_matching.cpp 
    ${CMAKE_CURRENT_LIST_DIR}/src/ndt_matching_types.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/ndt_matching_trajectory_estimator.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/ndt_matching_pose_prediction.cpp)

set(ros_dependencies
    aiim_autoware_msgs
    geometry_msgs
    pcl_conversions
    rclcpp
    sensor_msgs
    std_msgs
    tf2
    tf2_ros
    nav_msgs
    ndt_msgs)

# Create ros node
add_executable(${PROJECT_NAME} ${srcs})
aiim_auto_ros_dependencies(${PROJECT_NAME} ${ros_dependencies})
target_include_directories(${PROJECT_NAME} PUBLIC include ${PCL_INCLUDE_DIRS} ${OPTIMIZATION_DIRECTORIES}
                                                  ${EIGEN3_INCLUDE_DIRS})
aiim_target_set_default_properties(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} ${OPTIMIZATION_LIBRARIES})

# ##############################################################################
# Tests
# ##############################################################################

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(
        ndt_localization_tests
        ${CMAKE_CURRENT_LIST_DIR}/test/ndt_matching_trajectory_estimator_test.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/ndt_matching_types.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/ndt_matching_trajectory_estimator.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/ndt_matching_pose_prediction.cpp)
    aiim_auto_ros_dependencies(ndt_localization_tests ${ros_dependencies})
    target_link_libraries(ndt_localization_tests gtest_main)
    target_include_directories(
        ndt_localization_tests PUBLIC include ${PCL_INCLUDE_DIRS} ${OPTIMIZATION_DIRECTORIES}
                                       ${EIGEN3_INCLUDE_DIRS})
endif()

# ##############################################################################
# Install
# ##############################################################################

aiim_target_ament_install(${PROJECT_NAME})

ament_package()
