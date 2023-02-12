# This is a modified version of the original ndt_cpu CMakeLists.txt from Autoware.
# The original is licensed under the Apache License, Version 2.0. For more information
# see the LICENSE file in the repository root
#
# The changes made in this file, of which a summary is listed below, are copyrighted:
#
# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
#
# List of changes:
# * Renamed modules
# * Restructured CMake structure to one main and childs for the libraries
# * Changed to CMAKE_CURRENT_LIST_DIR to allow including as subdirectory
# * Moved find_package and others to the main CMakeLists.txt
# * Removed any unnecessary includes/links

cmake_minimum_required(VERSION 2.8.3)
project(ndt_anh_cpu)

set(srcs 
    ${CMAKE_CURRENT_LIST_DIR}/src/NormalDistributionsTransform.cpp 
    ${CMAKE_CURRENT_LIST_DIR}/src/Registration.cpp 
    ${CMAKE_CURRENT_LIST_DIR}/src/VoxelGrid.cpp 
    ${CMAKE_CURRENT_LIST_DIR}/src/Octree.cpp)

set(incs
    ${CMAKE_CURRENT_LIST_DIR}/include/ndt_anh_cpu/debug.h 
    ${CMAKE_CURRENT_LIST_DIR}/include/ndt_anh_cpu/NormalDistributionsTransform.h
    ${CMAKE_CURRENT_LIST_DIR}/include/ndt_anh_cpu/Registration.h 
    ${CMAKE_CURRENT_LIST_DIR}/include/ndt_anh_cpu/SymmetricEigenSolver.h 
    ${CMAKE_CURRENT_LIST_DIR}/include/ndt_anh_cpu/VoxelGrid.h
    ${CMAKE_CURRENT_LIST_DIR}/include/ndt_anh_cpu/Octree.h)

add_library(${PROJECT_NAME} ${incs} ${srcs})
target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/include ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME} ${ROS_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
set_target_properties(${PROJECT_NAME}
   PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)