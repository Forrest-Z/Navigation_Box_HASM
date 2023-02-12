# This is a modified version of the original ndt_gpu CMakeLists.txt from Autoware.
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
project(ndt_anh_gpu)

set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "")

set(CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};-gencode=arch=compute_50,code=sm_50;-gencode=arch=compute_52,code=sm_52;-gencode=arch=compute_60,code=sm_60;-gencode=arch=compute_61,code=sm_61;-gencode=arch=compute_70,code=sm_70;-gencode=arch=compute_72,code=sm_72;-gencode=arch=compute_70,code=compute_70
)

set(SUBSYS_DESC "Point cloud ndt gpu library")

set(srcs ${CMAKE_CURRENT_LIST_DIR}/src/MatrixDevice.cu 
    ${CMAKE_CURRENT_LIST_DIR}/src/MatrixHost.cu 
    ${CMAKE_CURRENT_LIST_DIR}/src/NormalDistributionsTransform.cu 
    ${CMAKE_CURRENT_LIST_DIR}/src/Registration.cu
    ${CMAKE_CURRENT_LIST_DIR}/src/VoxelGrid.cu 
    ${CMAKE_CURRENT_LIST_DIR}/src/SymmetricEigenSolver.cu)

set(incs
    ${CMAKE_CURRENT_LIST_DIR}/include/ndt_anh_gpu/common.h
    ${CMAKE_CURRENT_LIST_DIR}/include/ndt_anh_gpu/debug.h
    ${CMAKE_CURRENT_LIST_DIR}/include/ndt_anh_gpu/Matrix.h
    ${CMAKE_CURRENT_LIST_DIR}/include/ndt_anh_gpu/MatrixDevice.h
    ${CMAKE_CURRENT_LIST_DIR}/include/ndt_anh_gpu/MatrixHost.h
    ${CMAKE_CURRENT_LIST_DIR}/include/ndt_anh_gpu/NormalDistributionsTransform.h
    ${CMAKE_CURRENT_LIST_DIR}/include/ndt_anh_gpu/Registration.h
    ${CMAKE_CURRENT_LIST_DIR}/include/ndt_anh_gpu/SymmetricEigenSolver.h
    ${CMAKE_CURRENT_LIST_DIR}/include/ndt_anh_gpu/VoxelGrid.h)

cuda_add_library(${PROJECT_NAME} ${srcs} ${incs})

target_include_directories(${PROJECT_NAME} PUBLIC ${PCL_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/include ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${CUDA_LIBRARIES} ${CUDA_curand_LIBRARY} ${PCL_LIBRARIES})

install(
    TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

