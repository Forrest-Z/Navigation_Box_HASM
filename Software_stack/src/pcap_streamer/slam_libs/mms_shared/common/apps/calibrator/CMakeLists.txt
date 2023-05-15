# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
# Version 3.8 is needed for cmake supporting CUDA as a built in language.
#cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
cmake_minimum_required(VERSION 3.4 FATAL_ERROR)

project(calibrator)

find_package(OpenCV 4.5.0 REQUIRED)
find_package(Ceres REQUIRED)
find_package(CUDA 9.0 REQUIRED)
find_package(Boost 1.58.0 COMPONENTS date_time)

list(APPEND CUDA_NVCC_FLAGS "-arch=sm_61;-std=c++14")
# now that we also build against a bunch of aicv modules we do this to remove
# potential duplicates of -arch=sm_61 which are set from the outside, as NVCC
# fails upon encountering duplicate compiler flags
list(REMOVE_DUPLICATES CUDA_NVCC_FLAGS)

set(LIBS ${LIBS} nie_cv nie_formats ${Boost_LIBRARIES})

file(GLOB_RECURSE SOURCES_CPP "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp" "${CMAKE_CURRENT_LIST_DIR}/src/*.cu")

# Classic cuda cuda_add_executable of all the CLion bugs.
cuda_add_executable(${PROJECT_NAME} ${SOURCES_CPP})
#set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_STANDARD 14)
set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_SEPARABLE_COMPILATION ON)
cxx_default_target_properties(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} ${LIBS})
