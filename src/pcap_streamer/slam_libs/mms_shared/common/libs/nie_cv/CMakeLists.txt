# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
# CLion / Cmake bug
# https://intellij-support.jetbrains.com/hc/en-us/community/posts/360000457964-Not-Locating-CUDA-Compiler
#set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
#set(CMAKE_CUDA_FLAGS "-arch=sm_61")

# CLion bug. Using the CUDA language results in Clion trying to run an object file.
# https://intellij-support.jetbrains.com/hc/en-us/community/posts/115000705370-clion-tries-to-run-cmake-device-link-o-when-enable-language-cuda-is-used
# https://youtrack.jetbrains.com/issue/CPP-10292
project(nie_cv)

# This is deprecated by project(calibrator LANGUAGES CXX CUDA)
# But for some weird reason then the compiler cannot be found, but it can using the old find_package
# Arch lookup https://en.wikipedia.org/wiki/CUDA
# TODO(jbr) Temporary hack until we have a proper build server that allows for compiling with NVCC
if(WITH_CUDA)
    find_package(CUDA 9.0 REQUIRED)
    list(APPEND CUDA_NVCC_FLAGS "-arch=sm_61;-std=c++14")
    include_directories(SYSTEM ${CUDA_INCLUDE_DIRS})
endif()

set(LIBS nie_core)

file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp" "${CMAKE_CURRENT_LIST_DIR}/src/*.inl")

# TODO(jbr) Temporary hack until we have a proper build server that allows for compiling with NVCC
if(WITH_CUDA)
    file(GLOB_RECURSE SOURCES_CU "${CMAKE_CURRENT_LIST_DIR}/src/*.cu")
endif()

# TODO(jbr) Temporary hack until we have a proper build server that allows for compiling with NVCC
if(WITH_CUDA)
    # Classic cuda cuda_add_library of all the CLion bugs.
    cuda_add_library(${PROJECT_NAME} SHARED ${SOURCES} ${SOURCES_CU})
    #set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_STANDARD 14)
    set_property(TARGET ${PROJECT_NAME} PROPERTY CUDA_SEPARABLE_COMPILATION ON)
    cxx_default_target_properties(${PROJECT_NAME})
else()
    cxx_library_shared(${PROJECT_NAME} ${SOURCES})
endif()
target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src")
target_link_libraries(${PROJECT_NAME} ${LIBS})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

if(BUILD_TESTS)
    nie_build_tests(nie_cv_tests)
    target_link_libraries(nie_cv_tests nie_formats)
endif(BUILD_TESTS)
