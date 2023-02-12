# This is a modified version of the original pcl_omp_registration CMakeLists.txt from Autoware.
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
project(ndt_pcl_openmp)

if(PCL_VERSION VERSION_LESS "1.7.2")
    message("pcl_omp requires PCL 1.7.2 or higher versions")
else(PCL_VERSION VERSION_LESS "1.7.2")

    set(srcs ${CMAKE_CURRENT_LIST_DIR}/src/ndt.cpp)

    set(incs ${CMAKE_CURRENT_LIST_DIR}/include/ndt_pcl_openmp/ndt.h ${CMAKE_CURRENT_LIST_DIR}/include/ndt_pcl_openmp/registration.h)

    set(impl_incs ${CMAKE_CURRENT_LIST_DIR}/include/ndt_pcl_openmp/impl/ndt.hpp ${CMAKE_CURRENT_LIST_DIR}/include/ndt_pcl_openmp/impl/registration.hpp)

    add_library(${PROJECT_NAME} ${srcs} ${incs} ${impl_incs})

    target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
    
    install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
endif(PCL_VERSION VERSION_LESS "1.7.2")
