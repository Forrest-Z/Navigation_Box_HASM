# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

get_filename_component(NIE_DIR ${CMAKE_CURRENT_LIST_DIR}/../../ ABSOLUTE)
set(Boost_FIND_QUIETLY "TRUE")
#used by find_package for our custom FindXXX.cmake files
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/modules/")

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

#TODO BUILD_TESTS is deprecated, we should use Cmake's own BUILD_TESTING (from CTest)
#which ties in with enable_testing()
#More info : https:  // cmake.org/cmake/help/latest/module/CTest.html
set(BUILD_TESTS
    "TRUE"
    CACHE BOOL "Build with tests enabled")

#TODO Use CMakeDependentOption
set(BUILD_GMOCK
    "TRUE"
    CACHE BOOL "Build Gtest and GMock. WARNING: Use BUILD_TESTS to manage googletest/gmock builds")
set(BUILD_GTEST
    "TRUE"
    CACHE BOOL "Build Gtest and GMock. WARNING: Use BUILD_TESTS to manage googletest/gmock builds")

set(GOOGLETEST_DIR
    "/usr/src/googletest"
    CACHE PATH "Googletest root dir")
set(NIE_WARNING_FLAGS
    "-Wall -Wextra -pedantic"
    CACHE STRING "Default warning settings")

set(NIE_PYTHON_PACKAGES "") # Will hold list of installed local Python packages

if(BUILD_TESTS)
    enable_testing()
    if(NOT TARGET gtest_main)

        #Configs for googletest
        set(INSTALL_GTEST
            "FALSE" BOOL
            "Enable installation of googletest. (Projects embedding googletest may want to turn this OFF.)")

        #TODO it is desirable to hide it, but it is disabled due to cmake_policy CMP0077 warning
        #set(gtest_hide_internal_symbols "TRUE" CACHE BOOL "We don't need internal symbols")
        set(BUILD_SHARED_LIBS
            "FALSE"
            CACHE BOOL "Prefer static build")

        add_subdirectory(${GOOGLETEST_DIR} ${CMAKE_BINARY_DIR}/googletest EXCLUDE_FROM_ALL)

        #Mark GTest variables as advanced
        mark_as_advanced(
            FORCE
            BUILD_GMOCK
            BUILD_GTEST
            gmock_build_tests
            gtest_build_samples
            gtest_build_tests
            gtest_disable_pthreads
            gtest_force_shared_crt
            gtest_hide_internal_symbols)
        #Also BUILD_SHARED_LIBS(GTest forces as cache variable)
        mark_as_advanced(FORCE BUILD_SHARED_LIBS)

        set(WITH_COVERAGE
            "FALSE"
            CACHE BOOL "Build with test coverage")
        if(WITH_COVERAGE)
            set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
        endif()

        include("${NIE_DIR}/builder/cmake/find_python_tests.cmake")
    endif()
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE
        "Release"
        CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

#helper functions
include("${NIE_DIR}/builder/cmake/build_functions.cmake")
include("${NIE_DIR}/builder/cmake/cxx_build_wrappers.cmake")
include("${NIE_DIR}/builder/cmake/versioning.cmake")
