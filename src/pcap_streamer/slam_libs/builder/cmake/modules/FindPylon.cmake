# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
# 1-4-2018 Jonathan
#
# FindPylon.cmake - Find Basler Pylon library.
#
# This module defines the following variables:
#
# Pylon_FOUND: TRUE iff gflags is found.
# Pylon_INCLUDE_DIRS: Include directories for Pylon.
# Pylon_LIBRARIES: Libraries required to link Pylon.

if(NOT Pylon_FOUND)
    set(Pylon_PACKAGE_NAME "Pylon")
    if(EXISTS "/opt/pylon5/")
        set(Pylon_DIR
            "/opt/pylon5/"
            CACHE PATH "Path to the Pylon (Basler) SDK directory")
    else()
        set(Pylon_DIR
            "Pylon_DIR-NOTFOUND"
            CACHE PATH "Path to the Pylon (Basler) SDK directory")
        package_report_not_found(${Pylon_PACKAGE_NAME} "Could not find ${Pylon_PACKAGE_NAME} directory: ${Pylon_DIR}")
    endif()

    if(NOT EXISTS "${Pylon_DIR}/include")
        package_report_not_found(${Pylon_PACKAGE_NAME} "Could not find include directory: ${Pylon_DIR}/include")
    endif()

    set(Pylon_INCLUDE_DIRS "${Pylon_DIR}/include")

    if(NOT EXISTS "${Pylon_DIR}/lib64")
        package_report_not_found(${Pylon_PACKAGE_NAME} "Could not find binary directory: ${Pylon_DIR}/lib64")
    endif()

    file(GLOB Pylon_LIBRARIES "${Pylon_DIR}/lib64/*GCBase_gcc_*" "${Pylon_DIR}/lib64/*GenApi_gcc_*")

    list(LENGTH Pylon_LIBRARIES RES_LEN)
    if(NOT RES_LEN EQUAL 2)
        message(SEND_ERROR "Could not find one of the following library patterns: *GCBase_gcc_* *GenApi_gcc_*")        
    endif()

    foreach(LIB "pylonbase" "pylonutility" "pylonc")
        find_library(
            Pylon_LIBRARY_${LIB}
            NAMES ${LIB}
            HINTS "${Pylon_DIR}/lib64")

        if(Pylon_LIBRARY_${LIB}_NOTFOUND)
            package_report_not_found(${Pylon_PACKAGE_NAME} "Could not find library: ${LIB}")
        endif()

        list(APPEND Pylon_LIBRARIES ${Pylon_LIBRARY_${LIB}})
    endforeach()

    set(Pylon_FOUND TRUE)
    message(STATUS "Found Pylon: ${Pylon_DIR}")
else()
    message(STATUS "Found Pylon: ${Pylon_DIR}")
endif()
