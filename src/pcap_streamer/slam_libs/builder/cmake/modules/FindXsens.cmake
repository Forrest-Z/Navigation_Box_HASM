# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
#
# FindXsens.cmake - Find Xsens library.
#
# This module defines the following variables:
#
# Xsens_FOUND: TRUE iff Xsens is found.
# Xsens_INCLUDE_DIRS: Include directories for Xsens.
# Xsens_LIBRARIES: Libraries required to link Xsens.

if(NOT Xsens_FOUND)
    set(Xsens_PACKAGE_NAME "Xsens")
    if(EXISTS "/usr/local/xsens/")
        set(Xsens_DIR
            "/usr/local/xsens/"
            CACHE PATH "Path to the Xsens SDK directory")
    else()
        set(Xsens_DIR
            "Xsens_DIR-NOTFOUND"
            CACHE PATH "Path to the Xsens SDK directory")
        package_report_not_found(${Xsens_PACKAGE_NAME} "Could not find ${Xsens_PACKAGE_NAME} directory: ${Xsens_DIR}")
    endif()

    if(NOT EXISTS "${Xsens_DIR}/include")
        package_report_not_found(${Xsens_PACKAGE_NAME} "Could not find include directory: ${Xsens_DIR}/include")
    endif()

    set(Xsens_INCLUDE_DIRS "${Xsens_DIR}/include")

    if(NOT EXISTS "${Xsens_DIR}/lib")
        package_report_not_found(${Xsens_PACKAGE_NAME} "Could not find binary directory: ${Xsens_DIR}/lib")
    endif()

    #TODO make a function for this that is shared by FindPylon.cmake
    foreach(LIB "xstypes" "xsensdeviceapi")
        find_library(
            Xsens_LIBRARY_${LIB}
            NAMES ${LIB}
            HINTS "${Xsens_DIR}/lib")

        if(${Xsens_LIBRARY_${LIB}} STREQUAL "Xsens_LIBRARY_${LIB}-NOTFOUND")
            package_report_not_found(Xsens_LIBRARY_${LIB} "Could not find library: ${LIB}")
        endif()

        list(APPEND Xsens_LIBRARIES ${Xsens_LIBRARY_${LIB}})
    endforeach()

    set(Xsens_FOUND TRUE)
    message(STATUS "Found Xsens: ${Xsens_DIR}")
else()
    message(STATUS "Found Xsens: ${Xsens_DIR}")
endif()
