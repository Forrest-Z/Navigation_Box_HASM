# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
# 2-5-2018 Nikolas
#
# FindTensorflow.cmake - Find Tensorflow includes files and libraries.
#
# This module defines the following variables:
#
# Tensorflow_FOUND: TRUE iff tensorflow is found.
# Tensorflow_INCLUDE_DIRS: Include directories for Tensorflow.
# Tensorflow_LIBRARIES: Libraries required to link Tensorflow.

if(NOT Tensorflow_FOUND)
    set(Tensorflow_PACKAGE_NAME "Tensorflow")

    foreach(DIRECTORY "/usr/local/include" "/usr/local/include/eigen3" "/usr/local/include/google/tensorflow")
        if(NOT EXISTS "${DIRECTORY}")
            package_report_not_found(${Tensorflow_PACKAGE_NAME}
                                     "Could not find include directory: ${Tensorflow_DIR}/include")
        endif()

        list(APPEND Tensorflow_INCLUDE_DIRS ${DIRECTORY})
    endforeach()

    foreach(LIB "tensorflow_cc")
        find_library(TENSORFLOW_${LIB} NAMES ${LIB})

        if(TENSORFLOW_${LIB}_NOTFOUND)
            package_report_not_found(${Tensorflow_PACKAGE_NAME} "Could not find library: ${LIB}")
        endif()

        list(APPEND Tensorflow_LIBRARIES ${TENSORFLOW_${LIB}})
    endforeach()

    set(Tensorflow_FOUND TRUE)
    message(STATUS "Found Tensorflow")
else()
    message(STATUS "Found Tensorflow")
endif()
