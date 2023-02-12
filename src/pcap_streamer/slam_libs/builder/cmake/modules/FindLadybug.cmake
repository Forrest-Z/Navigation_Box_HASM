# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
#[=======================================================================[.rst:
FindLadybug
-----------

Finds the Ladybug SDK

Result Variables
^^^^^^^^^^^^^^^^

This will define the following variables:

``Ladybug_FOUND``
  True if the system has the Ladybug library.
``Ladybug_VERSION``
  The version of the Ladybug library which was found.
``Ladybug_INCLUDE_DIRS``
  Include directories needed to use Ladybug.
``Ladybug_LIBRARIES``
  Libraries needed to link to Ladybug.

#]=======================================================================]

# Try to locate the library with PkgConfig
find_package(PkgConfig)
pkg_check_modules(PC_Ladybug QUIET Ladybug)

# Define the default search paths
set(_Ladybug_SEARCH_PATHS ${PC_Ladybug_INCLUDE_DIRS} ${CMAKE_PREFIX_PATH} ${Ladybug_DIR} $ENV{Ladybug_DIR} /usr
                          /usr/local)

# Find ladybug include directories
unset(Ladybug_INCLUDE_DIRS CACHE)
find_path(
    Ladybug_INCLUDE_DIRS
    NAMES ladybug/ladybug.h
    PATHS ${_Ladybug_SEARCH_PATHS}
    PATH_SUFFIXES include)

# Find Ladybug library
unset(_${COMPONENT}_LIBRARY CACHE)
find_library(
    _${COMPONENT}_LIBRARY
    NAMES ladybug
    PATHS ${_Ladybug_SEARCH_PATH}
    PATH_SUFFIXES lib lib/ladybug)

# If the component was found, add it to the list of ladybug components
if(_${COMPONENT}_LIBRARY)
    list(APPEND Ladybug_LIBRARIES ${_${COMPONENT}_LIBRARY})
endif()

# Try to determine the ladybug library version
if(Ladybug_INCLUDE_DIRS AND Ladybug_LIBRARIES)
    find_file(FindLadybugVersion FindLadybugVersion.cpp HINTS ${CMAKE_MODULE_PATH})

    try_run(
        Ladybug_VERSION_RUN_RESULT Ladybug_VERSION_COMPILE_RESULT ${CMAKE_CURRENT_BINARY_DIR} ${FindLadybugVersion}
        CMAKE_FLAGS -DINCLUDE_DIRECTORIES=${Ladybug_INCLUDE_DIRS} LINK_LIBRARIES ${Ladybug_LIBRARIES}
        RUN_OUTPUT_VARIABLE Ladybug_VERSION_OUTPUT)

    # Check if the version test compiled succesfully
    if(Ladybug_VERSION_COMPILE_RESULT)
        if(Ladybug_VERSION_RUN_RESULT EQUAL 0)
            set(Ladybug_VERSION ${Ladybug_VERSION_OUTPUT})
        else()
            message("** WARNING ** unable to determine ladybug library version: ${Ladybug_VERSION_OUTPUT}")
        endif()
    else()
        message("** WARNING ** unable to compile a simple program using the ladybug library")
    endif()
endif()

# Set Ladybug_FOUND if variables are valid
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    Ladybug
    FOUND_VAR Ladybug_FOUND
    REQUIRED_VARS Ladybug_INCLUDE_DIRS Ladybug_LIBRARIES
    VERSION_VAR Ladybug_VERSION)

if(Ladybug_FOUND)
    set(Ladybug_DEFINITIONS ${PC_Ladybug_CFLAGS_OTHER})
endif()

# compatibility variables
set(Ladybug_VERSION_STRING ${Ladybug_VERSION})
