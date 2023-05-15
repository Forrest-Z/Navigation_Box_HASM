# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
#[=======================================================================[.rst:
FindPcap
--------

Finds the pcap library, a portable C/C++ library for network traffic capture.
For more information on the pcap library see: http://www.tcpdump.org/

Result Variables
^^^^^^^^^^^^^^^^

This will define the following variables:

``Pcap_FOUND``
  True if the system has the pcap library.
``Pcap_VERSION``
  The version of the pcap library which was found.
``Pcap_INCLUDE_DIRS``
  Include directories needed to use pcap library.
``Pcap_LIBRARIES``
  Libraries needed to link to pcap library.

#]=======================================================================]

# Try to locate the library with PkgConfig
find_package(PkgConfig)
pkg_check_modules(PC_Pcap QUIET Pcap)

# Define the default search paths
set(_Pcap_SEARCH_PATHS ${PC_Pcap_INCLUDE_DIRS} ${CMAKE_PREFIX_PATH} ${Pcap_DIR} $ENV{Pcap_DIR} /usr /usr/local)

# Find pcap include directories
unset(Pcap_INCLUDE_DIRS CACHE)
find_path(
    Pcap_INCLUDE_DIRS
    NAMES pcap/pcap.h pcap.h
    PATHS ${_Pcap_SEARCH_PATHS}
    PATH_SUFFIXES include)

# Find pcap library
if(WIN32)
    set(_Pcap_LIBRARY_NAME wpcap)
else()
    set(_Pcap_LIBRARY_NAME pcap)
endif()

unset(_${COMPONENT}_LIBRARY CACHE)
find_library(
    _${COMPONENT}_LIBRARY
    NAMES ${_Pcap_LIBRARY_NAME}
    PATHS ${_Pcap_SEARCH_PATH}
    PATH_SUFFIXES lib)

# If the component was found, add it to the list of pcap components
if(_${COMPONENT}_LIBRARY)
    list(APPEND Pcap_LIBRARIES ${_${COMPONENT}_LIBRARY})
endif()

# Try to determine the pcap library version
if(Pcap_INCLUDE_DIRS AND Pcap_LIBRARIES)
    find_file(FindPcapVersion FindPcapVersion.cpp HINTS ${CMAKE_MODULE_PATH})

    try_run(
        Pcap_VERSION_RUN_RESULT Pcap_VERSION_COMPILE_RESULT ${CMAKE_CURRENT_BINARY_DIR} ${FindPcapVersion}
        CMAKE_FLAGS -DINCLUDE_DIRECTORIES=${Pcap_INCLUDE_DIRS} LINK_LIBRARIES ${Pcap_LIBRARIES}
        RUN_OUTPUT_VARIABLE Pcap_VERSION_OUTPUT)

    # Check if the version test compiled succesfully
    if(Pcap_VERSION_COMPILE_RESULT)
        if(Pcap_VERSION_RUN_RESULT EQUAL 0)
            set(Pcap_VERSION ${Pcap_VERSION_OUTPUT})
        else()
            message("** WARNING ** unable to determine pcap library version: ${Pcap_VERSION_OUTPUT}")
        endif()
    else()
        message("** WARNING ** unable to compile a simple program using the pcap library")
    endif()
endif()

# Set Pcap_FOUND if variables are valid
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    Pcap
    FOUND_VAR Pcap_FOUND
    REQUIRED_VARS Pcap_INCLUDE_DIRS Pcap_LIBRARIES
    VERSION_VAR Pcap_VERSION)

if(Pcap_FOUND)
    set(Pcap_DEFINITIONS ${PC_Pcap_CFLAGS_OTHER})
endif()

# compatibility variables
set(Pcap_VERSION_STRING ${Pcap_VERSION})
