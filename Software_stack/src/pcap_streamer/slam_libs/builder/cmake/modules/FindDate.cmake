# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
#[=======================================================================[.rst:
FindDate
--------

Finds the date library, a portable C/C++ library that is the canonical implementation
of the C++20 additions to std::chrono

For more information on the date library see: https://github.com/HowardHinnant/date/wiki

Result Variables
^^^^^^^^^^^^^^^^

This will define the following variables:

``Date_FOUND``
  True if the system has the date library.
``Date_INCLUDE_DIRS``
  Include directories needed to use date library.
``Date_LIBRARIES``
  Libraries needed to link to date library.
``Date_DEFINITIONS``
  Definitions needed to pass to compiler to use date library.

#]=======================================================================]

# Define the default search paths
set(_Date_SEARCH_PATHS ${CMAKE_PREFIX_PATH} ${Date_DIR} $ENV{Date_DIR} /usr /usr/local)

# Find date / tz include directories
unset(Date_INCLUDE_DIRS CACHE)
find_path(
    Date_INCLUDE_DIRS
    NAMES date/date.h
    PATHS ${_Date_SEARCH_PATHS}
    PATH_SUFFIXES include)

# Find date / tz libraries
unset(Date_LIBRARIES CACHE)
find_library(
    Date_LIBRARIES
    NAMES tz
    PATHS ${_Date_SEARCH_PATH}
    PATH_SUFFIXES lib)

# Set compile definitions
# NOTE: The compile definitions must be the same as the compile definitions with which the date library was compiled
#       Unfortunately, these settings are currently NOT exported, see: https://github.com/HowardHinnant/date/issues/313
#       Failing to set the correct compile defintions can change the library ABI, which can lead to link errors, or
#       even worse, segfaults when trying to access timezones, see: https://github.com/HowardHinnant/date/issues/324
set(Date_DEFINITIONS USE_OS_TZDB=1 HAS_STRING_VIEW=OFF)

# Set Date_FOUND if variables are valid
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    Date
    FOUND_VAR Date_FOUND
    REQUIRED_VARS Date_INCLUDE_DIRS Date_LIBRARIES Date_DEFINITIONS)
