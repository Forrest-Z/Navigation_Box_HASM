# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

# The nie_build_tests function allows a custom name for the test target. If no argument is passed, the name will
# be equal to the last target for which the project(<target_name>) was called, post-fixed with "_test".
# I.e.: "${PROJECT_NAME}_tests"
function(NIE_BUILD_TESTS)
    # Check if a custom target name was requested, if not, use the default _test one.
    if(${ARGC} EQUAL 1)
        set(TESTS_PROJECT_NAME "${ARGV0}")
    else()
        set(TESTS_PROJECT_NAME "${PROJECT_NAME}_tests")
    endif()

    if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/test/)
        message(WARNING "The ${CMAKE_CURRENT_LIST_DIR}/test/ directory does not exist")
        return()
    endif()

    file(GLOB_RECURSE TESTS_CPP "${CMAKE_CURRENT_LIST_DIR}/test/*_test*.cpp")

    if(TESTS_CPP STREQUAL "")
        message(
            WARNING
                "No source files with valid format <test-identification>_test*.cpp found in ${CMAKE_CURRENT_LIST_DIR}/test/"
        )
        return()
    endif()

    # A variable PROJECT_TARGET_LINK_LIBRARIES is created that contains the libraries required to build the test
    # executable. If the target project is a library then we simply inherit its own dependencies by using it directly.
    # If it is an executable the relevant libraries are checked using get_target_property(... LINK_LIBRARIES).
    get_target_property(PROJECT_TARGET_TYPE ${PROJECT_NAME} TYPE)
    if(${PROJECT_TARGET_TYPE} STREQUAL "EXECUTABLE")
        get_target_property(PROJECT_TARGET_INCLUDE_DIRECTORIES ${PROJECT_NAME} INCLUDE_DIRECTORIES)
        get_target_property(PROJECT_TARGET_LINK_LIBRARIES ${PROJECT_NAME} LINK_LIBRARIES)
        # glob all source files in /src/ dir
        file(GLOB_RECURSE SRC_CPP "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp")
        # Remove file containing main()
        list(REMOVE_ITEM SRC_CPP "${CMAKE_CURRENT_LIST_DIR}/src/${PROJECT_NAME}.cpp")
        # Append source files for compilation
        list(APPEND TESTS_CPP "${SRC_CPP}")
    else()
        set(PROJECT_TARGET_LINK_LIBRARIES ${PROJECT_NAME})
        # PROJECT_TARGET_INCLUDE_DIRECTORIES comes from PROJECT_TARGET_LINK_LIBRARIES
    endif()

    cxx_executable(${TESTS_PROJECT_NAME} "${TESTS_CPP}")
    target_include_directories(${TESTS_PROJECT_NAME} SYSTEM PRIVATE ${PROJECT_TARGET_INCLUDE_DIRECTORIES})
    target_link_libraries_system(${TESTS_PROJECT_NAME} ${PROJECT_TARGET_LINK_LIBRARIES} gtest gtest_main
                                 ${CMAKE_THREAD_LIBS_INIT})

    include(GoogleTest REQUIRED)
    gtest_add_tests(${TESTS_PROJECT_NAME} "" AUTO)

endfunction()

# Called if we failed to find PACKAGE or any of it's required dependencies,
# unsets all public (designed to be used externally) variables and reports
# error message at priority depending upon [REQUIRED/QUIET/<NONE>] argument.
macro(PACKAGE_REPORT_NOT_FOUND PACKAGE REASON_MSG)
    unset(${PACKAGE}_FOUND)
    unset(${PACKAGE}_INCLUDE_DIRS)
    unset(${PACKAGE}_LIBRARIES)

    # Note <package>_FIND_[REQUIRED/QUIETLY] variables defined by FindPackage()
    # use the camelcase library name, not uppercase.
    if(${PACKAGE}_FIND_QUIETLY)
        message(STATUS "Failed to find ${PACKAGE} - " ${REASON_MSG} ${ARGN})
    elseif(${PACKAGE}_FIND_REQUIRED)
        message(FATAL_ERROR "Failed to find ${PACKAGE} - " ${REASON_MSG} ${ARGN})
    else()
        # Neither QUIETLY nor REQUIRED, use no priority which emits a message
        # but continues configuration and allows generation.
        message("-- Failed to find ${PACKAGE} - " ${REASON_MSG} ${ARGN})
    endif()
    return()
endmacro(PACKAGE_REPORT_NOT_FOUND)

# The nie_install_as_pip3_package function uses pip3 to install the package define by the ./scripts/setup.py file
function(NIE_INSTALL_AS_PIP3_PACKAGE)
    find_program(PIP3 pip3)
    if(NOT EXISTS ${PIP3})
        message(FATAL_ERROR "program 'pip3' not found!")
    endif()

    if(NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/scripts/setup.py)
        message(FATAL_ERROR "Unable to install pip3 package, expected file named '${CMAKE_CURRENT_LIST_DIR}/scripts/setup.py'.")
    endif()

    # Make a target, such that
    #  - it is an actual target, like the other (c++) shared libraries
    #  - other targets can depend on it
    add_custom_target(${PROJECT_NAME} ALL
            COMMAND ${PIP3} install ${CMAKE_CURRENT_LIST_DIR}/scripts)
    list(APPEND NIE_PYTHON_PACKAGES "${CMAKE_CURRENT_LIST_DIR}/scripts")

endfunction()
