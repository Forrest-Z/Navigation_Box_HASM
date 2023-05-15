# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
#Discover Python Tests
file(GLOB_RECURSE TESTS_PY "${CMAKE_SOURCE_DIR}/*_test.py")
if(TESTS_PY)
    # find_package(PythonInterp 3) is broken
    # see https://stackoverflow.com/a/16045924/5843895
    # it is deprecated from CMake version 3.12 in favor of FindPython3, FindPython2 and FindPython
    # https://cmake.org/cmake/help/latest/module/FindPythonInterp.html

    set(PYTHON_EXECUTABLE python3)

    # Search only in top level
    file(GLOB PYTHON_REQUIREMENTS "${CMAKE_SOURCE_DIR}/requirements.txt")
    if(PYTHON_REQUIREMENTS)
        message("Found requirements.txt. Creating virtualenv.")

        # Use python3 from a virtualenv instead
        set(PYTHON_EXECUTABLE ${CMAKE_CURRENT_BINARY_DIR}/venv/bin/python3)
        add_custom_target(
            Tests ALL
            DEPENDS venv.stamp
            SOURCES ${PYTHON_REQUIREMENTS})

        find_program(VIRTUALENV virtualenv)
        if(NOT VIRTUALENV)
            message(FATAL_ERROR "Could not find `virtualenv` in PATH")
        endif()
        set(VIRTUALENV ${VIRTUALENV} -p python3)

        # Generate the virtualenv and ensure it's up to date.
        add_custom_command(OUTPUT venv COMMAND ${VIRTUALENV} venv)
        add_custom_command(
            OUTPUT venv.stamp
            DEPENDS venv ${PYTHON_REQUIREMENTS}
            COMMAND ./venv/bin/pip install -r ${PYTHON_REQUIREMENTS} --upgrade)
    endif()

    # Build command line to run py.test.
    set(TEST_COMMAND ${PYTHON_EXECUTABLE} -m coverage run --branch --omit */site-packages/*,*/dist-packages/* -a -m pytest -ra)
    foreach(TEST_FILE ${TESTS_PY})
        get_filename_component(PARENT_PATH ${TEST_FILE} DIRECTORY)
        get_filename_component(PARENT_NAME ${PARENT_PATH} NAME)

        if(PARENT_NAME STREQUAL "test")
            get_filename_component(TEST_File_NAME ${TEST_FILE} NAME_WE)
            get_filename_component(PROJECT_PATH ${PARENT_PATH} DIRECTORY)
            get_filename_component(PROJECT_NAME ${PROJECT_PATH} NAME)
            add_test(
                NAME "Python.${PROJECT_NAME}.${TEST_File_NAME}"
                COMMAND ${TEST_COMMAND} ${TEST_FILE}
                WORKING_DIRECTORY ${CMAKE_BINARY_DIR})
        endif()
    endforeach()
endif()
