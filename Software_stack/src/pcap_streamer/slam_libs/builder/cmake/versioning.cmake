# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

# Create version files (cpp and hpp) and set it in ${version_file} calee should
# add it to target_include_directories
function(cxx_add_version name major minor)
    find_package(Git QUIET)

    if(GIT_FOUND)
        # the commit's SHA1, and whether the building workspace was dirty or not
        execute_process(
            COMMAND "${GIT_EXECUTABLE}" describe --match=NeVeRmAtCh --always --abbrev=40 --dirty
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
            OUTPUT_VARIABLE CMAKE_AUTO_GENERATED_GIT_SHA1
            ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

        # the date of the commit
        execute_process(
            COMMAND "${GIT_EXECUTABLE}" log -1 --format=%ad --date=local
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
            OUTPUT_VARIABLE CMAKE_AUTO_GENERATED_GIT_DATE ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)

        # the subject of the commit
        execute_process(
            COMMAND "${GIT_EXECUTABLE}" log -1 --format=%s
            WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
            OUTPUT_VARIABLE CMAKE_AUTO_GENERATED_GIT_COMMIT_SUBJECT ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    else()
        message(WARNING "Unable to find Git package")
        set(CMAKE_AUTO_GENERATED_GIT_SHA1 "<error>")
        set(CMAKE_AUTO_GENERATED_GIT_DATE "<error>")
        set(CMAKE_AUTO_GENERATED_GIT_COMMIT_SUBJECT "<error>")
    endif()

    # set project name, major and minor
    set(CMAKE_AUTO_GENERATED_PROJECT_NAME "${name}")
    set(CMAKE_AUTO_GENERATED_MAJOR "${major}")
    set(CMAKE_AUTO_GENERATED_MINOR "${minor}")

    # generate version file
    configure_file("${NIE_DIR}/builder/cmake/version.hpp.in" "${CMAKE_CURRENT_BINARY_DIR}/version.hpp" @ONLY)
    configure_file("${NIE_DIR}/builder/cmake/version.cpp.in" "${CMAKE_CURRENT_BINARY_DIR}/version.cpp" @ONLY)

    set(version_file
        "${CMAKE_CURRENT_BINARY_DIR}/version.cpp"
        PARENT_SCOPE)
endfunction()
