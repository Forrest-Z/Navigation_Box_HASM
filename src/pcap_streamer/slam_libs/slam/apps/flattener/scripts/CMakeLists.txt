# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
project(flattener_scripts)

file(GLOB SCRIPTS "${CMAKE_CURRENT_LIST_DIR}/*.py" "${CMAKE_CURRENT_LIST_DIR}/*.sh")

foreach(SCRIPT ${SCRIPTS})
    get_filename_component(SCRIPT "${SCRIPT}" NAME)
    set(COMMANDS
        "${COMMANDS}"
        COMMAND
        ${CMAKE_COMMAND}
        -E
        copy
        "${CMAKE_CURRENT_LIST_DIR}/${SCRIPT}"
        "${CMAKE_BINARY_DIR}/bin/${SCRIPT}")
endforeach()

add_custom_target(${PROJECT_NAME} ALL ${COMMANDS})
