# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
project(nie_formats)

file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp" "${CMAKE_CURRENT_LIST_DIR}/src/*.inl")

add_library(${PROJECT_NAME} SHARED)

# Ladybug
if(NOT IN_JENKINS)
    set(WITH_LADYBUG
        "FALSE"
        CACHE BOOL "Build with support for Ladybug stream files")
else()
    message("Skipping projects that can't build in Jenkins")
endif()

if(${WITH_LADYBUG})
    find_package(Ladybug REQUIRED)
    target_link_libraries(${PROJECT_NAME} PUBLIC ${Ladybug_LIBRARIES})
    target_include_directories(${PROJECT_NAME} PUBLIC ${Ladybug_INCLUDE_DIRS})
    add_definitions(-DUSE_LADYBUG)
else()
    # Remove globbed item
    list(REMOVE_ITEM SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/nie/formats/ladybug.cpp")
endif()

target_sources(${PROJECT_NAME} PRIVATE ${SOURCES})
target_link_libraries(${PROJECT_NAME} PUBLIC nie_core)
target_include_directories(${PROJECT_NAME} PUBLIC "${CMAKE_CURRENT_LIST_DIR}/src")
cxx_default_target_properties(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

if(BUILD_TESTS)
    nie_build_tests(nie_formats_tests)
    target_link_libraries(nie_formats_tests nie_cv)
endif(BUILD_TESTS)
