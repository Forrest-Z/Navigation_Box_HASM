# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
project(recorder)

find_package(OpenCV 4.5.0 REQUIRED)
find_package(Boost 1.58.0 REQUIRED COMPONENTS date_time)

set(LIBS nie_hardware ${Boost_LIBRARIES})

file(GLOB_RECURSE SOURCES_CPP "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp")

cxx_executable(${PROJECT_NAME} "${SOURCES_CPP}")
target_link_libraries(${PROJECT_NAME} ${LIBS})
