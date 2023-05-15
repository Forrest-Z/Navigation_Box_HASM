# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
project(nie_core)

find_package(Boost REQUIRED COMPONENTS filesystem iostreams)
find_package(Ceres REQUIRED)
find_package(Glog REQUIRED)
find_package(Gflags REQUIRED)
find_package(Date REQUIRED)
find_package(OpenCV 4.5.0 REQUIRED)
find_package(Eigen3 REQUIRED)

file(GLOB_RECURSE SOURCES "${CMAKE_CURRENT_LIST_DIR}/src/*.cpp" "${CMAKE_CURRENT_LIST_DIR}/src/*.inl")

# The target names GTest::GTest and GTest::Main are hidden through the builder script. This is to avoid importing
# targets in which we are not interested. However, as a result the target names of the ones we are interested in are
# gone and cannot be refered to as one would normally do (GTest::GTest, GTest::Main). The corresponding binaries are
# available through Glog and Gflags or can be refered to directly through the names of the binaries which are generated
# in the same directory as all other libs: gtest gtest_main
# Linking against nie_core will also make gtest available.
set(LIBS
    ${Date_LIBRARIES}
    ${Boost_LIBRARIES}
    Eigen3::Eigen
    ${CERES_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${GFLAGS_LIBRARIES}
    ${OpenCV_LIBS})

cxx_library_shared(${PROJECT_NAME} ${SOURCES})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${Date_DEFINITIONS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/src)
target_link_libraries(${PROJECT_NAME} ${LIBS})

if(BUILD_TESTS)
    nie_build_tests()
endif(BUILD_TESTS)
