# Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.

# This function installs our default package structure
function(aiim_target_ament_install target)
    install(TARGETS ${target}
            DESTINATION lib/${target})
    install(DIRECTORY launch/
            DESTINATION share/${target}/launch)
    install(DIRECTORY params/
            DESTINATION share/${target}/params)
endfunction()

# This function sets our default cmake properties
function(aiim_target_set_default_properties target)
    set_target_properties(${target}
            PROPERTIES
            CXX_STANDARD 17
            CXX_STANDARD_REQUIRED ON
            CXX_EXTENSIONS OFF
            COMPILE_FLAGS "-Wall"
            )
endfunction()

# This is similar to ament_auto_dependencies
# But gives us more control for including external libraries such as PCL
function(aiim_auto_ros_dependencies target)
        foreach(package ${ARGN})
                find_package(${package} REQUIRED)
        endforeach()
        ament_target_dependencies(${target} ${ARGN})
endfunction()