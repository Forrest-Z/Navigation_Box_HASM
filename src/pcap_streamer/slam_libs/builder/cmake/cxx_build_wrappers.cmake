# Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
# Information classification: Confidential
# This content is protected by international copyright laws.
# Reproduction and distribution is prohibited without written permission.
# Converts the includes of the provided libraries to system includes. These don't give warnings anymore.
function(TARGET_LINK_LIBRARIES_SYSTEM TARGET)
    set(LIBS_AS_SYSTEM_LIBS ${ARGN})
    foreach(LIB_AS_SYSTEM_LIB ${LIBS_AS_SYSTEM_LIBS})
        target_link_libraries(${TARGET} ${LIB_AS_SYSTEM_LIB})

        # If the library is not known as a target, we cannot query it for includes to convert to SYSTEM.
        if(NOT TARGET ${LIB_AS_SYSTEM_LIB})
            continue()
        endif()

        get_target_property(AS_SYSTEM_INCLUDES_DIRECTORIES ${LIB_AS_SYSTEM_LIB} INTERFACE_INCLUDE_DIRECTORIES)

        if(AS_SYSTEM_INCLUDES_DIRECTORIES)
            target_include_directories(${TARGET} SYSTEM INTERFACE ${AS_SYSTEM_INCLUDES_DIRECTORIES})
        endif()

        get_target_property(AS_SYSTEM_INCLUDES_DIRECTORIES ${LIB_AS_SYSTEM_LIB} INCLUDE_DIRECTORIES)

        if(AS_SYSTEM_INCLUDES_DIRECTORIES)
            target_include_directories(${TARGET} SYSTEM PUBLIC ${AS_SYSTEM_INCLUDES_DIRECTORIES})
        endif()
    endforeach()
endfunction()

function(cxx_default_target_properties name)
    set_property(TARGET ${name} APPEND_STRING PROPERTY COMPILE_FLAGS " ${NIE_WARNING_FLAGS}")
    set_target_properties(${name} PROPERTIES CXX_STANDARD 17 CXX_EXTENSIONS OFF)
endfunction()

# Wrapper functions inspired from GoogleTest cmake strategy https://github.com/g
# oogle/googletest/blob/release-1.8.0/googletest/cmake/internal_utils.cmake#L146

# Defines a wrapper function to create library targets
function(cxx_library_with_type name type cxx_flags libs)
    # type can be either STATIC or SHARED to denote a static or shared library.
    # ARGN refers to additional arguments after 'cxx_flags'.

    if(NOT (${type} STREQUAL "STATIC" OR ${type} STREQUAL "SHARED"))
        message(FATAL_ERROR "Library type can be either STATIC or SHARED.")
    endif()

    add_library(${name} ${type} ${ARGN})

    set_target_properties(${name} PROPERTIES COMPILE_FLAGS "${cxx_flags}")

    cxx_default_target_properties(${name})

    if(CMAKE_USE_PTHREADS_INIT)
        target_link_libraries(${name} ${CMAKE_THREAD_LIBS_INIT})
    endif()

    foreach(lib "${libs}")
        target_link_libraries(${name} ${lib})
    endforeach()

    install(
        TARGETS ${name}
        ARCHIVE DESTINATION libs
        LIBRARY DESTINATION lib)

endfunction()

function(cxx_library_static name)
    cxx_library_with_type(${name} STATIC "" "" ${ARGN})
endfunction()

function(cxx_library_static_with_libs name libs)
    cxx_library_with_type(${name} STATIC "" "${libs}" ${ARGN})
endfunction()

function(cxx_library_static_with_flags name cxx_flags)
    cxx_library_with_type(${name} STATIC "${cxx_flags}" "" ${ARGN})
endfunction()

function(cxx_library_static_with_flags_and_libs name cxx_flags libs)
    cxx_library_with_type(${name} STATIC "${cxx_flags}" "${libs}" ${ARGN})
endfunction()

function(cxx_library_shared name)
    cxx_library_with_type(${name} SHARED "" "" ${ARGN})
endfunction()

function(cxx_library_shared_with_libs name libs)
    cxx_library_with_type(${name} SHARED "" "${libs}" ${ARGN})
endfunction()

function(cxx_library_shared_with_flags name cxx_flags)
    cxx_library_with_type(${name} SHARED "${cxx_flags}" "" ${ARGN})
endfunction()

function(cxx_library_shared_with_flags_and_libs name cxx_flags libs)
    cxx_library_with_type(${name} SHARED "${cxx_flags}" "${libs}" ${ARGN})
endfunction()

# cxx_executable_with_flags_and_libs(name cxx_flags libs srcs...)
#
# creates a named C++ executable that depends on the given libraries and is
# built from the given source files with the given compiler flags.
function(cxx_executable_with_flags_and_libs name cxx_flags libs srcs)
    add_executable(${name} "${srcs}")

    set_target_properties(${name} PROPERTIES COMPILE_FLAGS "${cxx_flags}")

    cxx_default_target_properties(${name})

    # To support mixing linking in static and dynamic libraries, link each library
    # in with an extra call to target_link_libraries.
    foreach(lib "${libs}")
        target_link_libraries(${name} ${lib})
    endforeach()

    install(TARGETS ${name} RUNTIME DESTINATION bin)

endfunction()

function(cxx_executable_with_libs name libs srcs)
    cxx_executable_with_flags_and_libs(${name} "" "${libs}" "${srcs}")
endfunction()

function(cxx_executable_with_flags name cxx_flags srcs)
    cxx_executable_with_flags_and_libs(${name} "${cxx_flags}" "" "${srcs}")
endfunction()

function(cxx_executable name srcs)
    cxx_executable_with_flags_and_libs(${name} "" "" "${srcs}")
endfunction()
