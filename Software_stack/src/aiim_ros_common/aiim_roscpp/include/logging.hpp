/**
 * Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission.
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

// Small helper log functions/macros that mimic glog behavior.

// TODO(jbr): When we move this to some common lib, we should check for alternatives to throw. And check thread safety,
// etc. std::exit(EXIT_FAILURE); shouldn't be called for example, because local objects will not be destroyed which may
// cause all sorts of headaches (like not flushing or writing to files, etc, if that was expected for like a LOG).
#define RLOG_FATAL(logger, stream_arg) \
    do {                                   \
        RCLCPP_FATAL_STREAM(logger, stream_arg); \
        std::stringstream ss; \
        ss << stream_arg; \
        rclcpp::shutdown(nullptr, ss.str());     \
        throw std::runtime_error(ss.str()); \
    } while (0)


#define RCHECK(logger, stream_arg, check) \
    do {\
        if (!(check)) {\
            RLOG_FATAL(logger, stream_arg);\
        }\
    } while (0)
