/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <glog/logging.h>

namespace nie {

inline void LogException(const std::exception& e, int level = 0) {
    try {
        std::rethrow_if_nested(e);
    } catch (const std::exception& e) {
        LogException(e, level + 1);
    } catch (...) {
    }

    LOG(ERROR) << std::string(level, '-') << e.what();
}

inline void LogExceptionAndFatal(const std::exception& e) {
    try {
        std::rethrow_if_nested(e);
    } catch (const std::exception& e) {
        LogException(e, 1);
    } catch (...) {
    }

    LOG(FATAL) << e.what();
}

}  // namespace nie
