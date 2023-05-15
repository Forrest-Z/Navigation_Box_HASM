/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <string>

namespace nie {

/// Callback functor for handling individual rows from a csv file.
struct CsvRowHandler {
    /// Handle single csv row.
    virtual void operator()(std::string const& line) = 0;
};

}  // namespace nie
