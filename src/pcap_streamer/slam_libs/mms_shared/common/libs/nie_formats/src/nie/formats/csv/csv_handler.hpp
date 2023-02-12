/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <string>
#include <vector>

namespace nie {

/// Callback functor for handling raw strings read from csv file.
/// Typically this functor passes each row to an instance of CsvRowHandler.
struct CsvHandler {
    /// Handle vector of csv rows.
    virtual void operator()(
            std::vector<std::string>::const_iterator begin, std::vector<std::string>::const_iterator end) = 0;

    /// Handle header row.
    virtual void SetHeader(std::string const& header_line) = 0;
};

}  // namespace nie
