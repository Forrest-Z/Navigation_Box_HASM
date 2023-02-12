/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <string>
#include <vector>

namespace nie {

// TODO: Refactor operator()() to take random_access_iterator for begin and end instead of vector.

/// Callback functor for handling vector of string tokens.
struct CsvTokenHandler {
    /// Handle vector of tokens.
    /// \param tokens   vector of tokens belonging to a single row in csv file.
    virtual void operator()(std::vector<std::string> const& tokens) = 0;
};

}  // namespace nie
