/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <tuple>

namespace nie {

/// Callback functor for handling tuple of csv values.
template <typename... Ts>
struct CsvTupleHandler {
    virtual void operator()(std::tuple<Ts...>&& csv_tup) = 0;
};

}  // namespace nie
