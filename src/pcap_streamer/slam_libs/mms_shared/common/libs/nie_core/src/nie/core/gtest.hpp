/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <gtest/gtest.h>

// The epsilon used is proportional to the largest value. The typical ASSERT_NEAR use
// case has an epsilon defined with respect to the value 1.0 (like the one from numeric
// limits). It 1.0 + epsilon represents the next value a float can take. Since math isn't
// perfect and we have some stochastic processes the epsilon used is typically bigger than
// the machine one.
#ifndef ASSERT_NEAR_RELATIVE
#define ASSERT_NEAR_RELATIVE(val1, val2, abs_error) \
    ASSERT_NEAR(val1, val2, (abs_error * std::max(std::abs(val1), std::abs(val2))))
#endif
