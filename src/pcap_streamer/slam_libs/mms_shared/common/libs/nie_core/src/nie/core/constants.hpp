/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

namespace nie {

// TODO: [EDD] When moving to C++20 this may be replaced by std::numbers::pi
//            As specified in: http://eel.is/c++draft/numbers

// Now grabbed from #include <boost/math/constants/constants.hpp>
template <typename T = double>
static const T kPi = T(3.141592653589793238462643383279502884e+00);

template <typename T = double>
static const T kDeg2Rad = kPi<T> / T(180);

template <typename T = double>
static const T kRad2Deg = T(180) / kPi<T>;

}  // namespace nie
