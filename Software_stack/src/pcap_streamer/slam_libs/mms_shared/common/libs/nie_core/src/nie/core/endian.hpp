/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

namespace nie {

// Apparently, there is currently no portable way to detect the endianness of a target system at compile time
// See: https://stackoverflow.com/questions/4239993/determining-endianness-at-compile-time
// TODO: [EDD] When we move to C++20 we can use std::endian, see https://en.cppreference.com/w/cpp/types/endian
enum class Endian {
#if defined(_WIN32)
    kLittle = 0,
    kBig = 1,
    kNative = kLittle
#elif defined(__BYTE_ORDER__)
    kLittle = __ORDER_LITTLE_ENDIAN__,
    kBig = __ORDER_BIG_ENDIAN__,
    kNative = __BYTE_ORDER__
#else
    // Prefer static_assert over #error to avoid cpp_check choking
    static_assert(true, "Endianness of the target system is unknown")
#endif
};

}  // namespace nie
