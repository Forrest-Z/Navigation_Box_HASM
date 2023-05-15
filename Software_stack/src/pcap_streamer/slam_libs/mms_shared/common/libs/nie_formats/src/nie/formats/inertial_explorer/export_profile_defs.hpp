/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_INERTIAL_EXPLORER_EXPORT_PROFILE_DEFS_HPP
#define NIE_FORMATS_INERTIAL_EXPLORER_EXPORT_PROFILE_DEFS_HPP

#include "export_reader.hpp"

namespace nie {

namespace io {

namespace inertial_explorer {

// clang-format off
// TODO: [EDD] When switching to C++17, it is not required anymore to give an explicit definition for constexpr static
//             variables, because they are implicitly inlined, see:
//             https://stackoverflow.com/questions/8016780/undefined-reference-to-static-constexpr-char
constexpr std::array<PosTClassic        ::ColumnId, 9>  PosTClassic     ::expected_column_ids;
constexpr std::array<AiimFile           ::ColumnId, 23> AiimFile        ::expected_column_ids;
// clang-format on

}  // namespace inertial_explorer

}  // namespace io

}  // namespace nie

#endif  // NIE_FORMATS_INERTIAL_EXPLORER_EXPORT_PROFILE_DEFS_HPP
