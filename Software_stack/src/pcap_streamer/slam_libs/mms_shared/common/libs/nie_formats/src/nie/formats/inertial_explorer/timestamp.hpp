/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_INERTIAL_EXPLORER_TIMESTAMP_HPP
#define NIE_FORMATS_INERTIAL_EXPLORER_TIMESTAMP_HPP

#include <nie/core/time.hpp>

#include "export_profile.hpp"
#include "project_name.hpp"

namespace nie {
namespace io {
namespace inertial_explorer {

template <typename RowProfile>
nie::Timestamp_ns TimestampFromPosT(RowProfile const& row) {
    static_assert(
        !std::is_same_v<RowProfile, PosTClassic>, "PROGRAMMING_MISTAKE_FOR_POST_CLASSIC_USE_THE_TWO_ARGUMENT_FUNCTION");
    GPSWeekTime<std::chrono::nanoseconds> gps_week_time{
        row.weeks, nie::RepresentDoubleAsDuration<std::chrono::nanoseconds>(row.time_in_week)};
    return ToGPSTime(gps_week_time);
}

/// @brief Gives a timestamp based on a UTC date and gps time in the day.
/// @details Avoid using this function as the timestamp may not be correctly determined.
/// @param utc_zoned_date The date in UTC. Translates into a GPS date that may differ
nie::Timestamp_ns TimestampFromPosT(PosTClassic::Row const& row, std::chrono::year_month_day const& utc_zoned_date) {
    return nie::ToGPSTime(
        utc_zoned_date,
        nie::io::inertial_explorer::kTimezoneOffset,
        false,
        nie::RepresentDoubleAsDuration<std::chrono::nanoseconds>(row.time_in_day));
}

}  // namespace inertial_explorer
}  // namespace io
}  // namespace nie

#endif  // NIE_FORMATS_INERTIAL_EXPLORER_TIMESTAMP_HPP
