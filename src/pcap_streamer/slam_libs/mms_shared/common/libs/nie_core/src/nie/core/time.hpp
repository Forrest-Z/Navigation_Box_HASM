/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <chrono>
#include <type_traits>

// NOTE: Currently, this code depends on the 'date' library. See: https://github.com/HowardHinnant/date/wiki
//       In March 2018 this library is accepted by the ISO C++ standardisation committee to be included in the draft
//       C++20 standard. This temporary dependency can be removed when we move to the finalized C++20 standard.
//       See also: https://en.cppreference.com/w/cpp/chrono
#include <date/tz.h>

// NOTE: This is not fully standard compliant, but still proposed by the library author
namespace std {
namespace chrono {
using namespace date;
}

namespace literals {
namespace chrono_literals {
using namespace date::literals;
}
}  // namespace literals
}  // namespace std

#include <nie/core/glog.hpp>
#include <nie/core/string.hpp>

namespace nie {

namespace detail {

// Define the epoch of GPS and SYS clocks
// TODO: [EDD] These are also part of the date library, but it seems they are restricted to their own implementation
constexpr auto gps_epoch = std::chrono::year(1980) / 01 / 06;
constexpr auto sys_epoch = std::chrono::year(1970) / 01 / 01;

// Determine the epoch difference between GPS clock and SYS clock
constexpr auto epoch_difference_gps_to_sys = std::chrono::sys_days(gps_epoch) - std::chrono::sys_days(sys_epoch);

}  // namespace detail

/// @brief GPS Time, alternatively represented as composite GPS week, duration in week
template <typename Duration>
struct GPSWeekTime {
    std::chrono::weeks week;
    Duration time_in_week;

    [[nodiscard]] auto tie() const { return std::tie(week, time_in_week); }

    [[nodiscard]] bool operator==(GPSWeekTime const& other) const { return tie() == other.tie(); }
    [[nodiscard]] bool operator<(GPSWeekTime const& other) const { return tie() < other.tie(); }
};

/// @brief GPS Time, alternatively represented as composite GPS day, duration in day
template <typename Duration>
struct GPSDayTime {
    std::chrono::days day;
    Duration time_in_day;

    [[nodiscard]] auto tie() const { return std::tie(day, time_in_day); }

    [[nodiscard]] bool operator==(GPSDayTime const& other) const { return tie() == other.tie(); }
    [[nodiscard]] bool operator<(GPSDayTime const& other) const { return tie() < other.tie(); }
};

/// @brief UTC Zoned Time, alternatively represented as UTC date, time in date, timezone offset, leap second flag
template <typename Duration>
struct UTCDateTimeTz {
    // Default timezone is UTC +00:00, timezone offset in seconds is not supported
    UTCDateTimeTz(
            std::chrono::year_month_day const& date,
            Duration const& time_in_date,
            std::chrono::minutes tz_offset = std::chrono::hours(0),
            bool leap_second = false)
        : date_(date), time_in_date_(time_in_date), tz_offset_(tz_offset), leap_second_(leap_second) {
        // Time in day must lie in the valid 24 hour range
        CHECK(time_in_date_ >= std::chrono::hours(0) && time_in_date_ < std::chrono::hours(24))
                << "Time in day does not lie in valid range";

        // Currently, minimum UTC timezone offset is -12:00, maximum UTC timezone offset is +14:00, see:
        //   https://en.wikipedia.org/wiki/List_of_UTC_time_offsets
        CHECK(tz_offset_ >= -std::chrono::hours(12) && tz_offset_ <= +std::chrono::hours(14))
                << "Unexpected timezone offset";

        // Verify that a zoned leap second occurs 1 second before a whole minute, see:
        CHECK(!leap_second_ || (time_in_date_ % std::chrono::minutes(1) == std::chrono::seconds{59}))
                << "Leap second specified on a zoned UTC date / time, but it does not fall on a whole minute";

        // TODO: [EDD] Could verify that a zoned leap second occurs only on an expected date, see:
        //   https://en.wikipedia.org/wiki/Leap_second
    }

    [[nodiscard]] auto date() const { return date_; }
    [[nodiscard]] auto time_in_date() const { return time_in_date_; }
    [[nodiscard]] auto tz_offset() const { return tz_offset_; }
    [[nodiscard]] auto leap_second() const { return leap_second_; }

private:
    // Only allow durations with a period of at most 1 second
    static_assert(Duration::period::num / Duration::period::den <= 1.0);

    std::chrono::year_month_day date_;
    Duration time_in_date_;
    std::chrono::minutes tz_offset_;
    bool leap_second_;
};

/**
 * @brief Convert given duration to a floating point duration in seconds
 *
 * @warning It is strongly advised NOT to use a double as the representation of a duration, because a double may not be
 *          able to represent large durations with sufficient accuracy, especially if you need to represent small time
 *          differences (e.g. nanoseconds) over a large period (e.g. many years). You should NOT use this unless you
 *          are forced to by external reasons.
 */
template <typename Duration>
[[nodiscard]] double RepresentDurationAsDouble(Duration const& duration) {
    using Period = typename Duration::period::type;

    return static_cast<double>(duration.count()) * Period::num / Period::den;
}

/**
 * @brief Convert given time point to a floating point duration in seconds from the epoch of the clock
 *
 * @warning It is strongly advised NOT to use a double as the representation of a time point, because it is unclear
 *          from the type which clock it is derived from, which makes mixing clocks error prone. Additionally, a double
 *          may not be able to represent all required timestamps with sufficient accuracy. You should NOT use this
 *          unless you are forced to by external reasons.
 */
template <typename Clock, typename Duration>
[[nodiscard]] double RepresentTimePointAsDouble(std::chrono::time_point<Clock, Duration> const& time_point) {
    return RepresentDurationAsDouble(time_point.time_since_epoch());
}

/// @brief Convert given floating point duration in seconds to a  duration
template <typename Duration>
[[nodiscard]] Duration RepresentDoubleAsDuration(double v) {
    using Period = typename Duration::period::type;

    return Duration{static_cast<typename Duration::rep>(v * (Period::den / Period::num))};
}

/// @brief Convert a GPS Week / Time to a GPS time
template <typename Duration>
[[nodiscard]] std::chrono::gps_time<Duration> ToGPSTime(GPSWeekTime<Duration> const& week_time) {
    return std::chrono::gps_time<Duration>{week_time.week + week_time.time_in_week};
}

/// @brief Convert a GPS Day / Time to a GPS time
template <typename Duration>
[[nodiscard]] std::chrono::gps_time<Duration> ToGPSTime(GPSDayTime<Duration> const& day_time) {
    return std::chrono::gps_time<Duration>{day_time.day + day_time.time_in_day};
}

/// @brief Convert a UTC Date / Time / Timezone / Leap second flag to a GPS time
template <typename Duration>
[[nodiscard]] std::chrono::gps_time<Duration> ToGPSTime(UTCDateTimeTz<Duration> const& zoned_date_time) {
    // Compute SYS time
    auto const sys_time = std::chrono::sys_days{zoned_date_time.date()} + zoned_date_time.time_in_date() +
                          zoned_date_time.tz_offset();

    // Convert to GPS time
    return std::chrono::to_gps_time(sys_time) + std::chrono::seconds(zoned_date_time.leap_second() ? 1 : 0);
}

/// @brief Special case, where a date is given in UTC with a given timezone offset, but the time is
///        given as the time in the related GPS day.
///
/// You would be mad to specify it as such, but this is exactly what was done with (old style) PosT files
///
/// @note We assume that the UTC clock and GPS clock are synced (In reality they are not!)
template <typename Duration>
[[nodiscard]] std::chrono::gps_time<Duration> ToGPSTime(
        std::chrono::year_month_day const& utc_zoned_date,
        std::chrono::minutes const& utc_timezone_offset,
        bool utc_leap_second,
        Duration const& gps_time_in_day) {
    // Obtain timestamp of start of day of non zoned UTC date
    auto const utc_date = std::chrono::to_utc_time(std::chrono::sys_days{utc_zoned_date});

    // Determine the corrected time-in-day for UTC
    auto const utc_tz_corrected_time_in_day =
            gps_time_in_day - utc_timezone_offset - std::chrono::seconds(utc_leap_second ? 1 : 0);

    // Determine the time difference [in days] between GPS time and UTC time (including timezone)
    auto const utc_day_offset = std::chrono::floor<std::chrono::days>(utc_tz_corrected_time_in_day);

    // Determine days since GPS epoch
    auto const gps_day = std::chrono::floor<std::chrono::days>(to_gps_time(utc_date - utc_day_offset));

    return gps_day + gps_time_in_day;
}

template <typename ExtractedDuration, typename OriginalDuration>
std::tuple<ExtractedDuration, OriginalDuration> SplitDuration(OriginalDuration const& original_duration) {
    // Assert that the extracted duration is a proper multiple of the original duration
    static_assert(std::ratio_divide<typename ExtractedDuration::period, typename OriginalDuration::period>::den == 1);

    // Split elapsed time in GPS week and time in GPS week
    auto const extracted_duration = std::chrono::floor<ExtractedDuration>(original_duration);
    auto const remaining_duration = original_duration - extracted_duration;

    return {extracted_duration, remaining_duration};
}

/// @brief Convert a GPS time to GPS Week / Time
template <typename Duration>
[[nodiscard]] GPSWeekTime<Duration> ToGPSWeekTime(std::chrono::gps_time<Duration> const& gps_time) {
    // Obtain elapsed time since epoch
    auto const elapsed = gps_time.time_since_epoch();

    // Split elapsed time in GPS week and time in GPS week
    auto const [week, time_in_week] = SplitDuration<std::chrono::weeks>(elapsed);

    return {week, time_in_week};
}

/// @brief Convert a GPS timestamp to GPS Day / Time
template <typename Duration>
[[nodiscard]] GPSDayTime<Duration> ToGPSDayTime(std::chrono::gps_time<Duration> const& gps_time) {
    // Obtain elapsed time since epoch
    auto const elapsed = gps_time.time_since_epoch();

    // Split elapsed time in GPS week and time in GPS week
    auto const [day, time_in_day] = SplitDuration<std::chrono::days>(elapsed);

    return {day, time_in_day};
}

/**
 * @brief Define a timestamp as a time point on the GPS clock
 *
 * The GPS clock has been chosen as our standard because:
 *
 *   1) TAI / GPS clocks do not take leap seconds into account (Unlike UTC and UNIX time), so:
 *      a) All timestamps are unambiguous
 *      b) Time computations within the clock domain are simple
 *   2) The TAI / GPS clocks do not take timezones into account
 *   3) GPS time can easily be obtained with high accuracy from GNSS hardware
 *   4) Our recording platforms use GPS time as reference
 *
 * Note that leap seconds are introduced by committee to synchronize clocks with the rotation speed of the earth, and
 * are hence not known in advance. Time zones are defined by international agreements and political considerations.
 * Both issues may lead to complicated issues.
 *
 * See also: https://en.wikipedia.org/wiki/Global_Positioning_System#Timekeeping
 *           https://en.wikipedia.org/wiki/Unix_time#Leap_seconds
 *           https://en.wikipedia.org/wiki/International_Atomic_Time
 */
using Timestamp_us = std::chrono::gps_time<std::chrono::microseconds>;
using Timestamp_ns = std::chrono::gps_time<std::chrono::nanoseconds>;

/**
 * @brief Parse a duration that is serialized as two durations concatenated by a separator token
 *
 * For instance, "1234.567890" could be parsed as a duration of 1234 seconds and 567890 microseconds
 */
template <typename Duration0, typename Duration1>
[[nodiscard]] typename std::common_type<Duration0, Duration1>::type ParseFractionalDuration(
        std::string const& text, char const separator = '.') {
    auto const parts = nie::Split<std::string>(text, separator);

    return Duration0{std::stoull(parts[0])} + Duration1{std::stoull(parts[1])};
}

}  // namespace nie
