/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_INERTIAL_EXPLORER_EXPORT_PROFILE_HPP
#define NIE_FORMATS_INERTIAL_EXPLORER_EXPORT_PROFILE_HPP

#include <nie/core/time.hpp>

#include "mapped_fields.hpp"
#include "standard_columns.hpp"

namespace nie {

namespace io {

namespace inertial_explorer {

namespace detail {

inline std::chrono::weeks stow(std::string const& s) {
    return std::chrono::weeks{static_cast<std::chrono::weeks::rep>(std::stoul(s))};
}

}  // namespace detail

/// All export profiles may have headers and footers, and by default they are trivially parsed
struct ExportProfileDefinition {
    // TODO:[EDD] Could try to parse actual header lines (But I'm unsure how customizable they are)
    //      e.g. Project / Program / Profile / Source / SolFile / ProcessInfo / Datum / Projection / Etc...

    using Header = std::string;
    using Footer = std::string;

    static Header ParseHeader(std::string const& line) { return line; }
    static Footer ParseFooter(std::string const& line) { return line; }
};

/// A standard export profile only uses standard columns, which are uniquely identified by their standard column id
struct StandardExportProfile : public ExportProfileDefinition {
    using ColumnId = StandardColumn const;
    using ColumnMapping = std::unordered_map<ColumnId, std::size_t>;

    static char const* ColumnName(ColumnId const& column_id) { return kStandardColumnInfo.at(column_id).name; }
};

/// A custom export profile uses custom columns, which are uniquely identified by their name
struct CustomExportProfile : public ExportProfileDefinition {
    using ColumnId = char const* const;
    using ColumnMapping = std::unordered_map<ColumnId, std::size_t>;

    static char const* ColumnName(ColumnId const& column_id) { return column_id; }
};

/// Profile for backwards compatibility. Avoid using it since it is incomplete.
struct PosTClassic : public StandardExportProfile {
    // TODO: [EDD] std::array deduction guides from C++17 may be used to remove explicit type and size declaration
    //             See: https://en.cppreference.com/w/cpp/container/array/deduction_guides
    // clang-format off
    constexpr static std::array<ColumnId, 9> expected_column_ids{
        StandardColumn::SequenceNumber,
        StandardColumn::GPSTime,
        StandardColumn::Northing,
        StandardColumn::Easting,
        StandardColumn::HeightEllipsoid,
        StandardColumn::Roll,
        StandardColumn::Pitch,
        StandardColumn::Heading,
        StandardColumn::HorizontalSpeed
    };
    // clang-format on

    struct Row {
        std::size_t seq_num;
        /// Classic profiles use time_in_day.
        double time_in_day;
        double northing;
        double easting;
        double height;
        double roll;
        double pitch;
        double heading;
        float horizontal_speed;
    };

    static Row ParseRow(MappedFields<ColumnId> const& fields) {
        // clang-format off
        return {
            std::stoull  (fields[StandardColumn::SequenceNumber]),
            std::stod    (fields[StandardColumn::GPSTime]),
            std::stod    (fields[StandardColumn::Northing]),
            std::stod    (fields[StandardColumn::Easting]),
            std::stod    (fields[StandardColumn::HeightEllipsoid]),
            std::stod    (fields[StandardColumn::Roll]),
            std::stod    (fields[StandardColumn::Pitch]),
            std::stod    (fields[StandardColumn::Heading]),
            std::stof    (fields[StandardColumn::HorizontalSpeed])
        };
        // clang-format on
    }
};

// Currently it includes the Map/Meridian Convergence. This is not always needed, but when we use this as the new
// default, we don't care.
struct AiimFile : public StandardExportProfile {
    // TODO: [EDD] std::array deduction guides from C++17 may be used to remove explicit type and size declaration
    //             See: https://en.cppreference.com/w/cpp/container/array/deduction_guides
    // clang-format off
    constexpr static std::array<ColumnId, 23> expected_column_ids{
        StandardColumn::SequenceNumber,
        StandardColumn::Week,
        StandardColumn::GPSTime,
        StandardColumn::Northing,
        StandardColumn::Easting,
        StandardColumn::HeightEllipsoid,
        StandardColumn::Roll,
        StandardColumn::Pitch,
        StandardColumn::Heading,
        StandardColumn::Convg,
        StandardColumn::CxEE,
        StandardColumn::CxEH,
        StandardColumn::CxEN,
        StandardColumn::CxHH,
        StandardColumn::CxNH,
        StandardColumn::CxNN,
        StandardColumn::CxVHH,
        StandardColumn::CxVPP,
        StandardColumn::CxVPH,
        StandardColumn::CxVPR,
        StandardColumn::CxVRR,
        StandardColumn::CxVRH,
        StandardColumn::HorizontalSpeed
    };
    // clang-format on

    struct Row {
        std::size_t seq_num;
        std::chrono::weeks weeks;
        double time_in_week;
        double northing;
        double easting;
        double height;
        double roll;
        double pitch;
        double heading;
        double map_convergence;
        double cov_t_ee;
        double cov_t_eh;
        double cov_t_en;
        double cov_t_hh;
        double cov_t_nh;
        double cov_t_nn;
        double cov_r_hh;
        double cov_r_pp;
        double cov_r_ph;
        double cov_r_pr;
        double cov_r_rr;
        double cov_r_rh;
        float horizontal_speed;
    };

    static Row ParseRow(MappedFields<ColumnId> const& fields) {
        // clang-format off
        return {
            std::stoull(fields[StandardColumn::SequenceNumber]),
            detail::stow (fields[StandardColumn::Week]),
            std::stod    (fields[StandardColumn::GPSTime]),
            std::stod    (fields[StandardColumn::Northing]),
            std::stod    (fields[StandardColumn::Easting]),
            std::stod    (fields[StandardColumn::HeightEllipsoid]),
            std::stod    (fields[StandardColumn::Roll]),
            std::stod    (fields[StandardColumn::Pitch]),
            std::stod    (fields[StandardColumn::Heading]),
            std::stod    (fields[StandardColumn::Convg]),
            std::stod    (fields[StandardColumn::CxEE]),
            std::stod    (fields[StandardColumn::CxEH]),
            std::stod    (fields[StandardColumn::CxEN]),
            std::stod    (fields[StandardColumn::CxHH]),
            std::stod    (fields[StandardColumn::CxNH]),
            std::stod    (fields[StandardColumn::CxNN]),
            std::stod    (fields[StandardColumn::CxVHH]),
            std::stod    (fields[StandardColumn::CxVPP]),
            std::stod    (fields[StandardColumn::CxVPH]),
            std::stod    (fields[StandardColumn::CxVPR]),
            std::stod    (fields[StandardColumn::CxVRR]),
            std::stod    (fields[StandardColumn::CxVRH]),
            std::stof    (fields[StandardColumn::HorizontalSpeed])
        };
        // clang-format on
    }
};

}  // namespace inertial_explorer

}  // namespace io

}  // namespace nie

#endif  // NIE_FORMATS_INERTIAL_EXPLORER_EXPORT_PROFILE_HPP
