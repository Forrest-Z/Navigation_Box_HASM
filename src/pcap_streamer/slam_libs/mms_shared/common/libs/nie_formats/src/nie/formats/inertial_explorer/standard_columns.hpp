/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_INERTIAL_EXPLORER_STANDARD_COLUMNS_HPP
#define NIE_FORMATS_INERTIAL_EXPLORER_STANDARD_COLUMNS_HPP

#include <unordered_map>
#include <unordered_set>

namespace nie {

namespace io {

namespace inertial_explorer {

struct ColumnInfo {
    char const* const name;
    char const* const units;
};

// Define standard columns that may be exported by Inertial Explorer
// NOTE: This list is currently NOT complete (There are 100+ possible output variables), for all possible fields see
//       Appendix B of the Inertial Explorer 8.80 User Manual v7, found at:
//          https://docs.novatel.com/Waypoint/Content/PDFs/Waypoint_Software_User_Manual_OM-20000166.pdf
//       or the html version from Inertial Explorer 8.70 v4:
//          https://docs.novatel.com/Waypoint/Content/Appendix/Output_Variables.htm
// clang-format off
enum class StandardColumn {
    // Sequence number that increments by user defined value. Jumps of more than one indicate missing epochs
    SequenceNumber,
    // Week number starting at Jan 6, 1980
    // Note that the PosT file says Jan 4, which is a bug.
    Week,
    // Date of epoch or feature (GMT time zone)
    // Day of Week (0-6)
    Date,
    // Time of epoch or feature - Receiver time frame
    // Can be either of: Seconds of the Week, Seconds of the Day
    GPSTime,
    // North (y) coordinate in pre-defined grid system
    Northing,
    // Standard Deviation of the North coordinate
    NorthingSD,
    // East (x) coordinate in pre-defined grid system
    Easting,
    // Standard Deviation of the East coordinate
    EastingSD,
    // Height above the current ellipsoid
    HeightEllipsoid,
    // Height above the Ellipsoid Standard Deviation
    HeightEllipsoidSD,
    // North/South Geographic coordinate
    Latitude,
    // East/West Geographic coordinate
    Longitude,
    // Earth Centered Earth Fixed X-Axis Standard Deviation
    ECEFXSD,
    // Earth Centered Earth Fixed Y-Axis Standard Deviation
    ECEFYSD,
    // Earth Centered Earth Fixed Z-Axis Standard Deviation
    ECEFZSD,
    HorizontalSpeed,
    // GNSS/INS computed roll value - rotation about body y-axis
    Roll,
    RollSD,
    // GNSS/INS computed pitch value - rotation about body x-axis
    Pitch,
    PitchSD,
    // GNSS/INS computed heading value - rotation about body z-axis
    Heading,
    HeadingSD,
    // Convergence of the Meridians for current location and map projection
    Convg,
    // Computed local level position covariance covariance CX(E,E)
    CxEE,
    // Computed local level position covariance covariance CX(E,H)
    CxEH,
    // Computed local level position covariance covariance CX(E,N)
    CxEN,
    // Computed local level position covariance covariance CX(H,H)
    CxHH,
    // Computed local level position covariance covariance CX(E,N)
    CxNH,
    // Computed local level position covariance covariance CX(N,N)
    CxNN,
    // Computed heading covariance CX(H,H)
    CxVHH,
    // Computed pitch covariance CX(P,P)
    CxVPP,
    // Estimated pitch_heading covariance CX(P,H)
    CxVPH,
    // Computed pitch covariance CX(P,R)
    CxVPR,
    // Computed roll covariance CX(R,R)
    CxVRR,
    // Estimated roll_heading covariance CX(R,H)
    CxVRH,
    // Computed position ECEF covariance CX(1,1)
    ECEFCx11,
    // Computed position ECEF covariance CX(2,1)
    ECEFCx21,
    // Computed position ECEF covariance CX(2,2)
    ECEFCx22,
    // Computed position ECEF covariance CX(3,1)
    ECEFCx31,
    // Computed position ECEF covariance CX(3,2)
    ECEFCx32,
    // Computed position ECEF covariance CX(3,3)
    ECEFCx33,
    ProjectName,
    // Quality factor, 1 (best) to 6 (worst)
    Quality
};

constexpr static auto kUnitNone                     = "";
constexpr static auto kUnitWeeks                    = "weeks";
constexpr static auto kUnitDayOfWeek                = "d";    // Note: Column only says "d", not "(d)"
constexpr static auto kUnitSeconds                  = "sec";  // Inertial Explorer uses non-SI abbreviation
constexpr static auto kUnitMeters                   = "m";
constexpr static auto kUnitDegrees                  = "deg";
constexpr static auto kUnitMetersPerSecond          = "m/s";
constexpr static auto kUnitMetersSquared            = "m^2";
constexpr static auto kUnitDegreesSquared           = "d^2";  // Note: Never output, assumed to be as such based on sec
constexpr static auto kUnitMetersPerSecondSquared   = "m^2/s^2";

// TODO(jbr):
// Columns with different meanings can have the same name and they can be available within the same PosT file.
// Examples are:
//      GPSTime      Seconds of the Week            Time of epoch or feature - Receiver time frame
//      GPSTime      Seconds of the Day             Time of epoch or feature - Receiver time frame
//      CxVHH                                       Computed heading covariance CX(H,H)
//      CxVHH                                       Computed local level velocity covariance CX(H,H)
static std::unordered_map<StandardColumn, ColumnInfo> const kStandardColumnInfo = {
    { StandardColumn::SequenceNumber,    { "SeqNum",       kUnitNone            } },
    { StandardColumn::Week,              { "Week",         kUnitWeeks           } },
    { StandardColumn::Date,              { "D",            kUnitDayOfWeek       } },
    { StandardColumn::GPSTime,           { "GPSTime",      kUnitSeconds         } },
    { StandardColumn::Northing,          { "Northing",     kUnitMeters          } },
    { StandardColumn::NorthingSD,        { "SDNorth",      kUnitMeters          } },
    { StandardColumn::Easting,           { "Easting",      kUnitMeters          } },
    { StandardColumn::EastingSD,         { "SDEast",       kUnitMeters          } },
    { StandardColumn::HeightEllipsoid,   { "H-Ell",        kUnitMeters          } },
    { StandardColumn::HeightEllipsoidSD, { "SDHeight",     kUnitMeters          } },
    { StandardColumn::Latitude,          { "Latitude",     kUnitDegrees         } },
    { StandardColumn::Longitude,         { "Longitude",    kUnitDegrees         } },
    { StandardColumn::ECEFXSD,           { "SDX-ECEF",     kUnitMeters          } },
    { StandardColumn::ECEFYSD,           { "SDY-ECEF",     kUnitMeters          } },
    { StandardColumn::ECEFZSD,           { "SDZ-ECEF",     kUnitMeters          } },
    { StandardColumn::HorizontalSpeed,   { "HzSpeed",      kUnitMetersPerSecond } },
    { StandardColumn::Roll,              { "Roll",         kUnitDegrees         } },
    { StandardColumn::RollSD,            { "RollSD",       kUnitDegrees         } },
    { StandardColumn::Pitch,             { "Pitch",        kUnitDegrees         } },
    { StandardColumn::PitchSD,           { "PitchSD",      kUnitDegrees         } },
    { StandardColumn::Heading,           { "Heading",      kUnitDegrees         } },
    { StandardColumn::HeadingSD,         { "HdngSD",       kUnitDegrees         } },
    { StandardColumn::Convg,             { "Convg",        kUnitDegrees         } },
    { StandardColumn::CxEE,              { "CxEE",         kUnitMetersSquared   } },
    { StandardColumn::CxEH,              { "CxEH",         kUnitMetersSquared   } },
    { StandardColumn::CxEN,              { "CxEN",         kUnitMetersSquared   } },
    { StandardColumn::CxHH,              { "CxHH",         kUnitMetersSquared   } },
    { StandardColumn::CxNH,              { "CxNH",         kUnitMetersSquared   } },
    { StandardColumn::CxNN,              { "CxNN",         kUnitMetersSquared   } },
    // Note that according to the PosT file, the covariances of the heading, pitch and roll should be
    // kUnitMetersPerSecondSquared. It is assumed that this is a bug.
    // It is validated that sqrt(CxVHH) == HdngSD
    { StandardColumn::CxVHH,             { "CxVHH",        kUnitDegreesSquared  } },
    { StandardColumn::CxVPP,             { "CxVPP",        kUnitDegreesSquared  } },
    { StandardColumn::CxVPH,             { "CxVPH",        kUnitDegreesSquared  } },
    { StandardColumn::CxVPR,             { "CxVPR",        kUnitDegreesSquared  } },
    { StandardColumn::CxVRR,             { "CxVRR",        kUnitDegreesSquared  } },
    { StandardColumn::CxVRH,             { "CxVRH",        kUnitDegreesSquared  } },
    { StandardColumn::ECEFCx11,          { "Cx11",         kUnitMetersSquared   } },
    { StandardColumn::ECEFCx21,          { "Cx21",         kUnitMetersSquared   } },
    { StandardColumn::ECEFCx22,          { "Cx22",         kUnitMetersSquared   } },
    { StandardColumn::ECEFCx31,          { "Cx31",         kUnitMetersSquared   } },
    { StandardColumn::ECEFCx32,          { "Cx32",         kUnitMetersSquared   } },
    { StandardColumn::ECEFCx33,          { "Cx33",         kUnitMetersSquared   } },
    { StandardColumn::ProjectName,       { "Project Name", kUnitNone            } },
    { StandardColumn::Quality,           { "Q",            kUnitNone            } },
};
// clang-format on

// TODO: [EDD] Would be nice if this could be determined at compile time,..
std::unordered_set<std::string> StandardColumnNamesWithSpace();

}  // namespace inertial_explorer

}  // namespace io

}  // namespace nie

#endif  // NIE_FORMATS_INERTIAL_EXPLORER_STANDARD_COLUMNS_HPP
