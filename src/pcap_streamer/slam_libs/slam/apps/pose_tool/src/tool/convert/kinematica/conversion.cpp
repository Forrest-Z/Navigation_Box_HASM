/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "conversion.hpp"

#include <algorithm>  // for std::transform
#include <cassert>
#include <chrono>
#include <string>

#include <nie/core/geometry/isometry3.hpp>  // for nie::Isometry3qd
#include <nie/core/geometry/rotation.hpp>   // for nie::EulerToQuaternion
#include <nie/geo_convert/geo_convert.hpp>  // for nie::geo_convert::Wgs84ToUtm

namespace nie {

namespace kinematica {

/// Convert latitude and longitude (and height) to Isometry3qd::Translation in UTM
/// \param latitude latitude in degrees
/// \param longitude in degrees
/// \param height in meters
/// \param [out] gamma meridian convergence in radians
/// \param [out] translation converted translation
static void LatLon2Translation(
        double const latitude,
        double const longitude,
        double const height,
        double* gamma,
        nie::Isometry3qd::Translation* translation) {
    double easting{};
    double northing{};
    double k{};
    std::string utm_zone{};
    nie::geo_convert::Wgs84ToUtm(latitude, longitude, &easting, &northing, &utm_zone, gamma, &k);
    *translation = nie::Isometry3qd::Translation{easting, northing, height};
}

/// Convert an unix timestamp defined in seconds + microseconds to GPS time
/// \param unix_timestamp_seconds unix timestamp in seconds
/// \param microseconds additional microseconds to the timestamp
/// \return converted GPS time
static nie::Timestamp_ns UnixTimestamp2GpsTime(long const unix_timestamp_seconds, long const microseconds) {
    auto const duration = std::chrono::seconds(unix_timestamp_seconds) + std::chrono::microseconds(microseconds);
    std::chrono::time_point<std::chrono::system_clock> const system_time_point(duration);
    return std::chrono::to_gps_time<std::chrono::nanoseconds>(system_time_point);
}

/// Convert a KinematicaCsvRecord row to pose record
/// \param csv_row KinematicaCsvRecord row to be converted
/// \return converted PoseRecord
static nie::io::PoseRecord ConvertCsvRecordRowToPoseRecord(nie::io::KinematicaCsvRecord const& csv_row) {
    nie::io::PoseRecord pose_record{};

    pose_record.timestamp = UnixTimestamp2GpsTime(csv_row.unix_time, csv_row.microseconds);

    // Create variable to store the meridian convergence correction
    double gamma_radians{};

    // Transform the lat/lon to UTM and store in our output pose_record
    LatLon2Translation(
            csv_row.latitude, csv_row.longitude, csv_row.height, &gamma_radians, &pose_record.isometry.translation());

    // We are outputting an UTM position and orientation.
    // But the orientation from Kinematica does not match this.
    // Both UTM and Kinematica assume a right hand coordinate system.
    // With roll around x axis, pitch around y axis, yaw around z axis.
    // But, at the true origin:
    // UTM assumes y north, x east, z up
    // Kinematica assumes x north, y east, z down

    // Thus the solution to handling this is to read the Kinematica orientation as if rotated 180 around the x axis.
    // We also assumed to require to rotate -90 degrees to compensate 0 heading to be east instead of north, but from
    // the data we get it seems that the heading already is correct. Thus, we ignore this.
    // So, we take pitch as -pitch and yaw as -heading. The roll remains the same. Source:
    // https://confluence.navinfo.eu/display/AIIMPRIUS/01_General+Overview?preview=/23232745/47809605/Spatial%20FOG%20Dual%20Reference%20Manual%20v1.3.pdf

    // The Kinematica CSV format uses degrees
    double const roll_radians = Deg2Rad(csv_row.roll);
    double const pitch_radians = -Deg2Rad(csv_row.pitch);
    double const yaw_radians = -Deg2Rad(csv_row.heading);

    // The Advanced Navigation coordinate system defines a rotation first around yaw, then pitch, then roll:
    Eigen::Quaterniond kinematica_orientation = nie::EulerToQuaternion(roll_radians, pitch_radians, yaw_radians);

    // We have to compensate for the meridian convergence
    // From wiki: https://en.wikipedia.org/wiki/Transverse_Mercator_projection#Convergence
    // " The convergence angle γ at a point on the projection is defined by the angle measured from the projected
    // meridian, which defines true north, to a grid line of constant x, defining grid north. Therefore, γ is positive
    // in the quadrant north of the equator and east of the central meridian and also in the quadrant south of the
    // equator and west of the central meridian. "
    // This means, the meridian convergence should not be sign flipped to convert true north to grid north
    Eigen::Quaterniond meridian_correction{Eigen::AngleAxisd{gamma_radians, Eigen::Vector3d::UnitZ()}};

    // Because we work with a local to global system, we apply the meridian convergence correction and heading shift
    // last:
    pose_record.isometry.rotation() = meridian_correction * kinematica_orientation;

    return pose_record;
}

nie::io::PoseCollection ConvertKinematicaCsvToPoseCollection(
        nie::io::KinematicaRecordCollection const& kinematica_csv) {
    nie::io::PoseCollection pose_collection{};
    pose_collection.header.Set(nie::io::PoseHeader::kHasTimestampPerRecord);
    pose_collection.poses.reserve(kinematica_csv.size());

    // PoseIds are assigned incrementally, which is done by the lambda function below
    nie::io::PoseId id{0};
    std::transform(
            kinematica_csv.begin(),
            kinematica_csv.end(),
            std::back_inserter(pose_collection.poses),
            [&id](nie::io::KinematicaCsvRecord const& csv_row) -> nie::io::PoseRecord {
                nie::io::PoseRecord pose_record = ConvertCsvRecordRowToPoseRecord(csv_row);
                pose_record.id = id++;
                return pose_record;
            });

    assert(pose_collection.poses.size() == kinematica_csv.size());
    return pose_collection;
}

}  // namespace kinematica

}  // namespace nie
