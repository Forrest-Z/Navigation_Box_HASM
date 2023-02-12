/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "readers.hpp"

#include <glog/logging.h>
#include <nie/core/filesystem.hpp>
#include <nie/core/geometry/frame_conventions.hpp>
#include <nie/core/string.hpp>

namespace nie {

namespace io {

namespace kitti {

namespace detail {

// Parse time formatted like 2011-09-30 11:50:40.088857306
static nie::Timestamp_ns ParseTime(std::string const& str) {
    std::string const Y{str.cbegin() + 0, str.cbegin() + 0 + 4};
    std::string const M{str.cbegin() + 5, str.cbegin() + 5 + 2};
    std::string const D{str.cbegin() + 8, str.cbegin() + 8 + 2};
    std::string const h{str.cbegin() + 11, str.cbegin() + 11 + 2};
    std::string const m{str.cbegin() + 14, str.cbegin() + 14 + 2};
    std::string const s{str.cbegin() + 17, str.end()};

    std::chrono::year_month_day ymd{std::chrono::year(std::stoi(Y)) / std::stoi(M) / std::stoi(D)};

    std::chrono::nanoseconds seconds = nie::ParseFractionalDuration<std::chrono::seconds, std::chrono::nanoseconds>(s);
    seconds += std::chrono::seconds((std::stoul(h) * 60 + std::stoul(m)) * 60);

    return nie::ToGPSTime(ymd, {}, false, seconds);
}

OxtsRecord ParseOxtsFileLine(std::string const& line) {
    // The first 6 values represent:
    //  - lat:   latitude of the oxts-unit (deg)
    //  - lon:   longitude of the oxts-unit (deg)
    //  - alt:   altitude of the oxts-unit (m)
    //  - roll:  roll angle (rad),    0 = level, positive = left side up,      range: -pi   .. +pi
    //  - pitch: pitch angle (rad),   0 = level, positive = front down,        range: -pi/2 .. +pi/2
    //  - yaw:   heading (rad),       0 = east,  positive = counter clockwise, range: -pi   .. +pi
    //  - vn:    velocity towards north (m/s)
    //  - ve:    velocity towards east (m/s)
    //  - vf:    forward velocity, i.e. parallel to earth-surface (m/s)
    //  - vl:    leftward velocity, i.e. parallel to earth-surface (m/s)
    //  - vu:    upward velocity, i.e. perpendicular to earth-surface (m/s)
    //  - ax:    acceleration in x, i.e. in direction of vehicle front (m/s^2)
    //  - ay:    acceleration in y, i.e. in direction of vehicle left (m/s^2)
    //  - ay:    acceleration in z, i.e. in direction of vehicle top (m/s^2)
    //  - af:    forward acceleration (m/s^2)
    //  - al:    leftward acceleration (m/s^2)
    //  - au:    upward acceleration (m/s^2)
    //  - wx:    angular rate around x (rad/s)
    //  - wy:    angular rate around y (rad/s)
    //  - wz:    angular rate around z (rad/s)
    //  - wf:    angular rate around forward axis (rad/s)
    //  - wl:    angular rate around leftward axis (rad/s)
    //  - wu:    angular rate around upward axis (rad/s)
    //  - pos_accuracy:  velocity accuracy (north/east in m)
    //  - vel_accuracy:  velocity accuracy (north/east in m/s)
    //  - navstat:       navigation status (see navstat_to_string)
    //  - numsats:       number of satellites tracked by primary GPS receiver
    //  - posmode:       position mode of primary GPS receiver (see gps_mode_to_string)
    //  - velmode:       velocity mode of primary GPS receiver (see gps_mode_to_string)
    //  - orimode:       orientation mode of primary GPS receiver (see gps_mode_to_string)
    std::vector<double> const values = nie::Split<double>(line, ' ');
    CHECK(values.size() == 30);
    OxtsRecord result{};
    std::copy(values.cbegin(), values.cend(), result.begin());
    return result;
}

}  // namespace detail

std::vector<Timestamp_ns> ReadTimestamps(std::string const& path) {
    LOG(INFO) << "Reading timestamps from file " << path;
    return ReadLines(path, detail::ParseTime);
}

Isometry3qd ReadExtrinsics(std::string const& path) {
    std::vector<std::string> lines = ReadLines(path);
    CHECK(StartsWith(lines[1], "R"));
    CHECK(StartsWith(lines[2], "T"));

    std::vector<double> const r = Split<double>(std::string{lines[1].cbegin() + 3, lines[1].cend()}, ' ');
    std::vector<double> const t = Split<double>(std::string{lines[2].cbegin() + 3, lines[2].cend()}, ' ');

    // The rotation matrix is stored in a row-major way (first complete first row is given, etc.)
    return {Eigen::Map<Eigen::Vector3d const>(t.data()),
            Eigen::Quaterniond(Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> const>(r.data()))};
}

OxtsRecord ReadOxts(std::string const& path) {
    std::ifstream file_stream(path);

    std::string line;
    std::getline(file_stream, line);

    return detail::ParseOxtsFileLine(line);
}

std::vector<Eigen::Vector4f> ReadPoints(std::string const& path) {
    LOG(INFO) << "Reading file " << path;
    std::vector<Eigen::Vector4f> result;
    ReadLines(path, [&result](std::string const& line) -> bool {
        std::vector<float> values = nie::Split<float>(line, ' ');
        CHECK(values.size() == 4);
        result.emplace_back(Eigen::Map<Eigen::Vector4f>(values.data()));
        return true;  // dummy
    });
    return result;
}

PoseCollection ReadGroundTruth(std::string const& path) {
    PoseCollection pose_collection{};

    std::vector<std::string> const lines = ReadLines(path);
    pose_collection.poses.reserve(lines.size());

    for (std::size_t i = 0; i < lines.size(); ++i) {
        std::vector<double> const matrix = Split<double>(std::string{lines[i].cbegin(), lines[i].cend()}, ' ');

        // The elements are given represent the 3x4 transformation matrix by row (first complete first row is given,
        // etc.)
        auto const matrix_map = Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor> const>(matrix.data());
        pose_collection.poses.push_back(PoseRecord{
                static_cast<int>(i),
                PoseRecord::Category::kOdom,
                0,
                {},
                ConvertFrame<Frame::kCv, Frame::kAircraft>(Isometry3qd{matrix_map}),
                {}});
    }

    return pose_collection;
}

}  // namespace kitti

}  // namespace io

}  // namespace nie
