/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "convert_from_kitti.hpp"

#include <nie/core/filesystem.hpp>
#include <nie/core/geometry/frame_conventions.hpp>
#include <nie/formats/kitti/readers.hpp>
#include <nie/geo_convert/geo_convert.hpp>

#include "tool/io.hpp"

// The oxts files with the pose information is given per sweep, so many files will need to be aggregated.

// Helper class to convert the read oxts values to a pose and return it relative to the start (first pose).
class PoseCreator {
public:
    nie::io::PoseRecord operator()(nie::io::kitti::OxtsRecord values) {
        // Convert angles from degrees to radians
        std::for_each(values.begin(), values.begin() + 3, [](double& v) { v = nie::Deg2Rad<>(v); });

        // Determine scale based on the first pose received
        DetermineScale(values);

        // The resulting pose that will be returned
        nie::io::PoseRecord pose{};

        // Calculate the pose
        nie::Isometry3qd curr_pose = CreatePose(values);

        // Calculate the pose relative to the start
        if (rev_init_pose_ == nie::Isometry3qd::Identity()) {
            rev_init_pose_ = curr_pose.Inversed();
            pose.isometry = nie::Isometry3qd::Identity();
        } else {
            pose.isometry = rev_init_pose_ * curr_pose;
        }

        // Calculate the information matrix
        pose.information = CreateInfMatrix(values);

        return pose;
    }

private:
    void DetermineScale(nie::io::kitti::OxtsRecord const& values) {
        if (scale_ == -1.) {
            scale_ = std::cos(values[0]);
        }
    }

    [[nodiscard]] nie::Isometry3qd CreatePose(nie::io::kitti::OxtsRecord const& values) const {
        Eigen::Vector3d translation = Eigen::Vector3d::Zero();
        translation[0] = scale_ * r_earth_ * values[1];
        translation[1] = scale_ * r_earth_ * std::log(std::tan((values[0] + nie::kPi<> / 2.) / 2.));
        translation[2] = values[2];  // altitude

        Eigen::Quaterniond rotation =
                nie::EulerToQuaternion<nie::Axis::Z, nie::Axis::Y, nie::Axis::X>(values[5], values[4], values[3]);

        nie::Isometry3qd result{translation, rotation};
        result = nie::ConvertFrame<nie::Frame::kVehicle, nie::Frame::kAircraft>(result);

        return result;
    }

    [[nodiscard]] static Eigen::Matrix<double, 6, 6> CreateInfMatrix(nie::io::kitti::OxtsRecord const& values) {
        // Only one value for the accuracy for the x and y position is supplied, so rest is fixed.

        // Standard deviation for a height in meter
        double constexpr kStdDevHeight = 0.05;
        // Standard deviation for the vector part of a quaternion, based on 0.1146 degrees in overall rotation
        // Note that quaternion = cos(angle/2) + unit vector * sin(angle/2)
        double const kStdDevRotation = nie::Deg2Rad<>(0.1146 / 2.);

        Eigen::Array<double, 6, 1> const std_dev = (Eigen::ArrayXd(6) << Eigen::Array2d::Constant(values[23]),
                                                    kStdDevHeight,
                                                    Eigen::Array3d::Constant(kStdDevRotation))
                                                           .finished();
        return Eigen::Matrix<double, 6, 6>{std_dev.pow(-2.).matrix().asDiagonal()};
    }

    nie::Isometry3qd rev_init_pose_{nie::Isometry3qd::Identity()};

    static double constexpr r_earth_ = 6378137.;
    double scale_ = -1.;
};

void ConvertFromKitti() {
    std::string const& pose_ext = nie::io::graph::Extension<nie::io::PoseCollection>();

    InPathExistsOrFatal();
    CheckOutPathsLocationsOrFatal();

    boost::filesystem::path path;
    GetAndCheckInPathsForExtensionOrFatal(".txt", &path);
    boost::filesystem::path const in_file_name = path;
    GetAndCheckOutPathsForExtensionOrFatal(pose_ext, &path);
    std::string const out_file_name = path.string();

    // Read the timestamps belonging to the poses
    std::vector<nie::Timestamp_ns> const timestamps = nie::io::kitti::ReadTimestamps(in_file_name.string());

    // Look up all oxts files
    std::vector<boost::filesystem::path> const oxts_files =
            nie::FindFiles(in_file_name.parent_path() / "data", ".*\\.txt");
    CHECK(oxts_files.size() == timestamps.size())
            << "Number of oxts files and timestamps do not match: " << oxts_files.size() << " vs. "
            << timestamps.size();
    CHECK(std::is_sorted(oxts_files.cbegin(), oxts_files.cend()));  // FindFiles implementation expectation

    // Initialize output
    LOG(INFO) << "Writing " << pose_ext << " file '" << out_file_name << "'.";
    nie::io::PoseHeader header{};
    header.Set(nie::io::PoseHeader::kHasTimestampPerRecord);
    header.Set(nie::io::PoseHeader::kHasPoseInformationPerRecord);
    nie::io::PoseCollectionStreamWriter writer(out_file_name, header);

    // Read and write pose records
    PoseCreator pose_creator;
    for (std::size_t i = 0; i < timestamps.size(); ++i) {
        nie::io::PoseRecord pose = pose_creator(nie::io::kitti::ReadOxts(oxts_files[i].string()));
        pose.id = i;
        pose.timestamp = timestamps[i];
        writer.Write(pose);
    }

    LOG(INFO) << "Done";
}
