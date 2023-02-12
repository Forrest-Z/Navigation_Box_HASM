/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "lidar_pose_writer.hpp"

#include <nie/formats/ba_graph/collection_helper.hpp>

namespace {

nie::io::PoseHeader CreateHeader() {
    nie::io::PoseHeader pose_collection_header{};
    pose_collection_header.Set(nie::io::PoseHeader::Flag::kHasTimestampPerRecord);
    pose_collection_header.Unset(nie::io::PoseHeader::Flag::kHasPoseInformationPerRecord);
    pose_collection_header.Set(nie::io::PoseHeader::Flag::kHasEdgeInformationPerRecord);
    nie::io::SetNieAuthority(&pose_collection_header);
    return pose_collection_header;
}

}  // namespace

namespace nie {

LidarPoseWriter::LidarPoseWriter(std::string const& file_name, double const loam_sd_scale)
    : pose_collection_writer_{file_name, CreateHeader()},
      has_previous_{false},
      loam_sd_scale_{loam_sd_scale},
      previous_pose_{} {}

void LidarPoseWriter::ProcessSweep(io::PoseRecord const& pose) {
    pose_collection_writer_.Write(pose);

    if (!has_previous_) {
        previous_pose_ = pose;
        has_previous_ = true;
        return;
    }

    Isometry3qd pose_delta = previous_pose_.isometry.TransformInverseLeft(pose.isometry);

    double const percentage = 0.025 * loam_sd_scale_;
    double const motion_sd = pose_delta.translation().norm() * percentage;
    double const motion_variance = std::pow(motion_sd, 2);
    double const rotation_sd = std::sin((Eigen::AngleAxisd{pose_delta.rotation()}.angle() * percentage) / 2.0);
    double const rotation_variance = std::pow(rotation_sd, 2);

    Eigen::Matrix<double, 6, 6> covariance{Eigen::Matrix<double, 6, 6>::Zero()};
    covariance.diagonal().head<3>().setConstant(motion_variance);
    covariance.diagonal().tail<3>().setConstant(rotation_variance);
    Eigen::Matrix<double, 6, 6> inf;
    CovarianceToInformation(covariance, &inf);

    pose_collection_writer_.Write(
            io::PoseEdgeRecord{previous_pose_.id, pose.id, io::PoseEdgeRecord::Category::kOdom, pose_delta, inf});

    previous_pose_ = pose;
}

}  // namespace nie
