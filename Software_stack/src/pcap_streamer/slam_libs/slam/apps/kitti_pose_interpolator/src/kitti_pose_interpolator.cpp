/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <nie/core/geometry/frame_conventions.hpp>
#include <nie/core/gflags.hpp>
#include <nie/formats/ba_graph/collection_writer.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>
#include <nie/formats/kitti/readers.hpp>
#include <nie/lidar/cloud_matcher.hpp>
#include <nie/lidar/trace_cloud.hpp>

/// This app reads a kitti timestamp file (containing relative timestamps) and interpolates the poses from the given
/// pose file at the supplied times. Additionally, all poses are transformed, such that the first pose equals identity.

DEFINE_string(in_file_pose, "", "Filepath to input .pose file to resampled.");
DEFINE_string(in_file_times, "", "Filepath to input kitti middle lidar sweep timestamps file.");
DEFINE_string(in_file_calib_imu_to_lidar, "", "Filepath to input imu to lidar calibration file.");
DEFINE_string(in_file_calib_lidar_to_cam, "", "Filepath to input lidar to camera calibration file.");
DEFINE_string(in_file_ground_truth, "", "Filepath to input ground truth pose file.");
DEFINE_string(out_file_pose, "", "Filepath to output .pose file.");
DEFINE_string(range, "", "Subselection.");
DEFINE_bool(ground_truth, false, "Output the ground truth, which takes the gps only as input pose file.");

DEFINE_validator(in_file_pose, nie::ValidateIsFile);
DEFINE_validator(in_file_times, nie::ValidateIsFile);
DEFINE_validator(in_file_calib_imu_to_lidar, nie::ValidateIsFile);
DEFINE_validator(in_file_calib_lidar_to_cam, nie::ValidateIsFile);
DEFINE_validator(in_file_ground_truth, nie::ValidateIsFile);

using Times = std::vector<nie::Timestamp_ns>;

// Returned calibration is the full transformation going from the imu to the reference left gray camera 0.
nie::Isometry3qd ReadCalibrations(std::string const& imu_to_lidar, std::string const& lidar_to_camera) {
    nie::Isometry3qd const result = nie::io::kitti::ReadExtrinsics(imu_to_lidar);
    return nie::io::kitti::ReadExtrinsics(lidar_to_camera) * result;
}

std::vector<nie::io::PoseRecord> InterpolatePoses(std::vector<nie::io::PoseRecord> const& poses, Times const& times) {
    std::vector<nie::io::PoseRecord> result;
    result.reserve(times.size());

    auto const& first_time = poses.front().timestamp;
    auto const& last_time = poses.back().timestamp;
    auto iterators = std::make_pair(poses.cbegin(), poses.cend());

    // All regular poses are compared with the first gps coordinate as that is their reference point
    nie::Isometry3qd first_pose_inv = nie::Isometry3qd::Identity();
    for (std::size_t i = 0; i < times.size(); ++i) {
        CHECK(times[i] >= first_time) << "Sample timestamp " << times[i] << " before the start of trace " << first_time;
        CHECK(times[i] <= last_time) << "Sample timestamp " << times[i] << " after start of trace " << last_time;

        result.emplace_back();
        auto& p = result.back();

        p.id = i;
        p.timestamp = times[i];
        iterators = nie::io::InterpolateIsometry(iterators.first, poses.cend(), p.timestamp, &p.isometry);
        if (i == 0) {
            std::swap(first_pose_inv, p.isometry);
            first_pose_inv.Inverse();
        } else {
            p.isometry = first_pose_inv * p.isometry;
        }
        p.category = iterators.first->category;
    }

    return result;
}

void Align(nie::io::PoseCollection const& ground_truth, nie::io::PoseCollection* resampled_poses) {
    nie::Cloud<pcl::PointXYZ> const cloud_gt =
            nie::MakeTraceCloudFromTrace(ground_truth.poses.cbegin(), ground_truth.poses.cend());
    nie::Cloud<pcl::PointXYZ> const cloud =
            nie::MakeTraceCloudFromTrace(resampled_poses->poses.cbegin(), resampled_poses->poses.cend());

    auto transformation = nie::Isometry3qd::Identity();
    nie::Cloud<pcl::PointXYZ> result{};

    nie::CloudMatcher<pcl::PointXYZ>::Parameters params;
    params.transformation_eps = 1.e-9;
    params.euclidean_fitness_eps = -1.0;
    params.max_correspondence_distance = 5.;
    params.ransac_outlier_rejection_threshold = 5.;
    params.max_iterations = 200;
    params.ransac_max_iterations = 200;
    nie::CloudMatcher<pcl::PointXYZ> matcher{{params}};
    CHECK(matcher.Match(cloud_gt, cloud, &transformation, &result));

#if false
    // Also change CMakeLists.txt to use lib nie_drawing and include nie/drawing/drawing.hpp
    nie::PclViewer viewer{"result"};
    auto const clouds = std::vector{cloud_gt, result, cloud};
    nie::AddClouds<pcl::PointXYZ>(clouds, nie::GetColorHandlersRegisteredTraces3(), {"0", "1", "2"}, &viewer);
    nie::AddCoordinateSystem<pcl::PointXYZ>(clouds, &viewer);
    viewer.View();
#endif

    std::for_each(resampled_poses->poses.begin(), resampled_poses->poses.end(), [&transformation](auto& pose) {
        pose.isometry = transformation * pose.isometry;
    });
}

// Add origin pose as benchmark is relative to first pose
void AddOrigin(nie::io::PoseCollection* resampled_poses) {
    auto& poses = resampled_poses->poses;

    std::vector<nie::io::PoseRecord> new_poses(poses.size() + 1);

    // Add origin
    new_poses[0] = poses.front();
    new_poses[0].id = 0;
    new_poses[0].isometry = nie::Isometry3qd::Identity();

    // Add remainder and update their ids
    std::copy(poses.begin(), poses.end(), new_poses.begin() + 1);
    std::for_each(new_poses.begin() + 1, new_poses.end(), [](auto& pose) { ++pose.id; });

    // Replace existing poses with new ones
    std::swap(new_poses, poses);
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    LOG(INFO) << "Reading pose file: " << FLAGS_in_file_pose;
    auto const poses = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_pose);

    LOG(INFO) << "Reading timestamp file: " << FLAGS_in_file_times;
    Times times = nie::io::kitti::ReadTimestamps(FLAGS_in_file_times);
    if (!FLAGS_range.empty()) {
        auto range = nie::Split<std::size_t>(FLAGS_range, '-');
        ++range.back();
        if (range.back() > times.size()) {
            LOG(INFO) << "range end can only go up to " << times.size() - 1 << ", value updated in processing.";
            range.back() = times.size();
        }
        times = {times.begin() + range.front(), times.begin() + range.back()};
    }

    nie::io::PoseCollection const ground_truth = nie::io::kitti::ReadGroundTruth(FLAGS_in_file_ground_truth);

    nie::io::PoseCollection resampled_poses{};
    nie::io::CopyAuthority(poses.header, &resampled_poses.header);
    resampled_poses.header.Set(nie::io::PoseHeader::kHasTimestampPerRecord);
    resampled_poses.poses = InterpolatePoses(poses.poses, times);

    // The full calibration goes from the imu (in vehicle system) to the left camera (in cv system).
    // The poses given are for the imu reference frame and in the aircraft system.
    // That is why the poses are
    //  - first transformed from aircraft to vehicle,
    //  - then the calibration is applied, and
    //  - finally the converted from cv to aircraft again.
    nie::Isometry3qd const calib = ReadCalibrations(FLAGS_in_file_calib_imu_to_lidar, FLAGS_in_file_calib_lidar_to_cam);
    nie::Isometry3qd const calib_inv = calib.Inversed();
    std::for_each(
            resampled_poses.poses.begin(),
            resampled_poses.poses.end(),
            [&calib, &calib_inv](nie::io::PoseRecord& pose) {
                pose.isometry = nie::ConvertFrame<nie::Frame::kAircraft, nie::Frame::kVehicle>(pose.isometry);
                pose.isometry = calib * pose.isometry * calib_inv;
                pose.isometry = nie::ConvertFrame<nie::Frame::kCv, nie::Frame::kAircraft>(pose.isometry);
            });

    // Even though the resampling is done above, this process is not perfect as the exact sampling times are unknown.
    // Therefore as last step the full trajectory is aligned to the ground truth as best as possible to enabling a
    // "fair" comparison.
    Align(ground_truth, &resampled_poses);

    AddOrigin(&resampled_poses);

    LOG(INFO) << "Writing updated pose to file: " << FLAGS_out_file_pose;
    nie::io::Write(resampled_poses, FLAGS_out_file_pose);
}
