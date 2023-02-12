/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "benchmark_poses.hpp"

#include <glog/logging.h>

#include <nie/core/constants.hpp>

#include "correct_euler_singularity.hpp"

namespace nie {

MatchedPosesVector MatchReferenceCollection(
        io::PoseCollection const& pc_reference, io::PoseCollection const& pc_benchmark) {
    MatchedPosesVector poses_matched;
    poses_matched.reserve(pc_benchmark.poses.size());

    for (auto const& pose_benchmark : pc_benchmark.poses) {
        // We can only interpolate if the timestamp is within the range of the reference
        if (pose_benchmark.timestamp < pc_reference.poses.cbegin()->timestamp ||
            pose_benchmark.timestamp > pc_reference.poses.crbegin()->timestamp) {
            VLOG(4) << "Benchmarking pose has no match for timestamp: " << pose_benchmark.timestamp;
            continue;
        }
        // Create matched point
        auto pose_interpolated = pose_benchmark;
        nie::io::InterpolateIsometry(
                pc_reference.poses.begin(),
                pc_reference.poses.end(),
                pose_benchmark.timestamp,
                &pose_interpolated.isometry);
        // Add both points to output pose collection
        poses_matched.push_back(MatchedPoses{pose_benchmark, pose_interpolated});
    }
    return poses_matched;
}

std::vector<PoseDelta> FindPoseDeltas(MatchedPosesVector const& poses_matched) {
    std::vector<PoseDelta> pose_deltas;
    pose_deltas.reserve(poses_matched.size());

    for (auto const& pm : poses_matched) {
        auto delta = pm.first.isometry.Delta(pm.second.isometry);
        // We might not only be interested in the absolute position difference, but also the relative
        // To find this, we have to rotate the found difference into the benchmarking coordinate frame
        auto benchmark_rotation = pm.first.isometry.rotation().toRotationMatrix();
        auto const translation_absolute{delta.translation()};
        Eigen::Vector3d const translation_relative{benchmark_rotation * translation_absolute};

        pose_deltas.push_back({pm.first.timestamp, translation_absolute, translation_relative, delta.rotation()});
    }
    return pose_deltas;
}

void ValidatePoseCollectionCompatibility(
        nie::io::PoseCollection const& pc_reference, nie::io::PoseCollection const& pc_benchmark) {
    // Check if both collections are using the same coordinate system
    CHECK_EQ(pc_reference.header.authority, pc_benchmark.header.authority)
            << "PoseCollections not compatible. Pose files do not have the same coordinate system.";

    CHECK(pc_reference.header.HasTimestampPerRecord())
            << "Reference pose collection does not have timestamps per record.";
    CHECK(pc_benchmark.header.HasTimestampPerRecord())
            << "Benchmarking pose collection does not have timestamps per record.";

    VLOG(1) << "First Reference PoseRecord timestamp: " << pc_reference.poses.cbegin()->timestamp;
    VLOG(1) << "First Benchmark PoseRecord timestamp: " << pc_benchmark.poses.cbegin()->timestamp;
    VLOG(1) << "Last Reference PoseRecord timestamp: " << pc_reference.poses.crbegin()->timestamp;
    VLOG(1) << "Last Benchmark PoseRecord timestamp: " << pc_benchmark.poses.crbegin()->timestamp;

    // Check timestamps if they are within the same ranges
    // If not, fatal and provide time diff
    auto const time_overlap_benchmark_to_ref = std::chrono::duration_cast<std::chrono::nanoseconds>(
            pc_benchmark.poses.crbegin()->timestamp - pc_reference.poses.cbegin()->timestamp);
    auto const time_overlap_ref_to_benchmark = std::chrono::duration_cast<std::chrono::nanoseconds>(
            pc_reference.poses.crbegin()->timestamp - pc_benchmark.poses.cbegin()->timestamp);

    CHECK_GT(time_overlap_benchmark_to_ref.count(), 0.0)
            << "PoseCollections not compatible. Benchmarking time range starts "
            << -time_overlap_benchmark_to_ref.count() << " nsec after the end of the reference time range.";
    CHECK_GT(time_overlap_ref_to_benchmark.count(), 0.0)
            << "PoseCollections not compatible. Reference time range starts " << -time_overlap_ref_to_benchmark.count()
            << " nsec after the end of the benchmarking time range.";
}

void WriteMatchedPosesAsCollection(
        MatchedPosesVector const& poses_matched,
        io::PoseCollection::Header const& pc_header,
        std::string const& filename) {
    io::PoseCollection pc_write;
    pc_write.header = pc_header;
    // Converting the vector of pose pairs into a single vector
    std::transform(
            poses_matched.begin(),
            poses_matched.end(),
            std::back_inserter(pc_write.poses),
            [&pc_write](MatchedPoses const& mp) {
                pc_write.poses.push_back(mp.first);
                return mp.second;
            });
    nie::io::Write(pc_write, filename);
}

void WritePoseDeltasToCsv(std::vector<PoseDelta> const& pose_deltas, std::string const& filename) {
    std::ofstream csv_file{filename};
    std::string delimiter{','};
    std::string csv_header{
            "timestamp,absolute_dx,absolute_dy,absolute_dz,relative_dx,relative_dy,relative_dz,droll,dpitch,dyaw"};
    csv_file << csv_header << "\n";
    for (auto const& delta : pose_deltas) {
        // Rotation order is assumed to be yaw last. E.g. Rz * Ry * Rx
        // As such, the angles are also stored in the vector as [yaw, pitch, roll]
        auto delta_rpy = delta.rotation.toRotationMatrix().eulerAngles(2, 1, 0);

        nie::CorrectEulerSingularity(&delta_rpy);

        // Write to file
        csv_file << delta.timestamp << delimiter << delta.translation_absolute.x() << delimiter
                 << delta.translation_absolute.y() << delimiter << delta.translation_absolute.z() << delimiter
                 << delta.translation_relative.x() << delimiter << delta.translation_relative.y() << delimiter
                 << delta.translation_relative.z() << delimiter << delta_rpy[2] * nie::kRad2Deg<double> << delimiter
                 << delta_rpy[1] * nie::kRad2Deg<double> << delimiter << delta_rpy[0] * nie::kRad2Deg<double> << "\n";
    }
    csv_file.close();
}

}  // namespace nie