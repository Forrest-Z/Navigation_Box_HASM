/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <vector>

#include <nie/formats/ba_graph/pose_collection.hpp>

namespace nie {

using MatchedPoses = std::pair<io::PoseRecord, io::PoseRecord>;
using MatchedPosesVector = std::vector<MatchedPoses>;

/// Struct to contain pose difference info
/// \param timestamp the timestamp of the original benchmarking pose
/// \param pose_diff difference in poses between benchmark and interpolated pose
struct PoseDelta {
    nie::Timestamp_ns timestamp;
    Eigen::Vector3d translation_absolute;
    Eigen::Vector3d translation_relative;
    Eigen::Quaterniond rotation;
};

/// Matches the reference pose collection to the benchmarking pose collection
///
/// \param pc_reference  PoseCollection to be used as reference
/// \param pc_benchmark  PoseCollection to be benchmarked
/// \return  PoseCollection containing the benchmark and interpolated reference poses
MatchedPosesVector MatchReferenceCollection(
        io::PoseCollection const& pc_reference, io::PoseCollection const& pc_benchmark);

/// Finds the pose differences between alternating poses.
///
/// \param poses_matched vector of io::PoseRecord with original and matched poses alternating
/// \return  vector of PoseDelta, containing the timestamp and (absolute/relative) pose differences
std::vector<PoseDelta> FindPoseDeltas(MatchedPosesVector const& poses_matched);

/// Checks if the PoseCollections are compatible for benchmarking
///
/// \param pc_reference pose collection to be used as reference
/// \param pc_benchmark pose collection to be benchmarked
/// \remark Will FATAL if not compatible.
void ValidatePoseCollectionCompatibility(
        nie::io::PoseCollection const& pc_reference, nie::io::PoseCollection const& pc_benchmark);

/// Writes a matched poses vector to a .pose file
///
/// \param poses_matched vector contained matched poses pairs
/// \param pc_header nie::io::PoseCollection header containing pc information
/// \param filename path where to write the file
void WriteMatchedPosesAsCollection(
        MatchedPosesVector const& poses_matched,
        io::PoseCollection::Header const& pc_header,
        std::string const& filename);

/// Writes found pose deltas to csv file
///
/// \param pose_deltas vector contained matched poses pairs
/// \param filename path where to write the file
void WritePoseDeltasToCsv(std::vector<PoseDelta> const& pose_deltas, std::string const& filename);

}  // namespace nie
