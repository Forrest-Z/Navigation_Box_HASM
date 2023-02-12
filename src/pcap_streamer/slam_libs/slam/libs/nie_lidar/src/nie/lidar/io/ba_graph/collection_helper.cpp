/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "collection_helper.hpp"

#include <nie/formats/ba_graph/collection_helper.hpp>

namespace nie {

namespace io {

std::unordered_map<PoseId, PoseBbox> CreatePoseBboxMap(
        std::vector<PoseRecord> const& poses, std::vector<BboxRecord> const& boxes) {
    std::unordered_map<PoseId, PoseBbox> map;

    for (std::size_t i = 0; i < poses.size(); ++i) {
        auto const& pose = poses[i];
        auto const& bbox = boxes[i];
        CHECK_EQ(pose.id, bbox.id) << "Pose file and Bbox file do not correspond to each other.";

        Bboxf b;
        b.min = bbox.min.cast<float>();
        b.max = bbox.max.cast<float>();

        map.emplace(pose.id, PoseBbox{pose.isometry, b});
    }

    return map;
}

void CreatePoseBboxMap(
        std::vector<nie::io::PoseRecord> const& poses,
        std::vector<nie::io::BboxRecord> const& boxes,
        std::unordered_map<nie::io::PoseId, std::size_t>* pose_id_to_bound_index_map,
        std::vector<nie::PoseBbox>* bounds) {
    bounds->clear();
    bounds->reserve(poses.size());

    for (std::size_t i = 0; i < poses.size(); ++i) {
        auto const& pose = poses[i];
        auto const& bbox = boxes[i];
        CHECK_EQ(pose.id, bbox.id) << "Pose file and Bbox file do not correspond to each other.";

        Bboxf b;
        b.min = bbox.min.cast<float>();
        b.max = bbox.max.cast<float>();

        pose_id_to_bound_index_map->emplace(pose.id, i);
        bounds->emplace_back(pose.isometry, b);
    }
}

Eigen::Matrix<double, 6, 6> InformationMatrixFromIsometry(
        nie::Isometry3qd const& ei, double const w, double const delta_percentage) {
    // Minimum movement is a millimeter to determine the translation sigma
    constexpr double kMinTranslation = 0.001;
    // Minimum rotation is 0.001 degrees to determine the rotation sigma
    // 0.00001745329 = 0.001 degrees
    constexpr double kMinRotation = 0.00001745329;

    double const dt = std::max(ei.translation().norm(), kMinTranslation) * delta_percentage;
    double const dr = std::max(Eigen::AngleAxisd{ei.rotation()}.angle(), kMinRotation) * delta_percentage;

    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();
    information.diagonal().head<3>().setConstant(1.0 / (dt * dt) * w);
    information.diagonal().tail<3>().setConstant(1.0 / (dr * dr) * w);

    return information;
}

nie::io::PoseEdgeRecord MakeEdgeRecord(
        nie::io::PoseId const id0,
        nie::io::PoseId const id1,
        nie::Isometry3qd const& ei,
        double const w,
        double const delta_percentage) {
    return nie::io::PoseEdgeRecord{
            id0,
            id1,
            nie::io::PoseEdgeRecord::Category::kOdom,
            ei,
            nie::io::InformationMatrixFromIsometry(ei, w, delta_percentage)};
}

constexpr double kMedianDistanceScale = 3.0;

double GetMedianDistanceThreshold(PoseCollection const& collection) {
    return nie::io::GetMedianDistance(collection) * kMedianDistanceScale;
}

}  // namespace io

}  // namespace nie