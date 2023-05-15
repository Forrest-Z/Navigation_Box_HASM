/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nie/core/gflags.hpp>
#include <nie/formats/ba_graph.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>
#include <nie/lidar/helper_cloud.hpp>
#include <nie/lidar/io/las_reader.hpp>

#include <unordered_set>

#include "viewer.hpp"

DEFINE_string(in_file_pose, "", "Location of the bounding box pose file.");
DEFINE_string(in_file_bbox, "", "Location of the bounding box file.");
DEFINE_string(in_file_iref, "", "Location of the bounding box iref file.");
DEFINE_uint32(max_number_clouds, 20, "The maximum number of points that will be loaded. Default is 20.");

DEFINE_validator(in_file_pose, nie::ValidateIsFile);
DEFINE_validator(in_file_bbox, nie::ValidateIsFile);
DEFINE_validator(max_number_clouds, nie::ValidateLargerThanZero);

using Point = pcl::PointXYZ;
using CloudVector = std::vector<nie::Cloud<Point>>;

using PoseMapById = std::unordered_map<std::int32_t, std::reference_wrapper<const nie::io::PoseRecord>>;

std::vector<nie::PoseBbox> ReadBounds(PoseMapById const& pose_map, nie::io::BboxCollection const& bbox_collection) {
    std::vector<nie::PoseBbox> bounds;
    bounds.reserve(bbox_collection.boxes.size());

    std::transform(
            bbox_collection.boxes.cbegin(),
            bbox_collection.boxes.cend(),
            std::back_inserter(bounds),
            [&pose_map](nie::io::BboxRecord const& r) -> nie::PoseBbox {
                nie::Isometry3qd const& origin = pose_map.at(r.id).get().isometry;
                nie::Bboxf box{};
                box.min = r.min.cast<float>();
                box.max = r.max.cast<float>();
                return {origin, box};
            });

    return bounds;
}

CloudVector ReadClouds(
        PoseMapById const& pose_map,
        nie::io::InfoRefCollection const& iref_collection,
        std::uint32_t const max_number_clouds) {
    CloudVector clouds;
    clouds.reserve(std::min(iref_collection.info_refs.size(), static_cast<std::size_t>(max_number_clouds)));

    std::unordered_set<std::string> files_read;
    for (nie::io::InfoRefRecord const& r : iref_collection.info_refs) {
        auto [iter, inserted] = files_read.insert(r.path);
        if (inserted) {
            nie::Timestamp_ns const& time = pose_map.at(r.id).get().timestamp;
            clouds.push_back(nie::io::ReadLas<Point>(*iter, nie::ToGPSWeekTime(time).week));
        }
        if (files_read.size() >= max_number_clouds) {
            LOG(INFO) << "More than " << max_number_clouds << " clouds were selected to be viewed. "
                      << "Only going to show the first " << max_number_clouds << ".";
            break;
        }
    }

    return clouds;
}

void BboxViewer() {
    auto const pose_collection = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_pose);
    PoseMapById const pose_by_id =
            nie::io::CreateRecordMap(pose_collection.poses.cbegin(), pose_collection.poses.cend());
    auto const bbox_collection = nie::io::ReadCollection<nie::io::BboxCollection>(FLAGS_in_file_bbox);
    LOG_IF(FATAL, bbox_collection.boxes.empty()) << "No bounding boxes present in supplied file.";

    std::vector<nie::PoseBbox> bounds = ReadBounds(pose_by_id, bbox_collection);

    CloudVector clouds{};
    if (!FLAGS_in_file_iref.empty()) {
        nie::ValidateIsFile("in_file_iref", FLAGS_in_file_iref);
        auto const iref_collection = nie::io::ReadCollection<nie::io::InfoRefCollection>(FLAGS_in_file_iref);
        clouds = ReadClouds(pose_by_id, iref_collection, FLAGS_max_number_clouds);
        CHECK(clouds.size() == std::min(bounds.size(), static_cast<std::size_t>(FLAGS_max_number_clouds)))
                << "Different number of clouds compared to bounds.";
    }

    // Already converts all clouds and bounds to be with respect to a certain point. Note that the assumption made here
    // is that the bounding boxes and point clouds are in the same system.
    nie::Isometry3qd const T_ref_world = bounds[bounds.size() / 2].origin().Inversed();
    for (std::size_t i = 0; i < bounds.size(); ++i) {
        if (i < clouds.size()) {
            nie::TransformCloud(T_ref_world * clouds[i].origin(), &clouds[i]);
            clouds[i].origin() = nie::Isometry3qd::Identity();
        }
        bounds[i].origin() = T_ref_world * bounds[i].origin();
    }

    Viewer viewer("bbox_viewer", bounds, clouds);
    viewer.View();
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, false);

    BboxViewer();

    return 0;
}
