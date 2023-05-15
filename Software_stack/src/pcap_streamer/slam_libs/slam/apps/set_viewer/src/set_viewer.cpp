/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <unordered_set>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <nie/core/gflags.hpp>
#include <nie/drawing/map_viewer.hpp>
#include <nie/formats/ba_graph.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>
#include <nie/lidar/cloud_filter.hpp>
#include <nie/lidar/helper_vertex_se3.hpp>
#include <nie/lidar/io/ba_graph/collection_helper.hpp>
#include <nie/lidar/io/las_reader.hpp>

#include "viewer.hpp"

/// This set_viewer is intended to report or display the las files that are selected based on the given iref file.
/// Depending on the input supplied, there are different modes of the tool:
///   1. max_number_clouds is set to zero, or not
///        When max_number_clouds is set to zero, then there will be no clouds visualized when selecting some bounding
///        boxes on the map. In this case only the las files names will be reported.
///   2. bbox and corresponding pose file given, or not
///        When there are no bbox and corresponding pose file given, then the bounding box information is read for all
///        las files references in the iref. This might take some time. So when the bbox files are given, then that
///        information is taken from those files. In the latter situation the iref file must also be the bbox one.

// clang-format off
DEFINE_string(in_file_iref, "", "Location of iref file with the las files to be displayed.");
DEFINE_string(in_file_bbox_pose, "",
        "Location of bbox file with the las files to be displayed. When provided (together with the bbox file), then "
        "the bounding boxes are read from file, not from the las files. [optional]");
DEFINE_string(in_file_bbox_bbox, "",
        "Location of bbox file with the las files to be displayed. When provided (together with the pose file), then "
        "the bounding boxes are read from file, not from the las files. [optional]");
DEFINE_string(in_file_trace_pose, "", "Location of pose file with the trace to be displayed. [optional]");

DEFINE_uint32(max_number_clouds, 20,
        "The maximum number of point clouds that will be loaded. Default is 20. When set to 0 then no clouds will be "
        "visualized and only the selected las files will be reported.");
DEFINE_double(filter_grid_size, 0.125,
        "Grid size use to filter the point cloud. Must be larger than 0.125 (default value) or 0.0 (no filtering). "
        "Recommended to use a value between 0.125 and 0.25.");
// clang-format on

DEFINE_validator(in_file_iref, nie::ValidateIsFile);
DEFINE_validator(max_number_clouds, nie::ValidateLargerOrEqualToZero);

template <typename Point>
using CloudVector = std::vector<nie::Cloud<Point>>;

void ReadPoseBboxes(
        nie::io::InfoRefCollection const& iref,
        std::vector<nie::PoseBbox>* boxes,
        std::vector<std::string>* file_names) {
    std::unordered_set<std::string> files_read;
    static std::chrono::weeks const gps_week{};  // Timestamps are not of interest
    for (nie::io::InfoRefRecord const& r : iref.info_refs) {
        auto const [iter, inserted] = files_read.insert(r.path);
        if (inserted) {
            boxes->push_back(nie::io::ReadLasHeader<pcl::PointXYZ>(*iter, gps_week).bounds());
            file_names->push_back(*iter);
        }
    }
}

void ReadPoseBboxes(
        nie::io::InfoRefCollection const& iref,
        nie::io::PoseCollection const& pose,
        nie::io::BboxCollection const& bbox,
        std::vector<nie::PoseBbox>* boxes,
        std::vector<std::string>* file_names) {
    std::unordered_map<nie::io::PoseId, std::size_t> pose_id_to_bounds_index_map;
    nie::io::CreatePoseBboxMap(pose.poses, bbox.boxes, &pose_id_to_bounds_index_map, boxes);
    file_names->resize(boxes->size());

    for (nie::io::InfoRefRecord const& r : iref.info_refs) {
        (*file_names)[pose_id_to_bounds_index_map.at(r.id)] = r.path;
    }
}

template <typename Point>
CloudVector<Point> ReadClouds(std::vector<std::string> const& file_names, std::vector<std::size_t> const& indices) {
    if (indices.empty()) {
        return {};
    }

    CloudVector<Point> clouds;

    // Add the points to the clouds in the resulting vector
    static std::chrono::weeks const gps_week{};  // Not of interest
    for (unsigned long index : indices) {
        LOG(INFO) << "Loading points of las file " << file_names[index];
        clouds.emplace_back(nie::io::ReadLas<Point>(file_names[index], gps_week));
    }

    // Filter the point cloud, when specified
    if (FLAGS_filter_grid_size != 0.) {
        nie::CloudFilter<Point> filter{typename nie::CloudFilter<Point>::Parameters(FLAGS_filter_grid_size)};
        std::for_each(
                clouds.begin(),
                clouds.end(),
                std::bind(&nie::CloudFilter<Point>::Filter, filter, std::placeholders::_1));
    }

    return clouds;
}

template <typename Point>
void SelectionCallback(
        nie::PoseBbox const& selection,
        std::vector<nie::PoseBbox> const& bounds,
        std::vector<std::string> const& filenames,
        nie::MapViewer* const map_viewer,
        Viewer<Point>* const cloud_viewer) {
    std::vector<std::size_t> indices;
    for (std::size_t i = 0; i < bounds.size(); ++i) {
        nie::PoseBbox test_bounds = bounds[i].CopyWithOrigin(selection.origin().translation());
        test_bounds.bbox() &= selection.bbox();  // Volume can be negative
        Eigen::Vector2f box_range = test_bounds.bbox().Range().head<2>();
        if ((box_range.array() > 0.).all() && box_range.prod() > 0.) {
            indices.push_back(i);
        }
        if (cloud_viewer && indices.size() == FLAGS_max_number_clouds) {
            LOG(INFO) << "More than " << FLAGS_max_number_clouds << " clouds were selected to be viewed, but "
                      << "only the first " << FLAGS_max_number_clouds << " will be shown.";
            break;
        }
    }

    if (cloud_viewer) {
        LOG(INFO) << "Loading " << indices.size() << " clouds...";
        CloudVector<Point> read_clouds = ReadClouds<Point>(filenames, indices);
        LOG(INFO) << "Clouds loaded";
        cloud_viewer->SetClouds(std::move(read_clouds));
    } else {
        LOG(INFO) << "The following las files are selected:";
        for (std::size_t i : indices) {
            LOG(INFO) << "    " << filenames[i];
        }
    }
    map_viewer->Update({indices.cbegin(), indices.cend()});
}

template <typename Point>
void SetViewer() {
    auto const iref = nie::io::ReadCollection<nie::io::InfoRefCollection>(FLAGS_in_file_iref);

    nie::io::PoseCollection trace_pose{};
    if (!FLAGS_in_file_trace_pose.empty()) {
        nie::ValidateIsFile("in_file_trace_pose", FLAGS_in_file_trace_pose);
        trace_pose = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_trace_pose);
    }

    std::vector<nie::PoseBbox> bounds;
    std::vector<std::string> filenames;

    if (!FLAGS_in_file_bbox_pose.empty() or !FLAGS_in_file_bbox_bbox.empty()) {
        CHECK(!FLAGS_in_file_bbox_pose.empty() and !FLAGS_in_file_bbox_bbox.empty())
                << "If one parameter of in_file_bbox_pose and in_file_bbox_bbox is given, then both are required.";
        nie::ValidateIsFile("in_file_bbox_pose", FLAGS_in_file_bbox_pose);
        nie::ValidateIsFile("in_file_bbox_bbox", FLAGS_in_file_bbox_bbox);

        LOG(INFO) << "Reading the bounding boxes from the bbox files...";
        auto const bbox_pose = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_bbox_pose);
        auto const bbox_bbox = nie::io::ReadCollection<nie::io::BboxCollection>(FLAGS_in_file_bbox_bbox);
        ReadPoseBboxes(iref, bbox_pose, bbox_bbox, &bounds, &filenames);
    } else {
        LOG(INFO) << "Reading the bounding boxes from all las files...";
        ReadPoseBboxes(iref, &bounds, &filenames);
    }
    LOG(INFO) << "Bounding boxes read";

    // Create viewers
    nie::MapViewer map_viewer("map viewer", bounds, trace_pose);
    auto cloud_viewer_ptr = FLAGS_max_number_clouds > 0 ? std::make_unique<Viewer<Point>>("cloud viewer") : nullptr;

    // Show the map_viewer with the callback set for selection
    map_viewer.View([&bounds, &filenames, &map_viewer, &cloud_viewer_ptr](nie::PoseBbox const& selection) {
        SelectionCallback(selection, bounds, filenames, &map_viewer, cloud_viewer_ptr.get());
    });
    if (cloud_viewer_ptr) {
        cloud_viewer_ptr->View();  // Start cloud viewer event loop
    } else {
        // The current thread should be kept alive as the cloud viewer does not in this situation and the map viewer
        // runs in a separate thread
        while (map_viewer.Running()) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, false);

    SetViewer<pcl::PointXYZI>();

    return 0;
}
