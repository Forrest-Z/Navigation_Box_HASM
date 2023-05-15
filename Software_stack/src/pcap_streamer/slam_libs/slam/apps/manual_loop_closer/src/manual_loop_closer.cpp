/* Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <iomanip>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <nie/core/gflags.hpp>
#include <nie/drawing/map_viewer.hpp>
#include <nie/formats/ba_graph.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>
#include <nie/formats/calib3d/lidar_extrinsics.hpp>
#include <nie/lidar/io/ba_graph/collection_helper.hpp>
#include <nie/lidar/io/lidar_streamer.hpp>

#include <Eigen/Geometry>
#include <nie/core/geometry/frame_conventions.hpp>

#include "las_loader.hpp"
#include "las_pair.hpp"
#include "pair_viewer.hpp"

// clang-format off
DEFINE_string(in_file_csv_loops, "", "Filepath to input .csv file containing the loops to check.");

DEFINE_string(in_file_orbb_pose, "", "Location of the pose file containing the bounding box poses (oriented).");
DEFINE_string(in_file_orbb_bbox, "", "Location of the bbox file containing the bounding box boxes (oriented).");
DEFINE_string(in_file_bbox_iref, "", "Location of the iref file containing the bounding box irefs (axis aligned or oriented las file locations).");

DEFINE_string(in_file_aabb_pose, "", "Location of the pose file containing the bounding box poses (axis aligned).");

DEFINE_string(in_file_trace_pose, "", "Location of the pose file containing the odometry poses (gnss/imu or lidar).");

DEFINE_string(in_file_lidar_extrinsics, "", "Location of the lidar extrinsics (or empty - identity).");
DEFINE_string(lidar_id, "", "Identifier of lidar to be used.");

DEFINE_string(out_file_aabb_edge,"",
              "Location of the .pose file to which the found loop closure matches will be written with transformations "
              "between the trajectory poses.");

DEFINE_double(filter_grid_size, 0.125,
              "Grid size use to filter the point cloud. Must be larger than 0.125 (default value). Recommended to use a "
              "value between 0.125 and 0.25. [optional]");
// clang-format on

DEFINE_validator(in_file_csv_loops, nie::ValidateIsFile);

DEFINE_validator(in_file_orbb_pose, nie::ValidateIsFile);
DEFINE_validator(in_file_orbb_bbox, nie::ValidateIsFile);
DEFINE_validator(in_file_bbox_iref, nie::ValidateIsFile);

DEFINE_validator(in_file_aabb_pose, nie::ValidateIsFile);

DEFINE_validator(out_file_aabb_edge, nie::ValidateParentDirExists);

template <typename PointT>
class Handler {
public:
    Handler(std::vector<std::pair<nie::io::PoseId, nie::io::PoseId>> const& pairs,
            std::unordered_map<nie::io::PoseId, std::reference_wrapper<nie::io::PoseRecord>> const& cld_aabb_pose_map,
            std::unordered_map<nie::io::PoseId, std::reference_wrapper<nie::io::InfoRefRecord>> const&
                    cld_aabb_iref_map,
            std::unordered_map<nie::io::PoseId, std::size_t> const& map_orbb_pose_id_to_index_map,
            PairViewer<PointT>* p_cld_viewer,
            nie::MapViewer* p_map_viewer,
            nie::io::PoseCollection* p_cld_aabb_edge)
        : pairs_(pairs),
          cld_aabb_pose_map_(cld_aabb_pose_map),
          cld_aabb_iref_map_(cld_aabb_iref_map),
          map_orbb_pose_id_to_index_map_(map_orbb_pose_id_to_index_map),
          cld_viewer_(*p_cld_viewer),
          map_viewer_(*p_map_viewer),
          cld_aabb_edge_(*p_cld_aabb_edge),
          pair_index_(0) {
        Update();
    }

    void IncPairIndex() {
        if (pair_index_ == (pairs_.size() - 1)) {
            std::cout << "All pairs have been processed." << std::endl;
            std::cout << "Close the viewer to write the results to file." << std::endl;
            return;
        }

        std::cout << "Loading next pair..." << std::endl;
        pair_index_++;
        Update();
        std::cout << "Loaded pair." << std::endl;
    }

    void RemoveActiveEdge() {
        if (!cld_aabb_edge_.edges.empty() && (cld_aabb_edge_.edges.back().id_begin == pairs_[pair_index_].first ||
                                              cld_aabb_edge_.edges.back().id_end == pairs_[pair_index_].second)) {
            std::cout << "Removing latest ICP result/edge." << std::endl;
            cld_aabb_edge_.edges.pop_back();
        } else {
            std::cout << "ICP did not run. Nothing to remove." << std::endl;
        }
    }

    void RunIcp() {
        if (cld_aabb_edge_.edges.empty() || (cld_aabb_edge_.edges.back().id_begin != pairs_[pair_index_].first ||
                                             cld_aabb_edge_.edges.back().id_end != pairs_[pair_index_].second)) {
            nie::io::PoseEdgeRecord edge{};
            edge.id_begin = pairs_[pair_index_].first;
            edge.id_end = pairs_[pair_index_].second;
            edge.category = nie::io::PoseEdgeRecord::Category::kLoop;
            cld_aabb_edge_.edges.push_back(edge);
        }

        // Feedback because it can take a bit.
        std::cout << "Running ICP." << std::endl;
        bool converged = cld_viewer_.RunIcp(&cld_aabb_edge_.edges.back().isometry);
        std::cout << "ICP converged: " << converged << " " << cld_aabb_edge_.edges.back().isometry << std::endl;
    }

    PairViewer<PointT>& cld_viewer() { return cld_viewer_; }
    nie::MapViewer& map_viewer() { return map_viewer_; }

private:
    void Update() {
        auto it_pose_1st = map_orbb_pose_id_to_index_map_.find(pairs_[pair_index_].first);
        auto it_pose_2nd = map_orbb_pose_id_to_index_map_.find(pairs_[pair_index_].second);

        CHECK(it_pose_1st != map_orbb_pose_id_to_index_map_.end());
        CHECK(it_pose_2nd != map_orbb_pose_id_to_index_map_.end());

        map_viewer_.Update({it_pose_1st->second, it_pose_2nd->second});
        cld_viewer_.Update(CreateLasPair(pairs_[pair_index_]));
    }

    nie::LasPair CreateLasPair(std::pair<nie::io::PoseId, nie::io::PoseId> pair) {
        auto it_pose_1st = cld_aabb_pose_map_.find(pair.first);
        auto it_pose_2nd = cld_aabb_pose_map_.find(pair.second);
        auto it_iref_1st = cld_aabb_iref_map_.find(pair.first);
        auto it_iref_2nd = cld_aabb_iref_map_.find(pair.second);

        CHECK(it_pose_1st != cld_aabb_pose_map_.end());
        CHECK(it_pose_2nd != cld_aabb_pose_map_.end());
        CHECK(it_iref_1st != cld_aabb_iref_map_.end());
        CHECK(it_iref_2nd != cld_aabb_iref_map_.end());

        nie::LasPair lp;
        lp.bbox_pose_a = it_pose_1st->second.get().isometry;
        lp.bbox_pose_b = it_pose_2nd->second.get().isometry;
        lp.timestamp_a = it_pose_1st->second.get().timestamp;
        lp.timestamp_b = it_pose_2nd->second.get().timestamp;
        lp.filename_a = it_iref_1st->second.get().path;
        lp.filename_b = it_iref_2nd->second.get().path;

        return lp;
    }

    std::vector<std::pair<nie::io::PoseId, nie::io::PoseId>> const& pairs_;
    std::unordered_map<nie::io::PoseId, std::reference_wrapper<nie::io::PoseRecord>> const& cld_aabb_pose_map_;
    std::unordered_map<nie::io::PoseId, std::reference_wrapper<nie::io::InfoRefRecord>> const& cld_aabb_iref_map_;
    std::unordered_map<nie::io::PoseId, std::size_t> const& map_orbb_pose_id_to_index_map_;
    PairViewer<PointT>& cld_viewer_;
    nie::MapViewer& map_viewer_;
    nie::io::PoseCollection& cld_aabb_edge_;
    std::size_t pair_index_;
};

template <typename PointT>
void AddKeyboardListener(Handler<PointT>* p_handler) {
    auto& handler = *p_handler;

    boost::function<void(const pcl::visualization::KeyboardEvent&)> keyboard_event =
            [&handler](const pcl::visualization::KeyboardEvent& event) -> void {
        constexpr double kDelta = 0.5;
        // Don't want to do it twice (on up as well).
        if (event.keyDown()) {
            if (event.getKeySym() == "Insert") {
                handler.cld_viewer().OriginMut().x() += kDelta;
                handler.cld_viewer().Update();
            } else if (event.getKeySym() == "Delete") {
                handler.cld_viewer().OriginMut().x() -= kDelta;
                handler.cld_viewer().Update();
            } else if (event.getKeySym() == "Home") {
                handler.cld_viewer().OriginMut().y() += kDelta;
                handler.cld_viewer().Update();
            } else if (event.getKeySym() == "End") {
                handler.cld_viewer().OriginMut().y() -= kDelta;
                handler.cld_viewer().Update();
            } else if (event.getKeySym() == "Prior") {
                handler.cld_viewer().OriginMut().z() += kDelta;
                handler.cld_viewer().Update();
            } else if (event.getKeySym() == "Next") {
                handler.cld_viewer().OriginMut().z() -= kDelta;
                handler.cld_viewer().Update();
            } else if (event.getKeySym() == "i") {
                handler.RunIcp();
            } else if (event.getKeySym() == "Return") {
                handler.IncPairIndex();
            } else if (event.getKeySym() == "backslash") {
                handler.RemoveActiveEdge();
            } else if (event.getKeySym() == "h") {
                std::cout << std::endl;
                std::cout << "MANUAL LOOP CLOSURE OPTIONS: " << std::endl;
                std::cout << "Insert / Delete           = +/- X" << std::endl;
                std::cout << "Home / End                = +/- Y" << std::endl;
                std::cout << "Page-up / Page-down       = +/- Z" << std::endl;
                std::cout << std::endl;
                std::cout << "i                         = Run ICP on current pair." << std::endl;
                std::cout << "Backslash                 = Remove ICP result current pair." << std::endl;
                std::cout << "Enter                     = Load next loop pair." << std::endl;
                std::cout << std::endl;
            }
        }
    };
    handler.cld_viewer().viewer().registerKeyboardCallback(keyboard_event);
}

void SelectionCallback(
        nie::PoseBbox const& selection,
        nie::io::PoseCollection const& bbox_pose,
        std::vector<nie::PoseBbox> const& bounds,
        nie::io::InfoRefCollection const& bbox_iref) {
    std::vector<std::size_t> indices;
    for (std::size_t i = 0; i < bounds.size(); ++i) {
        double area;
        double iou;
        selection.Intersection2D(bounds[i], &area, &iou);

        if (area > 0.) {
            indices.push_back(i);
        }
    }

    if (!indices.empty()) {
        std::cout << "The following las files are selected:" << std::endl;
        std::cout << std::setw(8) << std::left << "id"
                  << "filename" << std::endl;
        for (std::size_t i : indices) {
            std::cout << std::setw(8) << std::left << bbox_pose.poses[i].id << bbox_iref.info_refs[i].path << std::endl;
        }
    } else {
        std::cout << "No las files selected." << std::endl;
    }
}

nie::Isometry3qd ReadExtrinsics() {
    if (!FLAGS_in_file_lidar_extrinsics.empty()) {
        LOG(INFO) << "Reading external calibration file: " << FLAGS_in_file_lidar_extrinsics;
        if (FLAGS_lidar_id.empty()) {
            return nie::ReadLidarExtrinsics(FLAGS_in_file_lidar_extrinsics);

        } else if (nie::ToLower(FLAGS_lidar_id) == "kitti") {
            return nie::kIsometryFromTo<nie::Frame::kVehicle, nie::Frame::kAircraft, Eigen::Quaterniond> *
                   nie::io::kitti::ReadExtrinsics(FLAGS_in_file_lidar_extrinsics);
        } else {
            return nie::io::ReadLidarParametersByIdentifier(FLAGS_in_file_lidar_extrinsics, FLAGS_lidar_id).extrinsics;
        }
    } else {
        LOG(INFO) << "No external calibration file specified. Using identity.";
        return nie::Isometry3qd::Identity();
    }
}

std::vector<std::pair<nie::io::PoseId, nie::io::PoseId>> ReadMatchPairs() {
    LOG(INFO) << "Reading match pairs: " << FLAGS_in_file_csv_loops;
    std::vector<std::pair<nie::io::PoseId, nie::io::PoseId>> pairs = nie::ReadLines(
            FLAGS_in_file_csv_loops, [](std::string const& s) -> std::pair<nie::io::PoseId, nie::io::PoseId> {
                std::vector<nie::io::PoseId> split = nie::Split<nie::io::PoseId>(s);
                CHECK(split.size() == 2);
                return {split[0], split[1]};
            });
    return pairs;
}

std::unordered_map<nie::io::PoseId, std::size_t> CreateIdToIndexMap(std::vector<nie::io::PoseRecord> const& poses) {
    std::unordered_map<nie::io::PoseId, std::size_t> map;
    for (std::size_t i = 0; i < poses.size(); ++i) {
        map[poses[i].id] = i;
    }
    return map;
}

void RunViewer() {
    // Calibration: vehicle to aircraft, lidar to gnss antena.
    nie::Isometry3qd T_gl = ReadExtrinsics();
    // Which las files to try and match.
    std::vector<std::pair<nie::io::PoseId, nie::io::PoseId>> pairs = ReadMatchPairs();

    if (pairs.empty()) {
        std::cout << "No pairs to align." << std::endl;
        return;
    }

    LOG(INFO) << "Reading iref file: " << FLAGS_in_file_bbox_iref;
    auto bbox_iref = nie::io::ReadCollection<nie::io::InfoRefCollection>(FLAGS_in_file_bbox_iref);

    // ************************************************************************************
    // MAP RELATED FILES
    // ************************************************************************************

    LOG(INFO) << "Reading pose file with orbb poses: " << FLAGS_in_file_orbb_pose;
    auto map_orbb_pose = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_orbb_pose);
    for (auto& pose : map_orbb_pose.poses) {
        pose.isometry = T_gl * pose.isometry;
    }

    LOG(INFO) << "Reading bbox file with orbb boxes: " << FLAGS_in_file_orbb_bbox;
    auto map_orbb_bbox = nie::io::ReadCollection<nie::io::BboxCollection>(FLAGS_in_file_orbb_bbox);

    std::unordered_map<nie::io::PoseId, std::size_t> pose_id_to_bounds_index_map;
    std::vector<nie::PoseBbox> map_orbb_bounds;
    CreatePoseBboxMap(map_orbb_pose.poses, map_orbb_bbox.boxes, &pose_id_to_bounds_index_map, &map_orbb_bounds);

    LOG(INFO) << "Reading pose file with the trace: " << FLAGS_in_file_trace_pose;
    auto map_trace_pose = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_trace_pose);
    for (auto& pose : map_trace_pose.poses) {
        pose.isometry = T_gl * pose.isometry;
    }

    auto map_orbb_pose_id_to_index_map = CreateIdToIndexMap(map_orbb_pose.poses);

    // ************************************************************************************
    // CLD RELATED FILE
    // ************************************************************************************

    LOG(INFO) << "Reading pose file with aabb poses: " << FLAGS_in_file_aabb_pose;
    auto cld_aabb_pose = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_aabb_pose);
    for (auto& pose : cld_aabb_pose.poses) {
        pose.isometry = T_gl * pose.isometry * T_gl.Inversed();
    }

    auto cld_aabb_pose_map = nie::io::CreateRecordMap(cld_aabb_pose.poses.begin(), cld_aabb_pose.poses.end());
    // Even if it was the orbb iref file, it doesn't matter. Same contents.
    auto cld_aabb_iref_map = nie::io::CreateRecordMap(bbox_iref.info_refs.begin(), bbox_iref.info_refs.end());

    // ************************************************************************************
    // VIEWERS
    // ************************************************************************************

    nie::MapViewer map_viewer("cloud_map", map_orbb_bounds, map_trace_pose);
    map_viewer.View([&map_orbb_pose, &map_orbb_bounds, &bbox_iref](nie::PoseBbox const& selection) {
        SelectionCallback(selection, map_orbb_pose, map_orbb_bounds, bbox_iref);
    });

    nie::io::PoseCollection cld_aabb_edge;
    cld_aabb_edge.header = cld_aabb_pose.header;

    PairViewer<pcl::PointXYZI> cld_viewer(
            "manual_loop_closer", LasLoader<pcl::PointXYZI>(T_gl, FLAGS_filter_grid_size));
    Handler<pcl::PointXYZI> handler(
            pairs,
            cld_aabb_pose_map,
            cld_aabb_iref_map,
            map_orbb_pose_id_to_index_map,
            &cld_viewer,
            &map_viewer,
            &cld_aabb_edge);
    // So at least the first las pair should be loaded.
    AddKeyboardListener(&handler);
    cld_viewer.View();

    // ************************************************************************************
    // DONE, viewer stopped.
    // ************************************************************************************

    LOG(INFO) << "Writing edge file with aabb edges: " << FLAGS_out_file_aabb_edge;
    for (auto& edge : cld_aabb_edge.edges) {
        // The edge was based on calibration corrected poses. We undo the calibration correction.
        // Edge equals T_ab, meaning T_a_inv * T_b. Both have been corrected like T_a = T_gl * T_a * T_gl_inv.
        // So total edge:
        //      = (T_gl * T_a * T_gl_inv)_inv * (T_gl * T_b * T_gl_inv)
        //      = (T_gl * T_a_inv * T_gl_inv) * (T_gl * T_b * T_gl_inv)
        //      = T_gl * T_a_inv * T_b * T_gl_inv
        // It should be:
        //      = T_a_inv * T_b
        // So below we do the following:
        //      = T_gl_inv * T_gl * T_a_inv * T_b * T_gl_inv * T_gl
        edge.isometry = T_gl.Inversed() * edge.isometry * T_gl;
    }

    nie::io::Write(cld_aabb_edge, FLAGS_out_file_aabb_edge);
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, false);

    RunViewer();

    return 0;
}
