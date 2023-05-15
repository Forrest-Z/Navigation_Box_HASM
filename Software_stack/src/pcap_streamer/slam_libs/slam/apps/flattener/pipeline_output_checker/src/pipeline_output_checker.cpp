/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <map>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <nie/core/constants.hpp>
#include <nie/core/filesystem.hpp>
#include <nie/core/gflags.hpp>
#include <nie/formats/ba_graph.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>

DEFINE_string(in_file_pose_trace, "", "Input .pose file that contains the SLAM pipeline output trace.");
DEFINE_string(in_file_iref_trace, "", "Input .pose file that contains the SLAM pipeline output trace.");
DEFINE_string(in_file_pose_gps, "", "Input .pose file that contains the projected GPS coordinates.");

DEFINE_string(
        out_file_csv_delta,
        "",
        "Output .csv file that contains the deltas between pipeline output poses and the gps trace.");

DEFINE_validator(in_file_pose_trace, nie::ValidateIsFile);
DEFINE_validator(in_file_iref_trace, nie::ValidateIsFile);
DEFINE_validator(in_file_pose_gps, nie::ValidateIsFile);
DEFINE_validator(out_file_csv_delta, nie::ValidateParentDirExists);

constexpr double kTresholdRotation = 2.0;
constexpr double kTresholdTranslation = 5.0;

void Check(nie::io::PoseCollection const& collection, nie::Timestamp_ns const& timestamp) {
    CHECK(timestamp >= collection.poses.begin()->timestamp && timestamp <= collection.poses.rbegin()->timestamp)
            << "The timestamp of the first relative pose is not in timestamp range of the absolute poses.";
}

struct ErrorRecord {
    nie::io::PoseId id;
    std::string las;
    double delta_R;
    double delta_t;
};

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    auto pose_collection_pose_trace = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_pose_trace);
    auto pose_collection_iref_trace = nie::io::ReadCollection<nie::io::InfoRefCollection>(FLAGS_in_file_iref_trace);
    auto pose_collection_pose_gps = nie::io::ReadCollection<nie::io::PoseCollection>(FLAGS_in_file_pose_gps);

    auto iref_map = nie::io::CreateRecordMap(
            pose_collection_iref_trace.info_refs.begin(), pose_collection_iref_trace.info_refs.end());
    auto its_pose_gps = std::make_pair(pose_collection_pose_gps.poses.begin(), pose_collection_pose_gps.poses.end());

    // std::map<std::string, std::vector<std::pair<double, double>>> errors_R;
    // std::map<std::string, std::vector<std::pair<double, double>>> errors_t;
    std::vector<ErrorRecord> errors;

    for (auto& p : pose_collection_pose_trace.poses) {
        Check(pose_collection_pose_gps, p.timestamp);

        nie::Isometry3qd T_world_gps;
        its_pose_gps = nie::io::InterpolateIsometry(
                its_pose_gps.first, pose_collection_pose_gps.poses.end(), p.timestamp, &T_world_gps);

        auto delta = p.isometry.Delta(T_world_gps);
        double delta_R = Eigen::AngleAxisd(delta.rotation()).angle() * nie::kRad2Deg<double>;
        double delta_t = delta.translation().norm();

        auto sr = iref_map.find(p.id);
        if (sr == iref_map.end()) {
            continue;
        }

        errors.push_back({p.id, sr->second.get().path, delta_R, delta_t});

        //        if(delta_R > kTresholdRotation) {
        //            auto sr = iref_map.find(p.id);
        //
        //            if (sr != iref_map.end()) {
        //                errors_R[sr->second.get().path].emplace_back(delta_R, delta_t);
        //
        ////                std::cout << "delta_t: " << delta_t << std::endl;
        ////                std::cout << "delta_R: " << delta_R << std::endl;
        //            }
        //        }
        //
        //        if(delta_t > kTresholdTranslation) {
        //            auto sr = iref_map.find(p.id);
        //
        //            if (sr != iref_map.end()) {
        //                errors_t[sr->second.get().path].emplace_back(delta_R, delta_t);
        //
        ////                std::cout << "delta_t: " << delta_t << std::endl;
        ////                std::cout << "delta_R: " << delta_R << std::endl;
        //            }
        //        }
    }

    // std::cout << "las count R: " << errors_R.size() << std::endl;
    // std::cout << "las count t: " << errors_t.size() << std::endl;

    auto transform = [](ErrorRecord const& r) -> std::string {
        return std::to_string(r.id) + "," + r.las + "," + std::to_string(r.delta_R) + "," + std::to_string(r.delta_t);
    };

    nie::WriteLines(FLAGS_out_file_csv_delta, errors, "id,las,ddegrees,dmeters", transform);

    return 0;
}