/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <pcl/io/ply_io.h>
#include <nie/core/geometry/rotation.hpp>
#include <nie/core/hash.hpp>
#include <nie/core/mt_stream.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>
#include <nie/lidar/geometry/helper_area.hpp>
#include <nie/lidar/point_density.hpp>
#include <nie/lidar/shared_constants.hpp>
#include <nie/lidar/trace_cloud.hpp>

#include "debug_statistics_writer.hpp"
#include "helper_transformation.hpp"

namespace nie {

class DebugIo {
private:
    using PoseIdPair = std::pair<nie::io::PoseId, nie::io::PoseId>;

public:
    DebugIo(std::string debug_folder, bool debug)
        : debug_(debug),
          debug_folder_(std::move(debug_folder)),
          debug_statistics_(std::ofstream(debug_ ? (debug_folder_ + "/" + kStatisticsFileName).c_str() : nullptr)),
          stats_writers_() {
        if (debug_) {
            Prepare();
        }
    }

    template <typename PointT>
    void SetClouds(
            PoseIdPair const& ids,
            Cloud<PointT> const& cloud_a,
            Cloud<PointT> const& cloud_b,
            std::vector<nie::io::PoseRecord> const& trace) {
        if (debug_) {
            auto& writer = GetStatsWriter(ids);
            // Store the statistics
            writer.Set<nie::DebugStatisticsWriter::kIdA>(ids.first);
            writer.Set<nie::DebugStatisticsWriter::kIdB>(ids.second);
            writer.Set<nie::DebugStatisticsWriter::kPointCountA>(cloud_a.point_cloud().size());
            writer.Set<nie::DebugStatisticsWriter::kPointCountB>(cloud_b.point_cloud().size());
            writer.Set<nie::DebugStatisticsWriter::kLengthTraceA>(nie::TraceLength(trace, cloud_a));
            writer.Set<nie::DebugStatisticsWriter::kLengthTraceB>(nie::TraceLength(trace, cloud_b));
        }
    }

    void SetOrientedBoundingBoxes(PoseIdPair const& ids, std::vector<nie::PoseBbox> const& bounds) {
        if (debug_) {
            CHECK(bounds.size() == 2);

            // Calculate properties
            double area_intersection, area_iou;
            bounds.front().Intersection2D(bounds.back(), &area_intersection, &area_iou);
            auto& writer = GetStatsWriter(ids);
            writer.Set<nie::DebugStatisticsWriter::kAreaIntersection>(area_intersection);
            writer.Set<nie::DebugStatisticsWriter::kAreaIntersectionRatio>(area_iou);
        }
    }

    template <typename PointT>
    void SetFilteredClouds(PoseIdPair const& ids, Cloud<PointT> const& cloud_a, Cloud<PointT> const& cloud_b) {
        if (debug_) {
            // Store the filtered clouds as ply files
            pcl::PLYWriter ply_writer;
            ply_writer.write(GetFilteredCloudFilename(ids.first, ids.second, ids.first), cloud_a.point_cloud(), true);
            ply_writer.write(GetFilteredCloudFilename(ids.first, ids.second, ids.second), cloud_b.point_cloud(), true);

            auto& writer = GetStatsWriter(ids);
            // Store the filtered clouds statistics
            writer.Set<nie::DebugStatisticsWriter::kPointCountFilteredA>(cloud_a.point_cloud().size());
            writer.Set<nie::DebugStatisticsWriter::kPointCountFilteredB>(cloud_b.point_cloud().size());

            // Calculate the "point neighborhood density" and store the statistics
            std::vector<PoseIdPair> neighbors;
            nie::PointDensity(cloud_a, cloud_b, &neighbors);
            writer.Set<nie::DebugStatisticsWriter::kPointDensity>(neighbors.size());
        }
    }

    void SetRegistrationError(PoseIdPair const& ids, float const& fitness_score, nie::Isometry3qd const& T) {
        if (debug_) {
            auto& writer = GetStatsWriter(ids);
            writer.Set<nie::DebugStatisticsWriter::kFitnessScore>(fitness_score);
            writer.Set<nie::DebugStatisticsWriter::kTranslationSize>(nie::CalcTranslationNorm(T));
            writer.Set<nie::DebugStatisticsWriter::kRotationSize>(nie::Rad2Deg(nie::CalcRotationAngle(T)));
        }
    }

    void SetCorrespondences(PoseIdPair const& ids, pcl::CorrespondencesConstPtr const& correspondences) {
        if (debug_) {
            // Write the correspondences to file
            std::ofstream ofs(GetDebugFilenamePrefix(ids.first, ids.second) + kCorrespondencesFileName);
            ofs << "point_id_a,point_id_b,distance\n";
            std::for_each(correspondences->begin(), correspondences->end(), [&ofs](pcl::Correspondence const& c) {
                // match = a, query = b
                ofs << c.index_match << "," << c.index_query << "," << c.distance << "\n";
            });
            ofs.close();

            // Add the statistics of the correspondences
            GetStatsWriter(ids).Set<nie::DebugStatisticsWriter::kCorrespondences>(correspondences->size());
        }
    }

    void SetDone(PoseIdPair const& ids) { stats_writers_.erase(ids); }

private:
    void Prepare() {
        debug_statistics_
                << "id_a,id_b,count_a,count_b,count_a_filtered,count_b_filtered,area_int,area_int_ratio,length_trace_a,"
                   "length_trace_b,point_density,correspondences,fitness_score,translation_size,rotation_size\n";

        if (not boost::filesystem::exists(debug_folder_ + "/" + kFilteredCloudFolderName)) {
            CHECK(boost::filesystem::create_directory(debug_folder_ + "/" + kFilteredCloudFolderName))
                    << "Could not create a new folder in the debug directory.";
        }
    }

    nie::DebugStatisticsWriter& GetStatsWriter(PoseIdPair const& ids) {
        auto iter = stats_writers_.find(ids);
        if (iter == stats_writers_.end()) {
            auto result = stats_writers_.emplace(ids, &debug_statistics_);
            iter = result.first;
        }
        return iter->second;
    }

    std::string GetDebugFilenamePrefix(nie::io::PoseId id_a, nie::io::PoseId id_b) {
        return debug_folder_ + "/" + kFilteredCloudFolderName + "/" + std::to_string(id_a) + "-" +
               std::to_string(id_b) + "-";
    }
    std::string GetFilteredCloudFilename(nie::io::PoseId id_a, nie::io::PoseId id_b, nie::io::PoseId id_selected) {
        return GetDebugFilenamePrefix(id_a, id_b) + std::to_string(id_selected) + "." + kFilteredCloudFileExt;
    }

    bool const debug_;
    std::string const debug_folder_;
    nie::mt::MtStream<std::ofstream> debug_statistics_;

    std::unordered_map<PoseIdPair, nie::DebugStatisticsWriter, nie::PairHash> stats_writers_;
};

}  // namespace nie
