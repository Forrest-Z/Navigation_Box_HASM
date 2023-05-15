/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "filter.hpp"

#include <nie/core/algorithm.hpp>
#include <nie/cv/ceres/edge_se3_cost.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>

#include <algorithm>
#include <map>
#include <memory>
#include <numeric>
#include <vector>

namespace detail {

std::vector<std::size_t> GetEdgeIndices(std::vector<nie::io::PoseEdgeRecord> const& edges) {
    std::vector<std::size_t> indices = GetEdgesIndicesByCategory(edges, nie::io::PoseEdgeRecord::Category::kLoop);
    // In the absence of loop edges, use all edges
    if (indices.empty()) {
        indices.resize(edges.size());
        std::iota(indices.begin(), indices.end(), 0);
    }
    return indices;
}

double MahalanobisDistance(
        nie::Isometry3qd const& pose_begin,
        nie::Isometry3qd const& pose_end,
        nie::Isometry3qd const& constraint,
        Eigen::Matrix<double, 6, 6> const& information) {
    Eigen::Matrix<double, 6, 6> const sqrt_information = information.llt().matrixL();

    std::unique_ptr<ceres::CostFunction> cost_function =
            std::unique_ptr<ceres::CostFunction>(nie::EdgeSe3Cost::Create(constraint, sqrt_information));

    std::vector<double const*> parameters(4);
    parameters[0] = pose_begin.translation().data();
    parameters[1] = pose_begin.rotation().coeffs().data();
    parameters[2] = pose_end.translation().data();
    parameters[3] = pose_end.rotation().coeffs().data();
    Eigen::Matrix<double, 6, 1> residuals;
    cost_function->Evaluate(parameters.data(), residuals.data(), nullptr);

    // Normally it would be this, however, the cost function already applies sqrt(information) to the residuals.
    // std::sqrt(residuals.dot(constraint.information * residuals));
    return std::sqrt(residuals.dot(residuals));
}

}  // namespace detail

// Filter constraints with high residuals
void Filter(
        double const& mahalanobis_threshold,
        std::vector<nie::io::PoseRecord> const& poses,
        std::vector<nie::io::PoseEdgeRecord>* p_edges) {
    assert(p_edges != nullptr);
    auto& edges = *p_edges;

    // The number of edges for every pose
    std::map<int, std::size_t> constraints_by_ids;

    // Count the number of edges for every pose
    for (auto const& edge : edges) {
        ++constraints_by_ids[edge.id_begin];
        ++constraints_by_ids[edge.id_end];
    }

    // Make copy of loop edges to be able to filter
    std::vector<std::size_t> indices_edges_to_remove = detail::GetEdgeIndices(edges);

    // Remove indices of edges that should be kept
    auto const& pose_map = nie::io::CreateRecordMap(poses.cbegin(), poses.cend());
    indices_edges_to_remove.erase(
            std::remove_if(
                    indices_edges_to_remove.begin(),
                    indices_edges_to_remove.end(),
                    [&mahalanobis_threshold, &edges, &pose_map](std::size_t const& i) -> bool {
                        auto const& edge = edges[i];
                        nie::io::PoseRecord const& pose_begin = pose_map.at(edge.id_begin);
                        nie::io::PoseRecord const& pose_end = pose_map.at(edge.id_end);

                        // Edge is not to filter, so remove entry.
                        return detail::MahalanobisDistance(
                                       pose_begin.isometry, pose_end.isometry, edge.isometry, edge.information) <
                               mahalanobis_threshold;
                    }),
            indices_edges_to_remove.end());

    for (std::size_t const& i : indices_edges_to_remove) {
        auto const& edge = edges[i];
        constraints_by_ids[edge.id_begin]--;
        constraints_by_ids[edge.id_end]--;
    }

    // Need to log before it happens, otherwise they are removed (done for the constraints).
    if (VLOG_IS_ON(6)) {
        {
            std::stringstream ss;
            ss << "Removing edges:";
            for (std::size_t const& i : indices_edges_to_remove) {
                ss << " " << edges[i].id_begin << "," << edges[i].id_end;
            }
            VLOG(6) << ss.str();
            VLOG(6) << "Edge count before: " << edges.size() << " and after "
                    << (edges.size() - indices_edges_to_remove.size());
        }
    }

    // Report poses without edges
    for (auto const& count_by_id : constraints_by_ids) {
        if (count_by_id.second == 0) {
            LOG(WARNING) << "Pose with id " << count_by_id.first << " does not have any constraints after filtering.";
        }
    }

    // Remove edges with high residuals
    std::sort(indices_edges_to_remove.begin(), indices_edges_to_remove.end());
    edges.erase(nie::RemoveIf(indices_edges_to_remove.begin(), indices_edges_to_remove.end(), &edges), edges.end());
}

double DetermineMahalanobisDistance(
        std::vector<nie::io::PoseRecord> const& poses,
        std::vector<nie::io::PoseEdgeRecord> const& edges,
        double const& inlier_percentage) {
    // Find all edges of interest
    std::vector<std::size_t> edge_indices = detail::GetEdgeIndices(edges);

    auto const& pose_map = nie::io::CreateRecordMap(poses.cbegin(), poses.cend());

    // Write the Mahalanobis distances of all edges of interest to a csv file
    std::vector<double> distances;
    distances.reserve(edge_indices.size());
    for (std::size_t const& i : edge_indices) {
        nie::io::PoseEdgeRecord const& edge = edges[i];
        nie::Isometry3qd const& pose_begin = pose_map.at(edge.id_begin).get().isometry;
        nie::Isometry3qd const& pose_end = pose_map.at(edge.id_end).get().isometry;
        distances.emplace_back(detail::MahalanobisDistance(pose_begin, pose_end, edge.isometry, edge.information));
    }

    std::sort(distances.begin(), distances.end());
    double const distance = distances[distances.size() * inlier_percentage / 100.];
    return distance;
}

void WriteMahalanobisDistances(
        std::vector<nie::io::PoseRecord> const& poses,
        std::vector<nie::io::PoseEdgeRecord> const& edges,
        std::string const& output_filename) {
    // Find all edges of interest
    std::vector<std::size_t> edge_indices = detail::GetEdgeIndices(edges);

    auto const& pose_map = nie::io::CreateRecordMap(poses.cbegin(), poses.cend());

    // Write the Mahalanobis distances of all edges of interest to a csv file
    char const delimiter = ';';
    std::string const header = std::string("id_begin") + delimiter + "id_end" + delimiter + "mahalanobis_distance";
    nie::WriteLines(output_filename, edge_indices, header, [&pose_map, &edges](std::size_t const& i) -> std::string {
        nie::io::PoseEdgeRecord const& edge = edges[i];
        nie::Isometry3qd const& pose_begin = pose_map.at(edge.id_begin).get().isometry;
        nie::Isometry3qd const& pose_end = pose_map.at(edge.id_end).get().isometry;
        double distance = detail::MahalanobisDistance(pose_begin, pose_end, edge.isometry, edge.information);

        std::stringstream ss;
        ss << edge.id_begin << delimiter << edge.id_end << delimiter << distance;
        return ss.str();
    });

    LOG(INFO) << "Mahalanobis distances written to csv file '" << output_filename << "'.";
}
