/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "solve.hpp"

#include <memory>

#include "covariance.hpp"
#include "filter.hpp"
#include "problem.hpp"

namespace detail {

ceres::Solver::Summary Solve(
        nie::io::RectifiedCameraParameters const& camera_parameters,
        nie::io::PoseCollection const& pose_constraints,
        nie::io::KeypointCollection const& kpnt_constraints,
        nie::io::ObjectCollection const& objt_constraints,
        nie::io::PoseCollection* p_pose_estimates,
        nie::io::ObjectCollection* p_objt_estimates,
        ceres::Problem* problem) {
    BuildOptimizationProblem(
            camera_parameters,
            pose_constraints,
            kpnt_constraints,
            objt_constraints,
            problem,
            p_pose_estimates,
            p_objt_estimates);

    LOG(INFO) << "Number of poses: " << p_pose_estimates->poses.size();
    LOG(INFO) << "Number of edges: " << pose_constraints.edges.size();

    return SolveOptimizationProblem(problem);
}

}  // namespace detail

bool Solve(
        nie::io::RectifiedCameraParameters const& camera_parameters,
        nie::io::KeypointCollection const& kpnt_constraints,
        nie::io::ObjectCollection const& objt_constraints,
        bool output_information_matrices,
        bool double_pass,
        double mahalanobis_threshold,
        double inlier_percentage,
        bool output_mahalanobis_distances,
        Filenamer filenamer,
        nie::io::PoseCollection* p_pose_constraints,
        nie::io::PoseCollection* p_pose_estimates,
        nie::io::ObjectCollection* p_objt_estimates) {
    // Create problem pointer, to easily reset the problem.
    std::unique_ptr<ceres::Problem> problem = std::make_unique<ceres::Problem>();

    if (output_mahalanobis_distances) {
        WriteMahalanobisDistances(p_pose_estimates->poses, p_pose_constraints->edges, filenamer("initial"));
    }

    auto const orig_pose_estimates_poses = p_pose_estimates->poses;
    auto const orig_objt_estimates_objects = p_objt_estimates->objects;

    // First solve the complete problem
    ceres::Solver::Summary summary = detail::Solve(
            camera_parameters,
            *p_pose_constraints,
            kpnt_constraints,
            objt_constraints,
            p_pose_estimates,
            p_objt_estimates,
            problem.get());
    bool solution_usable = summary.IsSolutionUsable();

    if (output_mahalanobis_distances) {
        WriteMahalanobisDistances(p_pose_estimates->poses, p_pose_constraints->edges, filenamer("first_pass"));
    }

    // In case of double pass, based on the results of the first pass, filter the following properties and create and
    // solve the problem again:
    //   * objects and corresponding keypoints (not implemented yet)
    //   * edges
    if (double_pass) {
        if (solution_usable) {
            LOG(INFO) << "Running double pass.";
            std::size_t const original_pose_constraints_edges_size = p_pose_constraints->edges.size();

            double threshold = mahalanobis_threshold;
            if (threshold == -1.) {
                threshold = DetermineMahalanobisDistance(
                        p_pose_estimates->poses, p_pose_constraints->edges, inlier_percentage);
                VLOG(1) << "Mahalanobis distance threshold determined to be " << threshold << " for inlier percentage "
                        << inlier_percentage << ".";
            }
            Filter(threshold, p_pose_estimates->poses, &p_pose_constraints->edges);
            CHECK(!p_pose_constraints->edges.empty())
                    << "No pose edges left after filtering with Mahalanobis threshold " << threshold << ".";

            // If problem changed, solve again.
            if (original_pose_constraints_edges_size != p_pose_constraints->edges.size()) {
                // Rebuild and solve the problem
                problem = std::make_unique<ceres::Problem>();

                p_pose_estimates->poses = orig_pose_estimates_poses;
                p_objt_estimates->objects = orig_objt_estimates_objects;

                summary = detail::Solve(
                        camera_parameters,
                        *p_pose_constraints,
                        kpnt_constraints,
                        objt_constraints,
                        p_pose_estimates,
                        p_objt_estimates,
                        problem.get());
                solution_usable = summary.IsSolutionUsable();

                if (output_mahalanobis_distances) {
                    WriteMahalanobisDistances(
                            p_pose_estimates->poses,
                            p_pose_constraints->edges,
                            filenamer("second_pass_" + std::to_string(inlier_percentage)));
                }
            } else {
                LOG(INFO) << "No edges filtered, no second run required.";
            }
        } else {
            LOG(ERROR) << "Graph optimizer solution error, skipping double pass.";
        }
    }

    if (solution_usable) {
        if (output_information_matrices) {
            LOG(INFO) << "Graph optimizer solution OK, computing information matrices...";
            if (!GetInformationMatrices(p_pose_estimates, p_objt_estimates, problem.get())) {
                LOG(ERROR) << "Unable to compute output information matrices.";
                solution_usable = false;
            }

            p_pose_estimates->header.flags &= ~nie::io::PoseHeader::kHasPoseInformation;
            p_pose_estimates->header.flags &= ~nie::io::PoseHeader::kHasEdgeInformation;
            p_pose_estimates->header.flags |= nie::io::PoseHeader::kHasPoseInformationPerRecord;
            p_pose_estimates->header.flags |= nie::io::PoseHeader::kHasEdgeInformationPerRecord;

            p_objt_estimates->header.flags = nie::io::ObjectHeader::kHasInformationPerRecord;
        }
    } else {
        LOG(ERROR) << "Graph optimizer solution error.";
    }
    return solution_usable;
}

bool Solve(
        bool output_information_matrices,
        bool double_pass,
        double mahalanobis_threshold,
        double inlier_percentage,
        bool output_mahalanobis_distances,
        Filenamer mahalanobis_distances_filenamer,
        nie::io::PoseCollection* p_pose_constraints,
        nie::io::PoseCollection* p_pose_estimates) {
    nie::io::RectifiedCameraParameters const camera_parameters{};
    nie::io::KeypointCollection const kpnt_constraints{};
    nie::io::ObjectCollection const objt_constraints{};
    nie::io::ObjectCollection objt_estimates{};

    return Solve(
            camera_parameters,
            kpnt_constraints,
            objt_constraints,
            output_information_matrices,
            double_pass,
            mahalanobis_threshold,
            inlier_percentage,
            output_mahalanobis_distances,
            std::move(mahalanobis_distances_filenamer),
            p_pose_constraints,
            p_pose_estimates,
            &objt_estimates);
}

bool Solve(
        bool output_information_matrices,
        bool double_pass,
        double mahalanobis_threshold,
        nie::io::PoseCollection* p_pose_constraints,
        nie::io::PoseCollection* p_pose_estimates) {
    nie::io::RectifiedCameraParameters const camera_parameters{};
    nie::io::KeypointCollection const kpnt_constraints{};
    nie::io::ObjectCollection const objt_constraints{};
    nie::io::ObjectCollection objt_estimates{};

    double const inlier_percentage{};
    bool const output_mahalanobis_distances = false;
    Filenamer const mahalanobis_distances_filenamer{};

    return Solve(
            camera_parameters,
            kpnt_constraints,
            objt_constraints,
            output_information_matrices,
            double_pass,
            mahalanobis_threshold,
            inlier_percentage,
            output_mahalanobis_distances,
            mahalanobis_distances_filenamer,
            p_pose_constraints,
            p_pose_estimates,
            &objt_estimates);
}

bool Solve(
        bool output_information_matrices,
        nie::io::PoseCollection* p_pose_constraints,
        nie::io::PoseCollection* p_pose_estimates) {
    bool const double_pass = false;
    double const mahalanobis_threshold{};
    return Solve(output_information_matrices, double_pass, mahalanobis_threshold, p_pose_constraints, p_pose_estimates);
}
