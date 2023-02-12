/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <nie/formats/ba_graph.hpp>
#include <nie/formats/calib3d/rectified_parameters.hpp>

using Filenamer = std::function<std::string(std::string const&)>;
bool Solve(
        nie::io::RectifiedCameraParameters const& camera_parameters,
        nie::io::KeypointCollection const& kpnt_constraints,
        nie::io::ObjectCollection const& objt_constraints,
        bool output_information_matrices,
        bool double_pass,
        double mahalanobis_threshold,
        double inlier_percentage,
        bool output_mahalanobis_distances,
        Filenamer mahalanobis_distances_filenamer,
        nie::io::PoseCollection* p_pose_constraints,
        nie::io::PoseCollection* p_pose_estimates,
        nie::io::ObjectCollection* p_objt_estimates);

bool Solve(
        bool output_information_matrices,
        bool double_pass,
        double mahalanobis_threshold,
        double inlier_percentage,
        bool output_mahalanobis_distances,
        Filenamer mahalanobis_distances_filenamer,
        nie::io::PoseCollection* p_pose_constraints,
        nie::io::PoseCollection* p_pose_estimates);

bool Solve(
        bool output_information_matrices,
        bool double_pass,
        double mahalanobis_threshold,
        nie::io::PoseCollection* p_pose_constraints,
        nie::io::PoseCollection* p_pose_estimates);

bool Solve(
        bool output_information_matrices,
        nie::io::PoseCollection* p_pose_constraints,
        nie::io::PoseCollection* p_pose_estimates);
