/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <thread>

#include <ceres/ceres.h>
#include <nie/formats/ba_graph.hpp>
#include <nie/formats/calib3d/rectified_parameters.hpp>

void BuildOptimizationProblem(
        nie::io::RectifiedCameraParameters const& parameters,
        nie::io::PoseCollection const& pose_constraints,
        nie::io::KeypointCollection const& kpnt_constraints,
        nie::io::ObjectCollection const& objt_constraints,
        ceres::Problem* problem,
        nie::io::PoseCollection* p_pose_estimates,
        nie::io::ObjectCollection* p_objt_estimates);

// Returns true if the solve was successful.
ceres::Solver::Summary SolveOptimizationProblem(
        ceres::Problem* problem, std::uint32_t const& num_threads = std::thread::hardware_concurrency());
