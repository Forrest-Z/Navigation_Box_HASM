/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <thread>

#include <ceres/ceres.h>
#include <nie/formats/ba_graph.hpp>

/// @brief Updates the information matrices for all poses and objects.
/// @details If an isometry was fixed inside the optimization, all elements
/// of the information matrix will have value nan
bool GetInformationMatrices(
        nie::io::PoseCollection* p_pose_estimates,
        nie::io::ObjectCollection* p_objt_estimates,
        ceres::Problem* problem,
        std::uint32_t const& num_threads = std::thread::hardware_concurrency());
