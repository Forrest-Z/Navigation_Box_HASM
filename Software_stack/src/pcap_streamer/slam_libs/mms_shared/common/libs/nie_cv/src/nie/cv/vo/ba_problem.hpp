/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_VO_BA_PROBLEM_HPP
#define NIE_CV_VO_BA_PROBLEM_HPP

#include <ceres/ceres.h>

#include <nie/core/geometry/isometry3.hpp>
#include <thread>

#include "nie/cv/vo/feature/keypoint.hpp"

namespace nie {

Keypoint const kDefaultKeypoint(-1, -1);

void BuildVoBaProblem(
    Eigen::Matrix3d const& K,
    std::vector<KeypointVector> const& keypoint_vectors,
    std::vector<Isometry3qd>* p_pose_estimates,
    std::vector<Eigen::Vector3d>* p_objt_estimates,
    ceres::Problem* problem);

bool SolveVoBaProblem(
    ceres::Problem* problem, std::uint32_t const num_threads = std::max(std::thread::hardware_concurrency(), 1u));

}  // namespace nie

#endif  // NIE_CV_VO_BA_PROBLEM_HPP
