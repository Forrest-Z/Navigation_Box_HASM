/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "ba_problem.hpp"

#include <nie/core/geometry/conversion.hpp>

#include "nie/cv/ceres/back_projection_cost.hpp"

namespace nie {

void BuildVoBaProblem(
        Eigen::Matrix3d const& K,
        std::vector<KeypointVector> const& keypoint_vectors,
        std::vector<Isometry3qd>* p_pose_estimates,
        std::vector<Eigen::Vector3d>* p_objt_estimates,
        ceres::Problem* problem) {
    assert(p_pose_estimates != nullptr);
    assert(p_objt_estimates != nullptr);
    assert(p_pose_estimates->size() > 1);
    assert(keypoint_vectors.size() == p_objt_estimates->size());
    for (auto const& keypoint_vector [[maybe_unused]] : keypoint_vectors) {
        assert(keypoint_vector.size() == p_pose_estimates->size());
    }

    ceres::LossFunction* loss_function = nullptr;
    ceres::LocalParameterization* quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;

    for (Isometry3qd& p : *p_pose_estimates) {
        Eigen::Vector3d& t = p.translation();
        Eigen::Quaterniond& r = p.rotation();
        problem->AddParameterBlock(t.data(), static_cast<int>(t.size()));
        problem->AddParameterBlock(
                r.coeffs().data(), static_cast<int>(r.coeffs().size()), quaternion_local_parameterization);
    }

    for (auto& x : *p_objt_estimates) {
        problem->AddParameterBlock(x.data(), static_cast<int>(x.size()));
    }

    std::vector<double> intrinsics = nie::CreateIntrinsicsVector(K);

    for (std::size_t object_index = 0; object_index < keypoint_vectors.size(); ++object_index) {
        Eigen::Vector3d& x = (*p_objt_estimates)[object_index];

        KeypointVector const& keypoint_vector = keypoint_vectors[object_index];
        for (std::size_t pose_index = 0; pose_index < keypoint_vector.size(); ++pose_index) {
            Keypoint const& k = keypoint_vector[pose_index];

            if (k == kDefaultKeypoint) {
                continue;
            }

            Isometry3qd& p = (*p_pose_estimates)[pose_index];
            Eigen::Vector3d& t = p.translation();
            Eigen::Quaterniond& r = p.rotation();

            ceres::CostFunction* cost_function =
                    nie::BackprojectionWithExtrinsicsCost::Create(intrinsics, nie::ConvertPoint(k));
            problem->AddResidualBlock(cost_function, loss_function, t.data(), r.coeffs().data(), x.data());
        }
    }

    // The first two positions remain fixed to limit the gauge freedom. Scale is thus determined by the first two
    // positions.
    problem->SetParameterBlockConstant((*p_pose_estimates)[0].translation().data());
    problem->SetParameterBlockConstant((*p_pose_estimates)[0].rotation().coeffs().data());
    problem->SetParameterBlockConstant((*p_pose_estimates)[1].translation().data());
    problem->SetParameterBlockConstant((*p_pose_estimates)[1].rotation().coeffs().data());
}

// Returns true if the solve was successful.
bool SolveVoBaProblem(ceres::Problem* problem, std::uint32_t const num_threads) {
    assert(problem != nullptr);

    ceres::Solver::Options options;
    options.max_num_iterations = 200;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.num_threads = num_threads;
    if (!VLOG_IS_ON(8)) {  // When verbosity level is < 8
        options.logging_type = ceres::SILENT;
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);

    if (VLOG_IS_ON(5)) {  // When verbosity level is >= 5
        std::cout << summary.FullReport() << std::endl;
    }

    return summary.IsSolutionUsable();
}

}  // namespace nie
