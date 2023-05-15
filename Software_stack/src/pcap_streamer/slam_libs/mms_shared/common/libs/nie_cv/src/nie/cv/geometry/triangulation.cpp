/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "triangulation.hpp"

#include <ceres/ceres.h>
#include <glog/logging.h>
#include <nie/core/geometry/conversion.hpp>
#include <nie/core/geometry/covariance.hpp>
#include <opencv2/calib3d.hpp>

#include "nie/cv/ceres/back_projection_cost.hpp"
#include "nie/cv/ceres/identity_se3_cost.hpp"
#include "nie/cv/ceres/weighted_cost_function.hpp"

//  "Normalization" is now handled by subtracting the first origin and then adding that to the result.
//  TODO(jbr): Improve numeric stability / matrix conditioning - Perhaps normalize over all origins.

namespace nie {

bool TriangulateNonLinear(
    std::vector<nie::Isometry3qd> const& extrinsics_wc,
    std::vector<Eigen::Vector2f> const& image_points,
    Eigen::Matrix3d const& K,
    Eigen::Vector3d* x,
    unsigned int const num_threads) {
    if (!TriangulateLinear(extrinsics_wc, image_points, K, x)) return false;

    *x -= extrinsics_wc[0].translation();

    ceres::Problem problem;
    std::vector<double> intrinsics = CreateIntrinsicsVector(K);

    for (std::size_t i = 0; i < extrinsics_wc.size(); ++i) {
        nie::Isometry3qd extrinsics = extrinsics_wc[i];
        extrinsics.translation() -= extrinsics_wc[0].translation();
        ceres::CostFunction* cost_function = BackprojectionCost::Create(intrinsics, image_points[i], extrinsics);
        problem.AddResidualBlock(cost_function, nullptr, x->data());
    }

    ceres::Solver::Options options_solver;
    options_solver.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
    options_solver.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
    options_solver.num_threads = num_threads;
    if (!VLOG_IS_ON(8)) {  // When verbosity level is < 8
        options_solver.logging_type = ceres::SILENT;
    }

    ceres::Solver::Summary summary;
    ceres::Solve(options_solver, &problem, &summary);

    *x += extrinsics_wc[0].translation();

    return summary.IsSolutionUsable();
}

bool TriangulateNonLinear(
    std::vector<cv::Point3d> const& translations_wc,
    std::vector<cv::Vec3d> const& rotations_wc,
    std::vector<cv::Point2f> const& image_points,
    cv::Matx33d const& K,
    cv::Point3d* x,
    unsigned int const num_threads) {
    assert(translations_wc.size() == rotations_wc.size());

    std::vector<nie::Isometry3qd> extrinsics_wc_eigen(translations_wc.size());
    for (std::size_t i = 0; i < extrinsics_wc_eigen.size(); ++i) {
        extrinsics_wc_eigen[i].translation() = ConvertPoint(translations_wc[i]);
        Eigen::Vector3d aa = ConvertMat(rotations_wc[i]);
        extrinsics_wc_eigen[i].rotation() = Eigen::Quaterniond{Eigen::AngleAxisd{aa.norm(), aa.normalized()}};
    }

    std::vector<Eigen::Vector2f> image_points_eigen = ConvertPoints(image_points);
    Eigen::Vector3d x_eigen;

    bool success = TriangulateNonLinear(extrinsics_wc_eigen, image_points_eigen, ConvertMat(K), &x_eigen, num_threads);
    *x = ConvertPoint(x_eigen);

    return success;
}

bool TriangulateNonLinear(
    std::vector<nie::Isometry3qd> const& extrinsics_wc,
    std::vector<Eigen::Vector2f> const& image_points,
    std::vector<Eigen::Matrix<double, 6, 6>> const& extrinsics_wc_cov,
    std::vector<double> const& image_points_var,
    Eigen::Matrix3d const& K,
    Eigen::Vector3d* x,
    Eigen::Matrix3d* x_cov,
    unsigned int const num_threads) {
    CHECK_EQ(image_points.size(), image_points_var.size())
        << "The amount of image_points and image_points_stddevs should be equal.";

    // Initialize with linear estimate.
    if (!TriangulateNonLinear(extrinsics_wc, image_points, K, x, num_threads)) return false;

    *x -= extrinsics_wc[0].translation();

    std::vector<double> intrinsics = CreateIntrinsicsVector(K);

    std::vector<nie::Isometry3qd> extrinsics = extrinsics_wc;
    for (std::size_t i = 0; i < extrinsics_wc.size(); ++i) {
        extrinsics[i].translation() -= extrinsics_wc[0].translation();
    }

    ceres::Problem problem;
    ceres::LocalParameterization* quaternion_parameterization = new ceres::EigenQuaternionParameterization;

    for (std::size_t i = 0; i < extrinsics.size(); ++i) {
        ceres::CostFunction* cost_function_projection = VecWeightedCostFunction::Create(
            VecWeightedCostFunction::WeightsFromVariances({image_points_var[i], image_points_var[i]}),
            BackprojectionWithExtrinsicsCost::Create(intrinsics, image_points[i]));

        Eigen::Matrix<double, 6, 6> weights;
        CovarianceToWeights(extrinsics_wc_cov[i], &weights);
        ceres::CostFunction* cost_function_extrinsics = IdentitySe3Cost::Create(extrinsics[i], weights);

        double* t = extrinsics[i].translation().data();
        double* r = extrinsics[i].rotation().coeffs().data();

        problem.AddResidualBlock(cost_function_projection, nullptr, t, r, x->data());
        problem.AddResidualBlock(cost_function_extrinsics, nullptr, t, r);
        problem.SetParameterization(r, quaternion_parameterization);
    }

    std::vector<std::pair<const double*, const double*>> covariance_blocks{1};
    covariance_blocks[0] = {x->data(), x->data()};

    ceres::Covariance::Options options_covariance;
    options_covariance.num_threads = num_threads;

    ceres::Covariance covariance(options_covariance);
    covariance.Compute(covariance_blocks, &problem);
    covariance.GetCovarianceBlock(x->data(), x->data(), x_cov->data());
    *x += extrinsics_wc[0].translation();

    return true;
}

bool TriangulateNonLinear(
    std::vector<nie::Isometry3qd> const& extrinsics_wc,
    std::vector<Eigen::Vector2f> const& image_points,
    std::vector<Eigen::Matrix<double, 6, 6>> const& extrinsics_wc_cov,
    double const& image_points_var,
    Eigen::Matrix3d const& K,
    Eigen::Vector3d* x,
    Eigen::Matrix3d* x_cov,
    unsigned int const num_threads) {
    return TriangulateNonLinear(
        extrinsics_wc,
        image_points,
        extrinsics_wc_cov,
        std::vector<double>(image_points.size(), image_points_var),
        K,
        x,
        x_cov,
        num_threads);
}

}  // namespace nie
