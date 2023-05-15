/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_CALIB3D_PINHOLE_MODEL_HPP
#define NIE_CV_CALIB3D_PINHOLE_MODEL_HPP

#include <algorithm>
#include <thread>

#include <ceres/ceres.h>
#include <Eigen/Core>

#include "nie/cv/calib3d/distortion_model_parameters.hpp"
#include "nie/cv/calib3d/pinhole_distortion_behavior.hpp"
#include "nie/cv/ceres/pinhole_undistort_cost.hpp"

namespace nie {

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
class PinholeDistortionModel {
public:
    static constexpr ParameterFocalLength kParameterFocalLength = FL;
    static constexpr ParameterSkew kParameterSkew = SK;
    static constexpr ParameterDistortionRadial kParameterDistortionRadial = RA;
    static constexpr ParameterDistortionTangential kParameterDistortionTangential = TA;
    static constexpr ParameterDistortionThinPrism kParameterDistortionThinPrism = TP;

    static constexpr int kParametersK = detail::PinholeDistortionModelHelper<FL, SK, RA, TA, TP>::kParametersK;
    static constexpr int kParametersIntrinsics =
        detail::PinholeDistortionModelHelper<FL, SK, RA, TA, TP>::kParametersIntrinsics;

    PinholeDistortionModel() { intrinsics_.resize(kParametersIntrinsics, 0); }
    explicit PinholeDistortionModel(const Eigen::Matrix3d& K);
    explicit PinholeDistortionModel(std::vector<double> intrinsics);

    void Distort(Eigen::Vector2f const& p_u, Eigen::Vector2f* p_d) const;

    // Distort image coordinates as if the distorted image does not have the same K matrix as the undistorted image.
    void Distort(Eigen::Matrix3d const& K_undistorted, Eigen::Vector2f const& p_u, Eigen::Vector2f* p_d) const;

    void Undistort(
        Eigen::Vector2f const& p_d,
        Eigen::Vector2f* p_u,
        const unsigned int num_threads = std::thread::hardware_concurrency()) const;

    // Undistort image coordinates as if the distorted image does not have the same K matrix as the undistorted image.
    void Undistort(
        Eigen::Matrix3d const& K_undistorted,
        Eigen::Vector2f const& p_d,
        Eigen::Vector2f* p_u,
        const unsigned int num_threads = std::thread::hardware_concurrency()) const;

    void Undistort(
        Eigen::Matrix3d const& K_undistorted,
        std::vector<Eigen::Vector2f> const& p_d,
        std::vector<Eigen::Vector2f>* p_u,
        const unsigned int num_threads = std::thread::hardware_concurrency()) const;

    Eigen::Matrix3d K() const;
    std::vector<double> const& intrinsics() const { return intrinsics_; };
    std::vector<double>& intrinsics() { return intrinsics_; };

    static std::vector<double> CreateVector(Eigen::Matrix3d const& K);
    static Eigen::Matrix3d CreateK(std::vector<double> const& intrinsics);

protected:
    // NOTE: NOT a general undistort -> distort. Only the K part is used from the reference intrinsics.
    template <typename T>
    void Distort(
        std::vector<T> const& intrinsics_from_K_only,
        std::vector<T> const& intrinsics_to,
        Eigen::Vector2f const& p_u,
        Eigen::Vector2f* p_d) const;
    template <typename T>
    void Undistort(
        std::vector<T> const& intrinsics_from,
        std::vector<T> const& intrinsics_to_K_only,
        const unsigned int num_threads,
        Eigen::Vector2f const& p_d,
        Eigen::Vector2f* p_u) const;
    template <typename T>
    void Undistort(
        std::vector<T> const& intrinsics_from,
        std::vector<T> const& intrinsics_to_K_only,
        const unsigned int num_threads,
        std::vector<Eigen::Vector2f> const& p_d,
        std::vector<Eigen::Vector2f>* p_u) const;

    std::vector<double> intrinsics_;
};

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
PinholeDistortionModel<FL, SK, RA, TA, TP>::PinholeDistortionModel(Eigen::Matrix3d const& K)
    : intrinsics_(CreateVector(K)) {}

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
PinholeDistortionModel<FL, SK, RA, TA, TP>::PinholeDistortionModel(std::vector<double> intrinsics)
    : intrinsics_(std::move(intrinsics)) {
    if (intrinsics_.size() != static_cast<std::size_t>(kParametersIntrinsics)) {
        throw std::invalid_argument(
            "PinholeDistortionModel::PinholeDistortionModel(): Vector input size does not match model parameter "
            "count.");
    }
}

// Public Distort/Undistort API
template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
void PinholeDistortionModel<FL, SK, RA, TA, TP>::Distort(Eigen::Vector2f const& p_u, Eigen::Vector2f* p_d) const {
    Distort(intrinsics_, intrinsics_, p_u, p_d);
}

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
void PinholeDistortionModel<FL, SK, RA, TA, TP>::Distort(
    Eigen::Matrix3d const& K_undistorted, Eigen::Vector2f const& p_u, Eigen::Vector2f* p_d) const {
    PinholeDistortionModel dm(K_undistorted);
    Distort(dm.intrinsics_, intrinsics_, p_u, p_d);
}

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
void PinholeDistortionModel<FL, SK, RA, TA, TP>::Undistort(
    Eigen::Vector2f const& p_d, Eigen::Vector2f* p_u, const unsigned int num_threads) const {
    Undistort(intrinsics_, intrinsics_, num_threads, p_d, p_u);
}

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
void PinholeDistortionModel<FL, SK, RA, TA, TP>::Undistort(
    Eigen::Matrix3d const& K_undistorted,
    Eigen::Vector2f const& p_d,
    Eigen::Vector2f* p_u,
    const unsigned int num_threads) const {
    PinholeDistortionModel dm(K_undistorted);
    Undistort(intrinsics_, dm.intrinsics_, num_threads, p_d, p_u);
}

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
void PinholeDistortionModel<FL, SK, RA, TA, TP>::Undistort(
    Eigen::Matrix3d const& K_undistorted,
    std::vector<Eigen::Vector2f> const& p_d,
    std::vector<Eigen::Vector2f>* p_u,
    const unsigned int num_threads) const {
    PinholeDistortionModel dm(K_undistorted);
    Undistort(intrinsics_, dm.intrinsics_, num_threads, p_d, p_u);
}

// Internal Distort/Undistort Implementations
// NOTE: NOT a general undistort -> distort. Only the K part is used from the reference intrinsics.
template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
template <typename T>
void PinholeDistortionModel<FL, SK, RA, TA, TP>::Distort(
    std::vector<T> const& intrinsics_from_K_only,
    std::vector<T> const& intrinsics_to,
    Eigen::Vector2f const& p_u,
    Eigen::Vector2f* p_d) const {
    T v_u = p_u.x();
    T v_v = p_u.y();
    detail::BehaviorFocalLength<FL>::Undo(intrinsics_from_K_only.data(), v_u, v_v, &v_u, &v_v);
    detail::BehaviorSkew<SK>::Undo(intrinsics_from_K_only.data(), v_u, v_v, &v_u, &v_v);
    // Still cast here because this is shared with a whole other bunch of things
    // incl. calculations regarding extrinsics (which we will keep as double for
    // the time being).
    Eigen::Vector2d cp_d = p_d->template cast<double>();
    detail::Distort<FL, SK, RA, TA, TP, T>(intrinsics_to.data(), v_u, v_v, &cp_d.x(), &cp_d.y());
    *p_d = cp_d.template cast<float>();
}

// Undistort a distorted point. Invert the lens distortion model polynomial
// function that maps from an undistorted to distorted point.
template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
template <typename T>
void PinholeDistortionModel<FL, SK, RA, TA, TP>::Undistort(
    std::vector<T> const& intrinsics_from,
    std::vector<T> const& intrinsics_to_K_only,
    const unsigned int num_threads,
    Eigen::Vector2f const& p_d,
    Eigen::Vector2f* p_u) const {
    const Eigen::Vector2d cp_d = p_d.template cast<double>();
    // The undistorted point initialized with the distorted one. In general,
    // initialization for gradient decent is important. Is this choice a problem?
    Eigen::Vector2d cp_u = cp_d;

    ceres::Problem problem;
    ceres::CostFunction* cost_function =
        PinholeUndistortCost<FL, SK, RA, TA, TP>::Create(intrinsics_from, intrinsics_to_K_only, cp_d);
    problem.AddResidualBlock(cost_function, nullptr, cp_u.data());

    // There is no sparsity in this function (2x2 Jacobian)
    ceres::Solver::Options options_solver;
    options_solver.linear_solver_type = ceres::LinearSolverType::DENSE_QR;
    options_solver.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
    options_solver.num_threads = num_threads;

    ceres::Solver::Summary summary;
    ceres::Solve(options_solver, &problem, &summary);

    *p_u = cp_u.template cast<float>();
}

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
template <typename T>
void PinholeDistortionModel<FL, SK, RA, TA, TP>::Undistort(
    std::vector<T> const& intrinsics_from,
    std::vector<T> const& intrinsics_to_K_only,
    const unsigned int num_threads,
    std::vector<Eigen::Vector2f> const& p_d,
    std::vector<Eigen::Vector2f>* p_u) const {
    std::vector<Eigen::Vector2d> cp_d;
    cp_d.reserve(p_d.size());
    std::transform(p_d.cbegin(), p_d.cend(), std::back_inserter(cp_d), [](auto const& input_p_d) {
        return input_p_d.template cast<double>();
    });
    // Here, we also initialize the output using the input.
    std::vector<Eigen::Vector2d> cp_u = cp_d;

    ceres::Problem problem;
    for (size_t i = 0; i < cp_d.size(); ++i) {
        ceres::CostFunction* cost_function =
            PinholeUndistortCost<FL, SK, RA, TA, TP>::Create(intrinsics_from, intrinsics_to_K_only, cp_d[i]);
        problem.AddResidualBlock(cost_function, nullptr, cp_u[i].data());
    }

    ceres::Solver::Options options_solver;
    // Got SPARSE_NORMAL_CHOLESKY from the ceres tutorial where they basically
    // change to this one once the problem becomes "too large" for QR_DENSE
    options_solver.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
    options_solver.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
    options_solver.num_threads = num_threads;

    ceres::Solver::Summary summary;
    ceres::Solve(options_solver, &problem, &summary);

    p_u->reserve(cp_u.size());
    std::transform(cp_u.cbegin(), cp_u.cend(), std::back_inserter(*p_u), [](auto const& out_p) {
        return out_p.template cast<float>();
    });
}

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
Eigen::Matrix3d PinholeDistortionModel<FL, SK, RA, TA, TP>::K() const {
    return CreateK(intrinsics_);
}

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
std::vector<double> PinholeDistortionModel<FL, SK, RA, TA, TP>::CreateVector(Eigen::Matrix3d const& K) {
    std::vector<double> intrinsics(kParametersIntrinsics, 0);

    intrinsics[0] = K(0, 0);

    if (detail::BehaviorFocalLength<FL>::kParameters == 2) {
        intrinsics[1] = K(1, 1);
    }

    if (detail::BehaviorSkew<SK>::kParameters == 1) {
        intrinsics[detail::PinholeDistortionModelHelper<FL, SK, RA, TA, TP>::kIndexDistortionSkew] =
            K(0, 1) / intrinsics[0];
    }

    intrinsics[detail::PinholeDistortionModelHelper<FL, SK, RA, TA, TP>::kIndexPrincipalPoint + 0] = K(0, 2);
    intrinsics[detail::PinholeDistortionModelHelper<FL, SK, RA, TA, TP>::kIndexPrincipalPoint + 1] = K(1, 2);

    return intrinsics;
}

template <
    ParameterFocalLength FL,
    ParameterSkew SK,
    ParameterDistortionRadial RA,
    ParameterDistortionTangential TA,
    ParameterDistortionThinPrism TP>
Eigen::Matrix3d PinholeDistortionModel<FL, SK, RA, TA, TP>::CreateK(std::vector<double> const& intrinsics) {
    double f_x = intrinsics[0];
    double f_y;
    double skew = 0.0;

    if (detail::BehaviorFocalLength<FL>::kParameters == 2) {
        f_y = intrinsics[1];
    } else {
        f_y = f_x;
    }

    if (detail::BehaviorSkew<SK>::kParameters == 1) {
        skew =
            intrinsics[detail::PinholeDistortionModelHelper<FL, SK, RA, TA, TP>::kIndexDistortionSkew] * intrinsics[0];
    }

    double pp_x = intrinsics[detail::PinholeDistortionModelHelper<FL, SK, RA, TA, TP>::kIndexPrincipalPoint + 0];
    double pp_y = intrinsics[detail::PinholeDistortionModelHelper<FL, SK, RA, TA, TP>::kIndexPrincipalPoint + 1];

    Eigen::Matrix3d intrinsics_mat;
    intrinsics_mat << f_x, skew, pp_x, 0.0, f_y, pp_y, 0.0, 0.0, 1.0;
    return intrinsics_mat;
}

}  // namespace nie

#endif  // NIE_CV_CALIB3D_PINHOLE_MODEL_HPP
