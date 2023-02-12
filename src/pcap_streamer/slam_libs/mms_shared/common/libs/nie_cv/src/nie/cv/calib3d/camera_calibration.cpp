/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "camera_calibration.hpp"

#include <cassert>
#include <limits>

#include <ceres/ceres.h>
#include <nie/core/geometry/conversion.hpp>
#include <opencv2/calib3d.hpp>

#include "nie/cv/ceres/calibration_cost.hpp"
#include "nie/cv/ceres/calibration_problem.hpp"
#include "nie/cv/ceres/covariance_isometry.hpp"

#include <iostream>
#include <thread>

namespace nie {

// Iac: Image of the Absolute Conic
cv::Matx33d KfromIac(const cv::Matx<double, 1, 6>& iac) {
    double t0 = iac(1) * iac(3) - iac(0) * iac(4);
    double t1 = iac(0) * iac(2) - iac(1) * iac(1);

    double ppy = t0 / t1;
    double l = iac(5) - (iac(3) * iac(3) + ppy * t0) / iac(0);
    double fx = std::sqrt(l / iac(0));
    double fy = std::sqrt(l * iac(0) / t1);
    double s = -iac(1) * fx * fx * fy / l;
    double ppx = s * ppy / fy - iac(3) * fx * fx / l;

    // Note that the signs of the focal lengths are lost due to the squaring that
    // happens in the relevant parts of the IAC. See paper.
    // In a right-handed coordinate system the y-axis of images and 3d coordinates
    // can have different signs (X-east, Y-up, Z-south).
    // We chose to use the opencv one in which they are still positive
    // (X-east, Y-down, Z-north).
    return cv::Matx33d{fx, s, ppx, 0.0, fy, ppy, 0.0, 0.0, 1.0};
}

// Builds the constraint vector from 2 selected columns of the homography
cv::Vec<double, 6> ConstraintVectorFromH(const cv::Matx33d& H, int c0, int c1) {
    return cv::Vec<double, 6>{H(0, c0) * H(0, c1),
                              H(0, c0) * H(1, c1) + H(1, c0) * H(0, c1),
                              H(1, c0) * H(1, c1),
                              H(2, c0) * H(0, c1) + H(0, c0) * H(2, c1),
                              H(2, c0) * H(1, c1) + H(1, c0) * H(2, c1),
                              H(2, c0) * H(2, c1)};
}

cv::Matx33d EstimateKfromHomographies(const std::vector<cv::Matx33d>& homographies) {
    // Estimate the image of the absolute conic as a 6 vector (symmetric matrix allows us to drop 3 values)
    // First get the two constraints imposed by each homography and use those to setup the equations.
    // Solve A^tAx=0
    cv::Matx<double, 6, 6> A;

    for (std::size_t i = 0; i < homographies.size(); ++i) {
        const cv::Matx33d& H = homographies[i];
        cv::Vec<double, 6> row0 = ConstraintVectorFromH(H, 0, 1);
        cv::Vec<double, 6> row1 = ConstraintVectorFromH(H, 0, 0) - ConstraintVectorFromH(H, 1, 1);

        A += row0 * row0.t();
        A += row1 * row1.t();
    }

    cv::Vec<double, 6> A_w;
    cv::Matx<double, 6, 6> A_u;
    cv::Matx<double, 6, 6> A_vt;
    cv::SVD::compute(A, A_w, A_u, A_vt, cv::SVD::MODIFY_A);

    return KfromIac(A_vt.row(5));
}

void EsimateExtrinsicsFromHomographiesAndK(
    const std::vector<cv::Matx33d>& homographies,
    const cv::Matx33d& K,
    std::vector<cv::Point3d>* translations,
    std::vector<cv::Matx33d>* rotations) {
    translations->resize(homographies.size());
    rotations->resize(homographies.size());

    cv::Matx33d K_inv = K.inv();

    for (std::size_t i = 0; i < homographies.size(); ++i) {
        // If we have a negative determinant, then the image is on the opposite side
        // of the plane. This means it would not possible to see the plane.
        // OpenCV does not check the sign of the determinant when it normalizes it.
        cv::Matx33d H;

        if (cv::determinant(homographies[i]) < 0.0) {
            H = homographies[i] * -1.0;
        } else {
            H = homographies[i];
        }

        cv::Matx<double, 3, 1> r0 = K_inv * H.col(0);
        cv::Matx<double, 3, 1> r1 = K_inv * H.col(1);
        cv::Matx33d& R = (*rotations)[i];
        // These should be the same but are not exactly. Maybe not important.
        double l0 = 1.0 / cv::norm(r0);
        double l1 = 1.0 / cv::norm(r1);

        R(0, 0) = r0(0) * l0;
        R(1, 0) = r0(1) * l0;
        R(2, 0) = r0(2) * l0;
        R(0, 1) = r1(0) * l1;
        R(1, 1) = r1(1) * l1;
        R(2, 1) = r1(2) * l1;
        // Right hand rule to get the missing axis.
        R(0, 2) = R(1, 0) * R(2, 1) - R(2, 0) * R(1, 1);
        R(1, 2) = R(2, 0) * R(0, 1) - R(0, 0) * R(2, 1);
        R(2, 2) = R(0, 0) * R(1, 1) - R(1, 0) * R(0, 1);

        // The above solution to obtain a rotation matrix does not typically satisfy
        // the properties of a rotation matrix. The solution below gives the closest
        // rotation matrix  in terms of the Frobenius norm.
        cv::Vec3d w;
        cv::Matx33d u;
        cv::Matx33d vt;
        cv::SVD::compute(R, w, u, vt, cv::SVD::MODIFY_A);
        R = u * vt;

        // The solved translation equals the position of the origin from the perspective
        // of the camera.
        cv::Matx<double, 3, 1> t = K_inv * H.col(2) * l0;
        (*translations)[i] = {t(0), t(1), t(2)};
    }
}

/// @brief A linear estimation of camera intrinsics and extrinsics using Zhang's method.
/// @details Outputs extrinsics in the world to camera convention.
// Rotation matrices bring world vectors into camera referenced vectors: V_c = R * V_w;
//
// The full projection can be handled as:
//
//      p = K * (R * X_w + t)
//
// The result is up to scale (divide by the Z of the result). Similar with a 4x4 matrix:
//
//          [ KR  | Kt ]
//      p = [ 0^T | 1  ] [ X_w, 1]
//
// The position of each camera may be determined as:
//
//      C_w = R^T * -t          (Obtained from 0 = R * C_w + t)
//
void EstimateCameraParametersZhangClosedForm(
    const std::vector<cv::Point3f>& object_points,
    const std::vector<std::vector<cv::Point2f>>& image_points,
    cv::Matx33d* K,
    std::vector<cv::Point3d>* translations,
    std::vector<cv::Matx33d>* rotations) {
    assert(image_points.size() > 2);

    cv::Scalar mean, stddev;
    cv::meanStdDev(object_points, mean, stddev);
    assert(mean[2] < std::numeric_limits<double>::epsilon() && stddev[2] < std::numeric_limits<double>::epsilon());

    std::vector<cv::Point2f> object_points_2d(object_points.size());

    for (std::size_t i = 0; i < object_points.size(); ++i) {
        object_points_2d[i] = {object_points[i].x, object_points[i].y};
    }

    std::vector<cv::Matx33d> homographies(image_points.size());

    for (std::size_t i = 0; i < image_points.size(); ++i) {
        // For now we assume that all points must have been seen.
        assert(object_points.size() == image_points[i].size());

        homographies[i] = cv::findHomography(object_points_2d, image_points[i], 0);
    }

    *K = EstimateKfromHomographies(homographies);

    EsimateExtrinsicsFromHomographiesAndK(homographies, *K, translations, rotations);
}

// TODO(jbr): Perhaps one day in the future we will change this when we care enough.
void EstimateCameraParametersZhangClosedForm_Eigen(
    std::vector<Eigen::Vector3f> const& object_points_eigen,
    std::vector<std::vector<Eigen::Vector2f>> const& image_points_eigen,
    Eigen::Matrix3d* K_eigen,
    std::vector<Eigen::Vector3d>* translations_eigen,
    std::vector<Eigen::Matrix3d>* rotations_eigen) {
    std::vector<cv::Point3f> object_points = nie::ConvertPoints(object_points_eigen);
    std::vector<std::vector<cv::Point2f>> image_points(image_points_eigen.size());
    for (std::size_t i = 0; i < image_points_eigen.size(); ++i) {
        image_points[i] = nie::ConvertPoints(image_points_eigen[i]);
    }
    cv::Matx33d K;
    std::vector<cv::Point3d> translations;
    std::vector<cv::Matx33d> rotations;
    EstimateCameraParametersZhangClosedForm(object_points, image_points, &K, &translations, &rotations);
    *K_eigen = nie::ConvertMat(K);
    *translations_eigen = nie::ConvertPoints(translations);
    *rotations_eigen = nie::ConvertMats(rotations);
}

void GetCovarianceBlocksIsometries(
    ceres::Covariance const& covariance,
    std::vector<nie::Isometry3qd> const& extrinsics,
    double const& variance_residuals,
    std::vector<Eigen::Matrix<double, 6, 6>>* p_cov_extrinsics) {
    std::vector<Eigen::Matrix<double, 6, 6>>& cov_extrinsics = *p_cov_extrinsics;
    cov_extrinsics.resize(extrinsics.size());

    for (std::size_t i = 0; i < extrinsics.size(); ++i) {
        GetCovarianceMatrix(covariance, extrinsics[i], &cov_extrinsics[i]);
        cov_extrinsics[i] *= variance_residuals;
    }
}

// For the intrinsics covariance it doesn't matter that Eigen is ColMajor and Ceres RowMajor.
void GetCovarianceBlockIntrinsics(
    ceres::Covariance const& covariance,
    std::vector<double> const& intrinsics,
    double const& variance_residuals,
    Eigen::MatrixXd* p_cov_intrinsics) {
    *p_cov_intrinsics = Eigen::MatrixXd(intrinsics.size(), intrinsics.size());
    covariance.GetCovarianceBlock(intrinsics.data(), intrinsics.data(), p_cov_intrinsics->data());
    *p_cov_intrinsics *= variance_residuals;
}

class ResidualCalculator {
public:
    ResidualCalculator(std::unique_ptr<CalibrationCostDefinition> const& factory, std::vector<double> const& intrinsics)
        : factory_(factory), intrinsics_(intrinsics) {}

    Eigen::Vector2f operator()(
        Isometry3qd const& extrinsics, Eigen::Vector3f const& object_point, Eigen::Vector2f const& image_point) const {
        ceres::CostFunction* cost_function = GetCostFunction(object_point, image_point);

        double r[2];
        cost_function->Evaluate(GetParameters(extrinsics).data(), r, nullptr);

        delete cost_function;

        return Eigen::Vector2f(static_cast<float>(r[0]), static_cast<float>(r[1]));
    }

protected:
    virtual ceres::CostFunction* GetCostFunction(
        Eigen::Vector3f const& object_point, Eigen::Vector2f const& image_point) const {
        return factory_->Create(object_point, image_point);
    }

    virtual std::vector<double const*> GetParameters(Isometry3qd const& extrinsics) const {
        return {intrinsics_.data(), extrinsics.translation().data(), extrinsics.rotation().coeffs().data()};
    }

    const std::unique_ptr<CalibrationCostDefinition>& factory_;
    const std::vector<double>& intrinsics_;
};

class ResidualCalculatorWithBaseline : public ResidualCalculator {
public:
    ResidualCalculatorWithBaseline(
        std::unique_ptr<CalibrationCostDefinition> const& factory,
        std::vector<double> const& intrinsics,
        Isometry3qd const& baseline)
        : ResidualCalculator(factory, intrinsics), baseline_(baseline) {}

protected:
    ceres::CostFunction* GetCostFunction(
        Eigen::Vector3f const& object_point, Eigen::Vector2f const& image_point) const override {
        return factory_->CreateWithBaseline(object_point, image_point);
    }

    std::vector<const double*> GetParameters(Isometry3qd const& extrinsics) const override {
        return {intrinsics_.data(),
                extrinsics.translation().data(),
                extrinsics.rotation().coeffs().data(),
                baseline_.translation().data(),
                baseline_.rotation().coeffs().data()};
    }

    Isometry3qd const& baseline_;
};

// The extrinsics always start counting from 0, but we don't know which selection of image points belong to it. Many
// positions for the right camera require both the left position + baseline. We calculate residuals for all left images,
// baseline count images (to project in the right image using the baseline), and finally all leftover right images.
template <typename Calculator>
void CalculateResidualsCamera(
    std::vector<Eigen::Vector3f> const& object_points,
    std::vector<std::vector<Eigen::Vector2f>> const& image_points,
    std::vector<Isometry3qd> const& extrinsics,
    Calculator const& calculator,
    std::size_t const begin_image,
    std::size_t const end_image,
    std::vector<std::vector<Eigen::Vector2f>>* residuals) {
    const std::size_t object_point_count = object_points.size();
    residuals->resize(image_points.size());

    std::size_t const size = end_image - begin_image;

    for (std::size_t i = 0; i < size; ++i) {
        std::size_t index_image = begin_image + i;
        (*residuals)[index_image].resize(object_point_count);

        for (std::size_t j = 0; j < object_point_count; ++j) {
            (*residuals)[index_image][j] = calculator(extrinsics[i], object_points[j], image_points[index_image][j]);
        }
    }
}

template <typename Calculator>
void CalculateResidualsCamera(
    std::vector<Eigen::Vector3f> const& object_points,
    std::vector<std::vector<Eigen::Vector2f>> const& image_points,
    std::vector<Isometry3qd> const& extrinsics,
    Calculator const& calculator,
    std::vector<std::vector<Eigen::Vector2f>>* residuals) {
    CalculateResidualsCamera(object_points, image_points, extrinsics, calculator, 0, image_points.size(), residuals);
}

ceres::ResidualBlockId AddResidualBlock(
    std::unique_ptr<CalibrationCostDefinition> const& factory,
    Eigen::Vector3f const& object_point,
    Eigen::Vector2f const& image_point,
    ceres::Problem* problem,
    std::vector<double>* intrinsics,
    nie::Isometry3qd* extrinsics) {
    ceres::CostFunction* cost_function = factory->Create(object_point, image_point);
    ceres::ResidualBlockId id = problem->AddResidualBlock(
        cost_function,
        nullptr,
        intrinsics->data(),
        extrinsics->translation().data(),
        extrinsics->rotation().coeffs().data());
    return id;
}

ceres::ResidualBlockId AddResidualBlock(
    std::unique_ptr<CalibrationCostDefinition> const& factory,
    Eigen::Vector3f const& object_point,
    Eigen::Vector2f const& image_point,
    ceres::Problem* problem,
    std::vector<double>* intrinsics,
    nie::Isometry3qd* extrinsics,
    nie::Isometry3qd* baseline) {
    ceres::CostFunction* cost_function = factory->CreateWithBaseline(object_point, image_point);
    ceres::ResidualBlockId id = problem->AddResidualBlock(
        cost_function,
        nullptr,
        intrinsics->data(),
        extrinsics->translation().data(),
        extrinsics->rotation().coeffs().data(),
        baseline->translation().data(),
        baseline->rotation().coeffs().data());
    return id;
}

void EstimateCameraParametersCeres(
    DistortionModelParameters const& parameters,
    std::vector<Eigen::Vector3f> const& object_points,
    std::vector<std::vector<Eigen::Vector2f>> const& image_points,
    std::vector<double>* p_intrinsics,
    std::vector<nie::Isometry3qd>* p_extrinsics,
    std::vector<std::vector<Eigen::Vector2f>>* p_residuals,
    Eigen::MatrixXd* p_cov_intrinsics,
    std::vector<Eigen::Matrix<double, 6, 6>>* p_cov_extrinsics,
    double* p_var_residuals,
    const unsigned int num_threads) {
    // ****************************************************************************************************************
    // SOLVER
    // ****************************************************************************************************************

    std::unique_ptr<CalibrationCostDefinition> factory = CalibrationCostDefinitionFactory::Create(parameters);
    const std::size_t object_point_count = object_points.size();
    const std::size_t image_count = image_points.size();
    const std::size_t parameter_count_intrinsics = factory->GetParameterCountIntrinsics();

    Eigen::Matrix3d K;
    std::vector<Eigen::Vector3d> translations;
    std::vector<Eigen::Matrix3d> rotations;

    // Get initial estimate.
    EstimateCameraParametersZhangClosedForm_Eigen(object_points, image_points, &K, &translations, &rotations);

    std::vector<double>& intrinsics = *p_intrinsics;
    intrinsics = factory->InitializeIntrinsicsFromK(K);

    std::vector<nie::Isometry3qd>& extrinsics = *p_extrinsics;
    extrinsics.resize(image_count);

    for (std::size_t i = 0; i < image_count; ++i) {
        extrinsics[i].translation() = translations[i];
        extrinsics[i].rotation() = Eigen::Quaterniond{rotations[i]};
        extrinsics[i].Inverse();
    }

    ceres::Problem problem;
    ceres::LocalParameterization* quaternion_parameterization = new ceres::EigenQuaternionParameterization;
    // TODO(jbr): Not super important, but check if we can optimize using custom parameter block ordering.
    // Add parameters blocks that might later be ordered for optimal sparse bundle adjustment. For now
    // we assume ceres does a good job using the sparse decomposition option.
    //    problem.AddParameterBlock(p_intrinsics, (*model)->intrinsics().size());
    //
    //    for (std::size_t i = 0; i < image_count; ++i)
    //    {
    //        double* p_extrinsics = &extrinsics[parameter_count_extrinsics * i];
    //        problem.AddParameterBlock(p_extrinsics, parameter_count_extrinsics);
    //    }
    for (std::size_t i = 0; i < image_count; ++i) {
        for (std::size_t j = 0; j < object_point_count; ++j) {
            AddResidualBlock(factory, object_points[j], image_points[i][j], &problem, &intrinsics, &extrinsics[i]);
            problem.SetParameterization(extrinsics[i].rotation().coeffs().data(), quaternion_parameterization);
        }
    }

    ceres::Solver::Options options_solver;
    options_solver.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
    options_solver.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
    options_solver.num_threads = num_threads;

    ceres::Solver::Summary summary;
    ceres::Solve(options_solver, &problem, &summary);

    // TODO(jbr): Logging on some V level and perhaps full report.
    // std::cout << summary.BriefReport() << std::endl;

    // ****************************************************************************************************************
    // VARIANCE COVARIANCE
    // ****************************************************************************************************************

    CalculateResidualsCamera(
        object_points, image_points, extrinsics, ResidualCalculator(factory, intrinsics), p_residuals);

    // We estimate the input input_variance (no a priori standard deviation). This will be used to scale the output
    // variances.
    double variance_residuals = 0.0;

    for (std::size_t i = 0; i < image_count; ++i) {
        for (std::size_t j = 0; j < object_point_count; ++j) {
            variance_residuals += (*p_residuals)[i][j].squaredNorm();
        }
    }

    // This calculates the average squared error or variance while taking into account the degrees of freedom of the
    // problem.
    std::size_t const degrees_of_freedom_extrinsics = 6;  // Not 7, degrees of freedom != parameter count

    variance_residuals =
        variance_residuals / static_cast<double>(
                                 (image_count * object_point_count) -
                                 (image_count * degrees_of_freedom_extrinsics + parameter_count_intrinsics));

    ceres::Covariance::Options options_covariance;
    options_covariance.num_threads = num_threads;

    ceres::Covariance covariance(options_covariance);

    std::vector<std::pair<const double*, const double*>> covariance_blocks{1 + image_count * 3};
    covariance_blocks[0] = {intrinsics.data(), intrinsics.data()};

    for (std::size_t i = 0; i < image_count; ++i) {
        std::size_t index = 1 + i * 3;
        SetCovarianceBlocksIsometry(extrinsics[i], index, &covariance_blocks);
    }

    covariance.Compute(covariance_blocks, &problem);

    GetCovarianceBlockIntrinsics(covariance, intrinsics, variance_residuals, p_cov_intrinsics);
    GetCovarianceBlocksIsometries(covariance, extrinsics, variance_residuals, p_cov_extrinsics);
    *p_var_residuals = variance_residuals;
}

void EstimateStereoParametersCeres(
    DistortionModelParameters const& parameters,
    std::vector<Eigen::Vector3f> const& object_points,
    std::vector<std::vector<Eigen::Vector2f>> const& image_points_left,
    std::vector<std::vector<Eigen::Vector2f>> const& image_points_right,
    std::size_t const& pair_count,
    std::vector<double>* p_intrinsics_left,
    std::vector<double>* p_intrinsics_right,
    std::vector<nie::Isometry3qd>* p_extrinsics_left,
    // The translations of the right camera that aren't estimated with respect to the left one
    // are stored here. The size of the vector equals image_points_right.size() - pair_count
    std::vector<nie::Isometry3qd>* p_extrinsics_right,
    nie::Isometry3qd* p_baseline,
    std::vector<std::vector<Eigen::Vector2f>>* p_residuals_left,
    std::vector<std::vector<Eigen::Vector2f>>* p_residuals_right,
    Eigen::MatrixXd* p_cov_intrinsics_left,
    Eigen::MatrixXd* p_cov_intrinsics_right,
    std::vector<Eigen::Matrix<double, 6, 6>>* p_cov_extrinsics_left,
    std::vector<Eigen::Matrix<double, 6, 6>>* p_cov_extrinsics_right,
    Eigen::Matrix<double, 6, 6>* p_cov_baseline,
    double* p_var_residuals,
    const unsigned int num_threads) {
    const std::size_t image_count_left = image_points_left.size();
    const std::size_t image_count_right = image_points_right.size();
    const std::size_t extrinsics_count_left = image_count_left;
    const std::size_t extrinsics_count_right = image_count_right - pair_count;
    const std::size_t extrinsics_count = image_count_left + extrinsics_count_right;

    assert(image_count_left >= pair_count);
    assert(image_count_right >= pair_count);

    // ****************************************************************************************************************
    // SOLVER
    // ****************************************************************************************************************

    std::unique_ptr<CalibrationCostDefinition> factory = CalibrationCostDefinitionFactory::Create(parameters);
    const std::size_t object_point_count = object_points.size();
    const std::size_t parameter_count_intrinsics = factory->GetParameterCountIntrinsics();

    Eigen::Matrix3d K;
    std::vector<Eigen::Vector3d> translations;
    std::vector<Eigen::Matrix3d> rotations;

    // Get initial estimate for the left camera.
    // The baseline will be determined with respect to the left camera.
    EstimateCameraParametersZhangClosedForm_Eigen(object_points, image_points_left, &K, &translations, &rotations);

    std::vector<double>& intrinsics_left = *p_intrinsics_left;
    intrinsics_left = factory->InitializeIntrinsicsFromK(K);

    std::vector<nie::Isometry3qd>& extrinsics_left = *p_extrinsics_left;
    extrinsics_left.resize(extrinsics_count_left);

    for (std::size_t i = 0; i < extrinsics_count_left; ++i) {
        extrinsics_left[i].translation() = translations[i];
        extrinsics_left[i].rotation() = Eigen::Quaterniond{rotations[i]};
        extrinsics_left[i].Inverse();
    }

    // Get initial estimate for the right camera.
    // Note: all output values are overwritten.
    // The rotation of the right camera should roughly be the same as the left one.
    // We just assume an initial rotation delta of 0 for the baseline.
    EstimateCameraParametersZhangClosedForm_Eigen(object_points, image_points_right, &K, &translations, &rotations);

    std::vector<double>& intrinsics_right = *p_intrinsics_right;
    intrinsics_right = factory->InitializeIntrinsicsFromK(K);

    Isometry3qd& baseline = *p_baseline;
    baseline = Isometry3qd::Identity();

    for (std::size_t i = 0; i < pair_count; ++i) {
        // The baseline is considered the same as the right camera relative to the left one.
        // In the Local to Global or Camera to World, where global/world means left, we get the
        // relative transformation as:
        //      T_l^-1 * T_r
        // Since we only use the translation part, we transform t_right using the left_inverse camera.
        Isometry3qd inversed_left = extrinsics_left[i].Inversed();
        Eigen::Vector3d t_right = rotations[i].transpose() * -translations[i];
        baseline.translation() += inversed_left * t_right;
    }

    baseline.translation() /= static_cast<double>(pair_count);

    std::vector<nie::Isometry3qd>& extrinsics_right = *p_extrinsics_right;
    extrinsics_right.resize(extrinsics_count_right);

    for (std::size_t i = 0; i < extrinsics_count_right; ++i) {
        std::size_t j = i + pair_count;
        extrinsics_right[i].translation() = translations[j];
        extrinsics_right[i].rotation() = Eigen::Quaterniond{rotations[j]};
        extrinsics_right[i].Inverse();
    }

    ceres::Problem problem;
    ceres::LocalParameterization* quaternion_parameterization = new ceres::EigenQuaternionParameterization;

    for (std::size_t i = 0; i < extrinsics_count_left; ++i) {
        for (std::size_t j = 0; j < object_point_count; ++j) {
            AddResidualBlock(
                factory, object_points[j], image_points_left[i][j], &problem, &intrinsics_left, &extrinsics_left[i]);
            problem.SetParameterization(extrinsics_left[i].rotation().coeffs().data(), quaternion_parameterization);
        }
    }

    for (std::size_t i = 0; i < pair_count; ++i) {
        for (std::size_t j = 0; j < object_point_count; ++j) {
            AddResidualBlock(
                factory,
                object_points[j],
                image_points_right[i][j],
                &problem,
                &intrinsics_right,
                &extrinsics_left[i],
                &baseline);
            // No quaternion parameterization (see previous loop).
        }
    }

    for (std::size_t i = 0; i < extrinsics_count_right; ++i) {
        std::size_t k = i + pair_count;
        for (std::size_t j = 0; j < object_point_count; ++j) {
            AddResidualBlock(
                factory, object_points[j], image_points_right[k][j], &problem, &intrinsics_right, &extrinsics_right[i]);
            problem.SetParameterization(extrinsics_right[i].rotation().coeffs().data(), quaternion_parameterization);
        }
    }

    ceres::Solver::Options options_solver;
    options_solver.linear_solver_type = ceres::LinearSolverType::SPARSE_NORMAL_CHOLESKY;
    options_solver.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
    options_solver.num_threads = num_threads;

    ceres::Solver::Summary summary;
    ceres::Solve(options_solver, &problem, &summary);

    // ****************************************************************************************************************
    // VARIANCE COVARIANCE
    // ****************************************************************************************************************

    CalculateResidualsCamera(
        object_points,
        image_points_left,
        extrinsics_left,
        ResidualCalculator(factory, intrinsics_left),
        p_residuals_left);

    CalculateResidualsCamera(
        object_points,
        image_points_right,
        extrinsics_left,
        ResidualCalculatorWithBaseline(factory, intrinsics_right, baseline),
        0,
        pair_count,
        p_residuals_right);

    CalculateResidualsCamera(
        object_points,
        image_points_right,
        extrinsics_right,
        ResidualCalculator(factory, intrinsics_right),
        pair_count,
        pair_count + extrinsics_right.size(),
        p_residuals_right);

    // We estimate the input input_variance (no a priori standard deviation). This will be used to scale the output
    // variances.
    double variance_residuals = 0.0;

    for (std::size_t i = 0; i < image_count_left; ++i) {
        for (std::size_t j = 0; j < object_point_count; ++j) {
            variance_residuals += (*p_residuals_left)[i][j].squaredNorm();
        }
    }

    for (std::size_t i = 0; i < image_count_right; ++i) {
        for (std::size_t j = 0; j < object_point_count; ++j) {
            variance_residuals += (*p_residuals_right)[i][j].squaredNorm();
        }
    }

    // This calculates the average squared error or variance while taking into account the degrees of freedom of the
    // problem.
    std::size_t const degrees_of_freedom_extrinsics = 6;  // Not 7, degrees of freedom != parameter count

    variance_residuals = variance_residuals /
                         static_cast<double>(
                             ((image_count_left + image_count_right) * object_point_count) -
                             ((extrinsics_count + 1) * degrees_of_freedom_extrinsics + parameter_count_intrinsics * 2));

    ceres::Covariance::Options options_covariance;
    options_covariance.num_threads = num_threads;

    ceres::Covariance covariance(options_covariance);

    std::vector<std::pair<const double*, const double*>> covariance_blocks{2 + (extrinsics_count + 1) * 3};
    covariance_blocks[0] = {intrinsics_left.data(), intrinsics_left.data()};
    covariance_blocks[1] = {intrinsics_right.data(), intrinsics_right.data()};

    std::size_t index = 2;

    for (std::size_t i = 0; i < extrinsics_count_left; ++i, index += 3) {
        SetCovarianceBlocksIsometry(extrinsics_left[i], index, &covariance_blocks);
    }

    for (std::size_t i = 0; i < extrinsics_count_right; ++i, index += 3) {
        SetCovarianceBlocksIsometry(extrinsics_right[i], index, &covariance_blocks);
    }

    SetCovarianceBlocksIsometry(baseline, index, &covariance_blocks);

    covariance.Compute(covariance_blocks, &problem);

    GetCovarianceBlockIntrinsics(covariance, intrinsics_left, variance_residuals, p_cov_intrinsics_left);
    GetCovarianceBlockIntrinsics(covariance, intrinsics_right, variance_residuals, p_cov_intrinsics_right);
    GetCovarianceBlocksIsometries(covariance, extrinsics_left, variance_residuals, p_cov_extrinsics_left);
    GetCovarianceBlocksIsometries(covariance, extrinsics_right, variance_residuals, p_cov_extrinsics_right);
    GetCovarianceMatrix(covariance, baseline, p_cov_baseline);
    (*p_cov_baseline) *= variance_residuals;
    *p_var_residuals = variance_residuals;
}

}  // namespace nie
