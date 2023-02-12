/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_CERES_COVARIANCE_ISOMETRY_HPP
#define NIE_CV_CERES_COVARIANCE_ISOMETRY_HPP

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <nie/core/geometry/covariance.hpp>
#include <nie/core/geometry/isometry3.hpp>

namespace nie {

inline void SetCovarianceBlocksIsometry(
    nie::Isometry3qd const& extrinsics,
    std::size_t const& index,
    std::vector<std::pair<const double*, const double*>>* p_covariance_blocks) {
    std::vector<std::pair<const double*, const double*>>& covariance_blocks = *p_covariance_blocks;
    double const* t = extrinsics.translation().data();
    double const* r = extrinsics.rotation().coeffs().data();
    covariance_blocks[index + 0] = {t, t};
    covariance_blocks[index + 1] = {t, r};
    covariance_blocks[index + 2] = {r, r};
}

inline void GetCovarianceMatrix(
    ceres::Covariance const& covariance,
    nie::Isometry3qd const& pose,
    Eigen::Matrix<double, 6, 6>* covariance_isometry) {
    double const* const t = pose.translation().data();
    double const* const r = pose.rotation().coeffs().data();
    // Temp buffer
    std::vector<double> covariance_block_data(16);

    // Covariance part is symmetric so there are no issues with row/col major matrices.
    CHECK(covariance.GetCovarianceBlock(t, t, covariance_block_data.data())) << "Invalid GetCovarianceBlock()";
    Eigen::Map<Eigen::Matrix3d> covariance_block_00(covariance_block_data.data());
    covariance_isometry->block<3, 3>(0, 0) = covariance_block_00;

    // The quaternion w part is dropped.
    // Note that covariance.GetCovarianceBlock outputs a row-major matrix while Eigen defaults to Column.
    // The difference between the two representations is the transpose, so the covariance is queried as the transpose.
    // Reading t x r = 3x4 while mapping it as a 4x3. Each row read becomes a column.
    CHECK(covariance.GetCovarianceBlock(t, r, covariance_block_data.data())) << "Invalid GetCovarianceBlock()";
    Eigen::Map<Eigen::Matrix<double, 4, 3>> covariance_block_10(covariance_block_data.data());
    covariance_isometry->block<3, 3>(3, 0) = covariance_block_10.block<3, 3>(0, 0);
    covariance_isometry->block<3, 3>(0, 3) = covariance_block_10.block<3, 3>(0, 0).transpose();

    // The quaternion w part is dropped.
    // Covariance part is symmetric so there are no issues with row/col major matrices.
    Eigen::Map<Eigen::Matrix4d> covariance_block_11(covariance_block_data.data());
    CHECK(covariance.GetCovarianceBlock(r, r, covariance_block_data.data())) << "Invalid GetCovarianceBlock()";
    covariance_isometry->block<3, 3>(3, 3) = covariance_block_11.block<3, 3>(0, 0);

    // TODO(jbr): Remove later, kept for debugging
    //    std::cout << std::endl;
    //    for (std::size_t i = 0; i <16; ++i)
    //    {
    //        std::cout << covariance_block_data[i] << " ";
    //
    //        if  ((i+1) % 4 == 0)
    //        {
    //            std::cout << std::endl;
    //        }
    //    }
    //    std::cout << std::endl;
    //    std::cout << constraint->information << std::endl;
}

///@brief Get the information matrix from the ceres problem covariance.
inline void GetInformationMatrix(
    ceres::Covariance const& covariance,
    nie::Isometry3qd const& pose,
    Eigen::Matrix<double, 6, 6>* information_isometry) {
    GetCovarianceMatrix(covariance, pose, information_isometry);
    CovarianceToInformation(*information_isometry, information_isometry);
}

}  // namespace nie

#endif  // NIE_CV_CERES_COVARIANCE_ISOMETRY_HPP
