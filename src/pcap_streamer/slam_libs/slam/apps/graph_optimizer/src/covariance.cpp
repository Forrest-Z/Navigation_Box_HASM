/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "covariance.hpp"

#include <nie/cv/ceres/covariance_isometry.hpp>
#include <utility>
#include <vector>

// There are three covariance blocks per pose. translation x translation, translation x rotation, rotation x rotation.
std::size_t BlockCountPose(nie::io::PoseCollection const& pose_estimates) { return pose_estimates.poses.size() * 3; }

std::size_t BlockCountObjt(nie::io::ObjectCollection const& objt_estimates) { return objt_estimates.objects.size(); }

bool GetInformationMatrices(
    nie::io::PoseCollection* p_pose_estimates,
    nie::io::ObjectCollection* p_objt_estimates,
    ceres::Problem* problem,
    std::uint32_t const& num_threads) {
    // Pre size.
    std::vector<std::pair<const double*, const double*> > covariance_blocks(
        BlockCountPose(*p_pose_estimates) + BlockCountObjt(*p_objt_estimates));

    for (std::size_t i = 0; i < p_pose_estimates->poses.size(); ++i) {
        // See BlockCountPose for the * 3.
        SetCovarianceBlocksIsometry(p_pose_estimates->poses[i].isometry, i * 3, &covariance_blocks);
    }

    std::size_t offset = BlockCountPose(*p_pose_estimates);
    for (std::size_t i = 0; i < p_objt_estimates->objects.size(); ++i) {
        auto const& o = p_objt_estimates->objects[i].position;
        covariance_blocks[offset + i].first = o.data();
        covariance_blocks[offset + i].second = o.data();
    }

    ceres::Covariance::Options options;
    options.num_threads = num_threads;

    ceres::Covariance covariance(options);
    bool success = covariance.Compute(covariance_blocks, problem);

    if (success) {
        LOG(INFO) << "Covariance computation success.";
        for (auto& p : p_pose_estimates->poses) {
            // NOTE: If the isometry was set as constant / fixed, it will have an information matrix with value
            // double::nan as its elements
            nie::GetInformationMatrix(covariance, p.isometry, &p.information);
        }

        for (auto& o : p_objt_estimates->objects) {
            Eigen::Matrix3d cov_object;
            CHECK(covariance.GetCovarianceBlock(o.position.data(), o.position.data(), cov_object.data()))
                << "Invalid GetCovarianceBlock()";
            nie::CovarianceToInformation(cov_object, &o.information);
        }
    } else {
        LOG(ERROR) << "Covariance computation failed.";
    }

    return success;
}
