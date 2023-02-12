/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <glog/logging.h>

namespace nie {

template <typename PointT>
bool CloudMatcher<PointT>::Match(
        CloudT const& cloud_a, CloudT const& cloud_b, nie::Isometry3qd* T_ab, CloudT* cloud_result) {
    // Sanity check: generalized ICP requires at least 20 points
    if (cloud_a.point_cloud().size() < 20 or cloud_b.point_cloud().size() < 20) {
        VLOG(3) << "CloudMatcher::Match(): Not matching. Minimum amount of points required in both clouds equals 20.";
        return false;
    }

    // Upload the clouds
    reg_.setInputSource(cloud_b.point_cloud_ptr());  // Input to be aligned with the target
    reg_.setInputTarget(cloud_a.point_cloud_ptr());  // Target cloud

    CloudT result = cloud_a.CopyMetadataOnly();
    Eigen::Matrix4f T_icp = T_ab->ToTransform().matrix().template cast<float>();

    for (std::size_t i = 0; i < parameters_vector_.size(); ++i) {
        VLOG(3) << "Alignment step " << (i + 1) << " of " << parameters_vector_.size();
        ResetParams(parameters_vector_[i]);
        reg_.align(result.point_cloud(), T_icp);
        T_icp = reg_.getFinalTransformation();
    }
    *T_ab = nie::Isometry3qd(T_icp.cast<double>());

    // Check the result and update the output variable
    VLOG(3) << "ICP has" << ((reg_.hasConverged()) ? " " : " not ") << "converged with a score of "
            << reg_.getFitnessScore() << ".";

    if (cloud_result != nullptr) {
        *cloud_result = result;
    }

    return reg_.hasConverged();
}

template <typename PointT>
void CloudMatcher<PointT>::ResetParams(Parameters const& parameters) {
    if (parameters.transformation_eps != -1) {
        reg_.setTransformationEpsilon(parameters.transformation_eps);
    }
    if (parameters.euclidean_fitness_eps != -1) {
        reg_.setEuclideanFitnessEpsilon(parameters.euclidean_fitness_eps);
    }
    if (parameters.max_correspondence_distance != -1) {
        reg_.setMaxCorrespondenceDistance(parameters.max_correspondence_distance);
    }
    if (parameters.max_iterations != -1) {
        reg_.setMaximumIterations(parameters.max_iterations);
    }

    if (parameters.ransac_outlier_rejection_threshold != -1) {
        rejector_ransac_->setInlierThreshold(parameters.ransac_outlier_rejection_threshold);
    }
    if (parameters.ransac_max_iterations != -1) {
        rejector_ransac_->setMaximumIterations(parameters.ransac_max_iterations);
    }
}

}  // namespace nie
