/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include "cloud.hpp"

#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/registration.h>

namespace nie {

template <typename PointSource, typename PointTarget, typename Scalar = float>
class Icp : public pcl::IterativeClosestPoint<PointSource, PointTarget, Scalar> {
public:
    using pcl::Registration<PointSource, PointTarget, Scalar>::correspondences_;

    pcl::CorrespondencesConstPtr getCorrespondences() { return correspondences_; }
};

template <typename PointT>
class CloudMatcher {
private:
    using CloudT = Cloud<PointT>;

public:
    struct Parameters {
        //
        // All parameters can be set to -1, then no value will be set and the
        // algorithm's defaults will be used.
        //

        /// @brief The transformation epsilon in order for an optimization to be considered as having converged.
        /// @details The transformation epsilon equals the maximum allowable difference between two consecutive
        /// transformations.
        double transformation_eps = 1.e-9;

        /// @brief The maximum allowed Euclidean error between two consecutive steps in the ICP loop, before the
        /// algorithm is considered to have converged.
        /// @details The error is estimated as the sum of the differences between correspondences in an Euclidean sense,
        /// divided by the number of correspondences.
        double euclidean_fitness_eps = -1.;

        /// @brief The maximum distance threshold between two correspondent points in source <-> target.
        double max_correspondence_distance = .5;

        /// @brief The maximum number of iterations the internal optimization should run for.
        int max_iterations = 30;

        /// @brief The inlier distance threshold for the internal RANSAC outlier rejection loop.
        /// @details The distance threshold for RANSAC where the the method considers a point to be an inlier, if the
        /// distance between the target data index and the transformed source index is smaller than the given inlier
        /// distance threshold. The default value used by the algorithm will be 0.05m.
        double ransac_outlier_rejection_threshold = .05;

        /// @brief The maximum number of RANSAC iterations.
        int ransac_max_iterations = 0;
    };

    explicit CloudMatcher(std::vector<Parameters> parameters = {Parameters()})
        : parameters_vector_(parameters),
          reg_(),
          rejector_ransac_(new pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>()) {
        reg_.addCorrespondenceRejector(rejector_ransac_);
    }

    /// @brief Finds the rigid point transformation from point cloud B to A. Points should be transformed as
    /// points_a = T * points_b.
    /// Note that the transformation should be initialized
    bool Match(
            CloudT const& cloud_a, CloudT const& cloud_b, nie::Isometry3qd* T_ab, CloudT* cloud_transformed = nullptr);

    // Query = b, Match = a
    pcl::CorrespondencesConstPtr GetCorrespondences() { return reg_.getCorrespondences(); }

    // Fitness score as used by ICP
    double GetFitnessScore() { return reg_.getFitnessScore(); }

private:
    void ResetParams(Parameters const& parameters);

    std::vector<Parameters> parameters_vector_;

    Icp<PointT, PointT> reg_;
    typename pcl::registration::CorrespondenceRejectorSampleConsensus<PointT>::Ptr rejector_ransac_;
};

}  // namespace nie

#include "cloud_matcher.inl"
