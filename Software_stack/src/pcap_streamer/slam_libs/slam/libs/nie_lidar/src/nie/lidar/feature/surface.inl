/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

// Same as in GICP code
template <typename PointT>
void CalculatePointCovariances(
    pcl::search::KdTree<PointT> const& kdtree,
    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>* p_covariances) {
    auto const& cloud = kdtree.getInputCloud();

    std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>& covariances = *p_covariances;
    covariances.resize(cloud->size());

    // The number of neighbors used for covariances computation. default: 20
    constexpr int k_correspondences = 30;
    // Search radius (different than the original pcl implementation which is NN).
    constexpr double search_radius = 1.0;
    // The epsilon constant for gicp paper; this is NOT the convergence tolerence default: 0.001
    // constexpr double gicp_epsilon = 0.001;

    Eigen::Vector3d mean;
    std::vector<int> nn_indecies;
    nn_indecies.reserve(k_correspondences);
    std::vector<float> nn_dist_sq;
    nn_dist_sq.reserve(k_correspondences);

    auto points_iterator = cloud->begin();
    auto matrices_iterator = covariances.begin();
    for (; points_iterator != cloud->end(); ++points_iterator, ++matrices_iterator) {
        auto const& query_point = *points_iterator;
        Eigen::Matrix3d& cov = *matrices_iterator;
        // Zero out the cov and mean
        cov.setZero();
        mean.setZero();

        // TODO (jbr) Radius search?
        // Search for the K nearest neighbours
        // kdtree.nearestKSearch(query_point, k_correspondences, nn_indecies, nn_dist_sq);
        // int correspondences = k_correspondences;
        int correspondences =
            kdtree.radiusSearch(query_point, search_radius, nn_indecies, nn_dist_sq, k_correspondences);

        // Find the covariance matrix
        for (int j = 0; j < correspondences; j++) {
            auto const& pt = (*cloud)[nn_indecies[j]];

            mean[0] += pt.x;
            mean[1] += pt.y;
            mean[2] += pt.z;

            cov(0, 0) += pt.x * pt.x;

            cov(1, 0) += pt.y * pt.x;
            cov(1, 1) += pt.y * pt.y;

            cov(2, 0) += pt.z * pt.x;
            cov(2, 1) += pt.z * pt.y;
            cov(2, 2) += pt.z * pt.z;
        }

        mean /= static_cast<double>(correspondences);
        // Get the actual covariance
        for (int k = 0; k < 3; k++)
            for (int l = 0; l <= k; l++) {
                cov(k, l) /= static_cast<double>(correspondences);
                cov(k, l) -= mean[k] * mean[l];
                cov(l, k) = cov(k, l);
            }

        // TODO(jbr): Should we?
        // Compute the SVD (covariance matrix is symmetric so U = V')
        //        Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU);
        //        cov.setZero ();
        //        Eigen::Matrix3d U = svd.matrixU ();
        //        // Reconstitute the covariance matrix with modified singular values using the column     // vectors in
        //        V. for(int k = 0; k < 3; k++) {
        //            Eigen::Vector3d col = U.col(k);
        //            double v = 1.; // biggest 2 singular values replaced by 1
        //            if(k == 2)   // smallest singular value replaced by gicp_epsilon
        //                v = gicp_epsilon;
        //            cov+= v * col * col.transpose();
        //        }
    }
}
