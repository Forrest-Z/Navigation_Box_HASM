/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <pcl/common/transforms.h>

namespace nie {

template <typename PointT>
PoseBbox GetTotalExtent(std::vector<Cloud<PointT>> const& clouds, Eigen::Vector3d const& reference) {
    return GetTotalExtent(
            clouds.cbegin(), clouds.cend(), reference, [](Cloud<PointT> const& cloud) { return cloud.bounds(); });
}

template <typename Iterator, typename Getter>
PoseBbox GetTotalExtent(Iterator begin, Iterator end, Eigen::Vector3d const& reference, Getter getter) {
    Bboxf extent = Bboxf::InverseMaxBoundingBox();
    for (auto it = begin; it != end; ++it) {
        auto bounds = getter(*it).CopyWithOrigin(reference);
        extent |= bounds.bbox();
    }
    return {reference, extent};
}

template <typename PointT>
PoseBbox GetOverlappingExtent(std::vector<Cloud<PointT>> const& clouds, Eigen::Vector3d const& reference) {
    Bboxf extent = Bboxf::MaxBoundingBox();

    for (auto& pc : clouds) {
        auto bounds = pc.bounds().CopyWithOrigin(reference);
        extent &= bounds.bbox();
    }

    return {reference, extent};
}

template <typename PointT>
PoseBbox CalculateOrientedBounds(pcl::PointCloud<PointT> const& cloud) {
    Eigen::Vector4d center_of_mass;
    pcl::compute3DCentroid(cloud, center_of_mass);
    Eigen::Matrix3d covariance;
    pcl::computeCovarianceMatrixNormalized(cloud, center_of_mass, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance, Eigen::ComputeEigenvectors);

    // Make sure the eigen vectors are orthogonal and more importantly have correct handedness
    Eigen::Matrix3d R = eigen_solver.eigenvectors();
    R.col(2) = R.col(0).cross(R.col(1));

    // The oriented origin with respect to the cloud origin
    nie::Isometry3qd origin(center_of_mass.head<3>(), Eigen::Quaterniond(R));

    // Get initial bounds using the oriented origin
    return {origin, nie::Bboxf::Create(cloud, origin)};
}

template <typename PointT>
PoseBbox CalculateOrientedBounds(nie::Cloud<PointT> const& cloud) {
    // Get initial bounds based on the point in the cloud only
    PoseBbox bounds = CalculateOrientedBounds(cloud.point_cloud());
    // Update the bounds with respect to the world, incorporating the cloud origin
    bounds.origin().translation() += cloud.origin().translation();
    return bounds;
}

template <typename PointT>
void Filter(nie::Isometry3qd const& T, Bboxf const& box, nie::Cloud<PointT>* filtered) {
    // Matrix form for cheaper point transformation calculations
    nie::Isometry3mf const T_m = nie::Isometry3md(T.translation(), T.rotation().toRotationMatrix()).cast<float>();
    auto filter = [&box, &T_m](PointT const& p) -> bool {
        PointT b;
        b.getVector3fMap() = T_m.Transform(p.template getVector3fMap());
        return !box.Contains(b);
    };
    filtered->FilterPoints(filter);
}

template <typename PointT>
typename pcl::search::KdTree<PointT>::Ptr GetKdTree(nie::Cloud<PointT> const& cloud, bool sorted) {
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>(sorted));
    tree->setInputCloud(cloud.point_cloud_ptr());
    return tree;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> GetPointClouds(std::vector<nie::Cloud<PointT>> const& clouds) {
    std::vector<typename pcl::PointCloud<PointT>::Ptr> point_clouds(clouds.size());
    for (std::size_t i = 0; i < clouds.size(); ++i) {
        point_clouds[i] = clouds[i].point_cloud_ptr();
    }
    return point_clouds;
}

template <typename Iterator, typename CloudGetter>
std::vector<PoseBbox> GetBounds(Iterator begin, Iterator end, CloudGetter cloud_getter) {
    std::vector<PoseBbox> bounds{};
    bounds.reserve(std::distance(begin, end));
    for (auto it = begin; it != end; ++it) {
        bounds.push_back(cloud_getter(*it).bounds());
    }
    return bounds;
}

template <typename PointT>
std::vector<PoseBbox> GetBounds(std::vector<Cloud<PointT>> const& clouds) {
    return GetBounds(clouds.cbegin(), clouds.cend(), [](Cloud<PointT> const& p) { return p; });
}

template <typename PointT>
void TransformCloud(nie::Isometry3qd const& T, nie::Cloud<PointT>* cloud) {
    typename pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
    pcl::transformPointCloud(cloud->point_cloud(), *transformed, T.ToTransform().matrix());
    std::swap(transformed, cloud->point_cloud_ptr());
    cloud->bounds().UpdateBbox(cloud->point_cloud());
}

template <typename PointT>
nie::Cloud<PointT> TransformCloud(nie::Cloud<PointT> const& cloud, nie::Isometry3qd const& T) {
    nie::Cloud<PointT> cloud_transformed = cloud.CopyMetadataOnly();
    pcl::transformPointCloud(cloud.point_cloud(), cloud_transformed.point_cloud(), T.ToTransform().matrix());
    cloud_transformed.bounds().UpdateBbox(cloud_transformed.point_cloud());
    return cloud_transformed;
}

template <typename PointT>
nie::Cloud<PointT> TransformCloud(
        nie::Cloud<PointT> const& cloud, nie::Isometry3qd const& T, nie::Isometry3qd const& origin) {
    nie::Cloud<PointT> cloud_transformed = TransformCloud(cloud, T);
    cloud_transformed.bounds().origin() = origin;
    return cloud_transformed;
}

}  // namespace nie
