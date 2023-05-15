/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <random>
#include <thread>

#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <nie/lidar/helper_cloud.hpp>

namespace nie {

namespace detail {

template <typename PointT>
void AddClouds(
        Eigen::Vector3d const& origin,
        std::vector<typename pcl::PointCloud<PointT>::Ptr> const& clouds,
        std::vector<PoseBbox> const& bounds,
        std::vector<typename pcl::visualization::PointCloudColorHandler<PointT>::Ptr> const& color_handlers,
        // This is the only one allowed to be empty (for now).
        std::vector<pcl::PointCloud<pcl::Normal>::Ptr> const& normals,
        std::vector<std::string> const& cloud_names,
        std::vector<std::string> const& normals_names,
        pcl::visualization::PCLVisualizer* viewer) {
    CHECK(clouds.size() > 0);
    CHECK(color_handlers.size() == clouds.size()) << "Size of color_handlers should be equal to the size of clouds.";
    CHECK(normals.empty() || normals.size() == clouds.size())
            << "Size of normals should be 0 or equal to the size of clouds.";

    for (std::size_t index = 0; index < clouds.size(); ++index) {
        auto const& cloud = *clouds[index];

        std::string cloud_name = cloud_names[index];
        auto const& cloud_bounds = bounds[index];

        // Actually copies data
        auto cloud_ptr = cloud.makeShared();
        Eigen::Vector3f offset = (cloud_bounds.origin().translation() - origin).cast<float>();
        cloud_ptr->sensor_origin_ = {offset.x(), offset.y(), offset.z(), 0.f};

        auto& color_handler = *color_handlers[index];
        color_handler.setInputCloud(cloud_ptr);
        viewer->addPointCloud(cloud_ptr, color_handler, cloud_name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_name);

        if (!normals.empty()) {
            viewer->addPointCloudNormals<PointT, pcl::Normal>(cloud_ptr, normals[index], 1, 0.5, normals_names[index]);
        }
    }
}

inline void AddCoordinateSystem(
        Eigen::Vector3d const& origin,
        std::vector<PoseBbox> const& bounds_list,
        pcl::visualization::PCLVisualizer* viewer) {
    Eigen::Vector3d average = {0.0, 0.0, 0.0};
    for (auto const& bounds : bounds_list) {
        Eigen::Vector3d w = bounds.origin() * bounds.bbox().Center().cast<double>();
        average += w - origin;
    }

    float scale = 10.0f;

    if (!bounds_list.empty()) {
        average /= static_cast<double>(bounds_list.size());
        scale = bounds_list.front().bbox().Range().array().minCoeff() * 0.5f;
    }

    pcl::PointXYZ center = MakePointXYZ(average);
    viewer->addCoordinateSystem(scale, center.x, center.y, center.z);
}

}  // namespace detail

template <typename PointT>
void AddClouds(
        std::vector<Cloud<PointT>> const& clouds,
        PointCloudColorHandlerPtr<PointT> const& handler,
        std::vector<std::string> const& names,
        nie::PclViewer* viewer) {
    detail::AddClouds<PointT>(
            clouds[0].origin().translation(),
            nie::GetPointClouds(clouds),
            nie::GetBounds(clouds),
            std::vector<PointCloudColorHandlerPtr<PointT>>(clouds.size(), handler),
            {},
            names,
            GenerateNames(detail::kNormalsPrefix, clouds.size()),
            &viewer->viewer);
}

template <typename PointT>
void AddClouds(
        Eigen::Vector3d const& origin,
        std::vector<Cloud<PointT>> const& clouds,
        std::vector<PointCloudColorHandlerPtr<PointT>> const& handlers,
        std::vector<std::string> const& names,
        nie::PclViewer* viewer) {
    detail::AddClouds<PointT>(
            origin,
            nie::GetPointClouds(clouds),
            nie::GetBounds(clouds),
            handlers,
            {},
            names,
            GenerateNames(detail::kNormalsPrefix, clouds.size()),
            &viewer->viewer);
}

template <typename PointT>
void AddClouds(
        std::vector<Cloud<PointT>> const& clouds,
        std::vector<PointCloudColorHandlerPtr<PointT>> const& handlers,
        std::vector<std::string> const& names,
        nie::PclViewer* viewer) {
    AddClouds<PointT>(clouds[0].origin().translation(), clouds, handlers, names, viewer);
}

template <typename Iterator, typename CloudGetter>
void AddCoordinateSystem(Iterator begin, Iterator end, nie::PclViewer* viewer, CloudGetter cloud_getter) {
    AddCoordinateSystem(
            cloud_getter(*begin).origin().translation(), nie::GetBounds(begin, end, cloud_getter), &viewer->viewer);
}

template <typename PointT>
void AddCoordinateSystem(std::vector<Cloud<PointT>> const& clouds, nie::PclViewer* viewer) {
    AddCoordinateSystem(clouds[0].origin().translation(), nie::GetBounds(clouds), &viewer->viewer);
}

template <typename PointT>
void DrawCloudsWithCoordinateSystem(
        Cloud<PointT> const& cloud_a, Cloud<PointT> const& cloud_b, std::string const& name) {
    nie::PclViewer viewer(name);
    AddClouds(cloud_a, cloud_b, &viewer);
    std::vector<Cloud<PointT>> clouds = {cloud_a, cloud_b};
    AddCoordinateSystem(clouds, &viewer);
    viewer.View();
}

}  // namespace nie
