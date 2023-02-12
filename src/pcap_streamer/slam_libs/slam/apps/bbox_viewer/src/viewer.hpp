/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <nie/drawing/drawing.hpp>

class Viewer {
    // The addPointCloud function of the pcl viewer does not have the same interface for the point types with and
    // without intensity, so templating is not that easy out of the box.
    using Cloud = nie::Cloud<pcl::PointXYZ>;

public:
    explicit Viewer(std::string const& name, std::vector<nie::PoseBbox> const& bounds, std::vector<Cloud> const& clouds)
        : pcl_viewer_(name), bounds_(bounds), clouds_(clouds) {
        Init();
    }

    void View() { pcl_viewer_.View(); }

private:
    void Init() {
        pcl_viewer_.Reset();

        // This is just a choice. Plotting of all objects will just substract this origin.
        Eigen::Vector3d const origin = Eigen::Vector3d::Zero();

        AddCoordinateSystem(origin, bounds_, &pcl_viewer_.viewer);
        DrawOrientedBounds(origin, bounds_, &pcl_viewer_.viewer);
        std::for_each(clouds_.cbegin(), clouds_.cend(), [this, &origin](auto const& c) { AddCloud(c, origin); });
    }

    void AddCloud(Cloud const& cloud, Eigen::Vector3d const& origin) {
        std::string const cloud_name = CloudNamer();

        auto cloud_ptr = cloud.point_cloud().makeShared();
        Eigen::Vector3f const offset = (cloud.bounds().origin().translation() - origin).cast<float>();
        cloud_ptr->sensor_origin_ = {offset.x(), offset.y(), offset.z(), 0.f};

        pcl_viewer_.viewer.addPointCloud(cloud_ptr, cloud_name);
        pcl_viewer_.viewer.setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_name);
    }

    std::string CloudNamer() { return std::string{"cloud_"} + std::to_string(cloud_id_++); }

    nie::PclViewer pcl_viewer_;
    std::vector<nie::PoseBbox> const& bounds_;
    std::vector<Cloud> const& clouds_;

    std::size_t cloud_id_ = 0;
};
