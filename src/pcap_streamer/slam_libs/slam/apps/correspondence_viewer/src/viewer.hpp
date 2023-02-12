/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef CORRESPONDENCE_VIEWER_VIEWER_HPP
#define CORRESPONDENCE_VIEWER_VIEWER_HPP

#include <pcl/point_types_conversion.h>
#include <nie/drawing/drawing.hpp>
#include <nie/drawing/drawing_color_handlers.hpp>

namespace nie {

template <typename PointT>
class Viewer {
public:
    explicit Viewer(std::string const& name) : pcl_viewer_(name), cloud_(), correspondences_() {}

    pcl::visualization::PCLVisualizer& viewer() { return pcl_viewer_.viewer; }

    void View() { pcl_viewer_.View(); }

    void Update(nie::Cloud<PointT> const& cloud, std::vector<nie::Correspondence> const& corresps) {
        correspondences_ = CreateCorrespondencesCloud(cloud, corresps);
        CreateScene();
    }

private:
    nie::Cloud<pcl::PointXYZRGB> CreateCorrespondencesCloud(
            Cloud<PointT> const& cloud, std::vector<nie::Correspondence> const& correspondences) {
        nie::Cloud<pcl::PointXYZRGB> colored_cloud = cloud.template CopyMetadataOnly<pcl::PointXYZRGB>();
        colored_cloud.point_cloud().points.reserve(cloud.point_cloud().points.size());

        float min = std::numeric_limits<float>::max();
        float max = std::numeric_limits<float>::lowest();
        for (nie::Correspondence const& c : correspondences) {
            min = std::min(min, c.distance);
            max = std::max(max, c.distance);
        }

        std::size_t index_index = 0;
        for (std::size_t index = 0; index < cloud.point_cloud().size(); ++index) {
            pcl::PointXYZRGB point;

            if (!correspondences.empty() && index == correspondences[index_index].index_b) {
                float ratio = (correspondences[index_index].distance - min) / (max - min);
                point = nie::MakePointXYZRGB((1.f - ratio) * 120.f, 1.f);
                ++index_index;
            } else {
                point.r = 0;
                point.g = 0;
                point.b = 255;
            }

            point.x = cloud.point_cloud()[index].x;
            point.y = cloud.point_cloud()[index].y;
            point.z = cloud.point_cloud()[index].z;

            colored_cloud.point_cloud().points.push_back(point);
        }

        return colored_cloud;
    }

    void CreateScene() {
        pcl_viewer_.Reset();

        nie::AddClouds<PointT>({cloud_}, nie::GetColorHandlerReferenceCloud<PointT>(), {"cloud"}, &pcl_viewer_);
        AddCorrespondences(correspondences_, "correspondences");

        nie::AddCoordinateSystem<PointT>({cloud_}, &pcl_viewer_);
    }

    void AddCorrespondences(Cloud<pcl::PointXYZRGB> const& cloud, std::string const& name) {
        std::vector<PointCloudColorHandlerPtr<pcl::PointXYZRGB>> handlers;
        handlers.emplace_back(
                new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(cloud.point_cloud_ptr()));
        nie::AddClouds<pcl::PointXYZRGB>({cloud}, handlers, {name}, &pcl_viewer_);
        pcl_viewer_.viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);
    }

    nie::PclViewer pcl_viewer_;

    nie::Cloud<PointT> cloud_;
    nie::Cloud<pcl::PointXYZRGB> correspondences_;
};

}  // namespace nie

#endif  // CORRESPONDENCE_VIEWER_VIEWER_HPP
