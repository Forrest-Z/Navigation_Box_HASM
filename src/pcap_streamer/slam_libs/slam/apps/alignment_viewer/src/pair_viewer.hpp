/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <nie/drawing/drawing.hpp>
#include <nie/drawing/drawing_color_handlers.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>
#include <nie/lidar/cloud_filter.hpp>
#include <nie/lidar/io/las_reader.hpp>
#include <nie/lidar/trace_cloud.hpp>

#include "registered_pair.hpp"

namespace detail {

template <typename PointT>
class ViewableData {
public:
    nie::Cloud<PointT> cloud_a_untransformed;
    nie::Cloud<PointT> cloud_b_untransformed;
    nie::Cloud<PointT> cloud_b_transformed;

    nie::Cloud<pcl::PointXYZ> trace_a_untransformed;
    nie::Cloud<pcl::PointXYZ> trace_b_untransformed;
    nie::Cloud<pcl::PointXYZ> trace_b_transformed;
};

}  // namespace detail

template <typename PointT>
class PairViewer {
public:
    explicit PairViewer(std::string const& name, std::vector<nie::io::PoseRecord> trace, double filter_grid_size)
        : pcl_viewer_(name),
          trace_(std::move(trace)),
          filter_(typename nie::CloudFilter<PointT>::Parameters{filter_grid_size}),
          visualize_registered_cloud_(true),
          visualize_registered_trace_(true),
          visualize_unregistered_cloud_(true),
          visualize_unregistered_trace_(true) {}

    bool& visualize_registered_cloud() { return visualize_registered_cloud_; }
    bool& visualize_registered_trace() { return visualize_registered_trace_; }
    bool& visualize_unregistered_cloud() { return visualize_unregistered_cloud_; }
    bool& visualize_unregistered_trace() { return visualize_unregistered_trace_; }

    pcl::visualization::PCLVisualizer& viewer() { return pcl_viewer_.viewer; }

    void View() { pcl_viewer_.View(); }

    void Update(nie::RegisteredPair const& pair) {
        cloud_data_ = Read(pair);
        CreateScene();
        Update();
    }

    void Update() {
        pcl_viewer_.SetVisibility(registered_cloud_name_, visualize_registered_cloud_);
        pcl_viewer_.SetVisibility(registered_trace_name_, visualize_registered_trace_);
        pcl_viewer_.SetVisibility(unregistered_cloud_name_, visualize_unregistered_cloud_);
        pcl_viewer_.SetVisibility(unregistered_trace_name_, visualize_unregistered_trace_);
    }

private:
    void CreateScene() {
        pcl_viewer_.Reset();

        std::vector<nie::Cloud<PointT>> clouds = {
                cloud_data_.cloud_a_untransformed, cloud_data_.cloud_b_transformed, cloud_data_.cloud_b_untransformed};
        nie::AddClouds(
                clouds,
                nie::GetColorHandlersRegisteredClouds3<PointT>(),
                {"ref cloud", registered_cloud_name_, unregistered_cloud_name_},
                &pcl_viewer_);
        nie::AddClouds<pcl::PointXYZ>(
                {cloud_data_.trace_a_untransformed, cloud_data_.trace_b_transformed, cloud_data_.trace_b_untransformed},
                nie::GetColorHandlersRegisteredTraces3(),
                {"ref trace", registered_trace_name_, unregistered_trace_name_},
                &pcl_viewer_);

        nie::AddCoordinateSystem(clouds, &pcl_viewer_);
    }

    detail::ViewableData<PointT> Read(nie::RegisteredPair const& pair) {
        auto const [t_corr, r_corr] = CalculateCorrection(pair);
        LOG(INFO) << "Loading registered pair with ids " << pair.id_a << " and " << pair.id_b << " and files "
                  << boost::filesystem::path(pair.filename_a).filename().string() << " and "
                  << boost::filesystem::path(pair.filename_b).filename().string() << ", having a correction of "
                  << t_corr << " meter and " << r_corr << " degrees.";

        detail::ViewableData<PointT> result{};

        // The clouds are in local loam coordinates, the absolute axis-aligned bounding box poses and the icp
        // transformation are used to relate the b to a
        auto const T_ab = pair.bbox_pose_abs_a.Inversed() * pair.bbox_pose_abs_b;

        result.cloud_a_untransformed =
                filter_.Filter(nie::io::ReadLas<PointT>(pair.filename_a, nie::ToGPSWeekTime(pair.timestamp_a).week));

        auto const& origin_a = result.cloud_a_untransformed.origin();

        auto cloud_b = nie::io::ReadLas<PointT>(pair.filename_b, nie::ToGPSWeekTime(pair.timestamp_b).week);
        result.cloud_b_untransformed = filter_.Filter(nie::TransformCloud(cloud_b, T_ab, origin_a));
        result.cloud_b_transformed = filter_.Filter(nie::TransformCloud(cloud_b, pair.T_ab_icp, origin_a));

        // On top of the transformations also done for clouds, the absolute traces also need to be converted to the
        // local coordinates.
        result.trace_a_untransformed = nie::MakeTraceCloudFromTrace(
                trace_,
                origin_a,
                result.cloud_a_untransformed.time_range().first,
                result.cloud_a_untransformed.time_range().second);

        auto const& origin_b = cloud_b.origin();
        auto trace_b = nie::MakeTraceCloudFromTrace(
                trace_,
                origin_b,
                result.cloud_b_untransformed.time_range().first,
                result.cloud_b_untransformed.time_range().second);
        result.trace_b_untransformed = nie::TransformCloud(trace_b, T_ab, origin_a);
        result.trace_b_transformed = nie::TransformCloud(trace_b, pair.T_ab_icp, origin_a);

        return result;
    }

    detail::ViewableData<PointT> cloud_data_;

    std::string const registered_cloud_name_ = "registered cloud";
    std::string const registered_trace_name_ = "registered trace";
    std::string const unregistered_cloud_name_ = "unregistered cloud";
    std::string const unregistered_trace_name_ = "unregistered trace";

    nie::PclViewer pcl_viewer_;
    std::vector<nie::io::PoseRecord> const trace_;
    nie::CloudFilter<PointT> filter_;

    bool visualize_registered_cloud_;
    bool visualize_registered_trace_;

    bool visualize_unregistered_cloud_;
    bool visualize_unregistered_trace_;
};
