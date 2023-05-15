/* Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <nie/drawing/drawing.hpp>
#include <nie/drawing/drawing_color_handlers.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>
#include <nie/lidar/cloud_matcher.hpp>
#include <nie/lidar/trace_cloud.hpp>

#include "las_loader.hpp"
#include "las_pair.hpp"

namespace detail {

// Pyramid scaled parameters. Half of those of the loop_closure_matcher.
template <typename PointT>
std::vector<typename nie::CloudMatcher<PointT>::Parameters> GetMatchParameters() {
    std::vector<typename nie::CloudMatcher<PointT>::Parameters> matcher_parameters;
    auto parameters = typename nie::CloudMatcher<PointT>::Parameters();

    parameters.transformation_eps = 1.e-9;
    parameters.euclidean_fitness_eps = -1.0;
    parameters.max_iterations = 30;
    parameters.ransac_max_iterations = 15;

    // Alignment step 1
    parameters.max_correspondence_distance = 5.0;
    parameters.ransac_outlier_rejection_threshold = 2.5;
    matcher_parameters.push_back(parameters);

    // Alignment step 2
    parameters.max_correspondence_distance = 2.5;
    parameters.ransac_outlier_rejection_threshold = 1.25;
    parameters.max_iterations = 20;
    matcher_parameters.push_back(parameters);

    // Alignment step 3
    parameters.max_correspondence_distance = 1.25;
    parameters.ransac_outlier_rejection_threshold = 0.625;
    matcher_parameters.push_back(parameters);

    return matcher_parameters;
}

template <typename PointT>
class ViewableData {
public:
    nie::Isometry3qd T_ab;
    nie::Cloud<PointT> cloud_ref;
    nie::Cloud<PointT> cloud_mut;
};

}  // namespace detail

template <typename PointT>
class PairViewer {
public:
    explicit PairViewer(std::string const& name, LasLoader<pcl::PointXYZI> las_loader)
        : pcl_viewer_(name), las_loader_(std::move(las_loader)) {}

    pcl::visualization::PCLVisualizer& viewer() { return pcl_viewer_.viewer; }

    void View() { pcl_viewer_.View(); }

    void Update(nie::LasPair const& pair) {
        cloud_data_ = Read(pair);
        CreateScene();
    }

    Eigen::Vector3d& OriginMut() { return cloud_data_.cloud_mut.origin().translation(); }

    bool RunIcp(nie::Isometry3qd* T_ab) {
        // Note that the difference between the two origins as stored by cloud_data_ should be
        // identity or very close to it: We set the origin to be equal in the Read function.
        nie::Isometry3qd T_icp = nie::Isometry3qd::Identity();

        nie::CloudMatcher<PointT> matcher(detail::GetMatchParameters<PointT>());

        bool converged = matcher.Match(cloud_data_.cloud_ref, cloud_data_.cloud_mut, &T_icp);
        cloud_data_.cloud_mut.origin() = cloud_data_.cloud_mut.origin() * T_icp.Inversed();

        // Because we transformed the origin of b to equal a, we need to apply this difference
        // again to get the total transformation.
        // Reminder: cloud_data_.cloud_mut.origin() = T_b * cloud_data_.T_ab.Inversed() * T_icp.Inversed()
        *T_ab = T_icp * cloud_data_.T_ab;

        CreateScene();

        return converged;
    }

    // Update means update cloud b
    void Update() {
        // Nothing to see here
        pcl_viewer_.viewer.removePointCloud(mut_cloud_name_);
        nie::AddClouds<PointT>(
                cloud_data_.cloud_ref.origin().translation(),
                {cloud_data_.cloud_mut},
                {nie::GetColorHandlerRegisteredCloud<PointT>()},
                {mut_cloud_name_},
                &pcl_viewer_);
    }

private:
    void CreateScene() {
        pcl_viewer_.Reset();

        std::vector<nie::Cloud<PointT>> clouds = {cloud_data_.cloud_ref, cloud_data_.cloud_mut};
        nie::AddClouds(
                clouds,
                nie::GetColorHandlersRegisteredClouds2<PointT>(),
                {ref_cloud_name_, mut_cloud_name_},
                &pcl_viewer_);

        nie::AddCoordinateSystem(clouds, &pcl_viewer_);
    }

    detail::ViewableData<PointT> Read(nie::LasPair const& pair) {
        detail::ViewableData<PointT> result;
        // We set the origins to be equal by transforming the point cloud towards the origin of a. Hence the
        // name: T_ab (transform from b to a).
        result.T_ab = pair.bbox_pose_a.Inversed() * pair.bbox_pose_b;
        result.cloud_ref = las_loader_(pair.filename_a, nie::ToGPSWeekTime(pair.timestamp_a).week);
        result.cloud_mut = las_loader_(pair.filename_b, nie::ToGPSWeekTime(pair.timestamp_b).week, result.T_ab);

        return result;
    }

    detail::ViewableData<PointT> cloud_data_;

    std::string const ref_cloud_name_ = "ref_cloud";
    std::string const mut_cloud_name_ = "mut_cloud";

    nie::PclViewer pcl_viewer_;
    LasLoader<PointT> las_loader_;
};
