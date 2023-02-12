/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights
 * reserved Information classification: Confidential This content is protected
 * by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "path_cluster_detection_node.hpp"

#include <memory>
#include <string>
#include <utility>  // move
#include <vector>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>

#include <aiim_geometry/bounding_box/eigenbox_2d.hpp>
#include <aiim_geometry/bounding_box/lfit.hpp>

PathClusterDetectionNode::PathClusterDetectionNode() : rclcpp::Node("path_cluster_detection") {
    // Load parameters
    // Publish/subscribe topics
    GetParameter("points_topic", &points_topic_);
    GetParameter("cluster_topic", &cluster_topic_);
    GetParameter("state_topic", &state_topic_);
    GetParameter("bbox_topic", &bbox_topic_);

    // Vehicle Information
    GetParameter("vehicle_reference_frame", &vehicle_reference_frame_);

    // Path filter parameters;
    GetParameter("height_min", &height_min_);
    GetParameter("height_max", &height_max_);
    GetParameter("obstacle_distance_min", &obstacle_distance_min_);
    GetParameter("obstacle_distance_max", &obstacle_distance_max_);
    GetParameter("voxel_leaf_size", &voxel_leaf_size_);

    // Euclidian Clustering parameters
    GetParameter("use_lfit", &use_lfit_);
    GetParameter("cluster_tolerance", &cluster_tolerance_);
    GetParameter("cluster_points_min", &cluster_points_min_);
    GetParameter("cluster_points_max", &cluster_points_max_);
    GetParameter("cloud_points_min", &cloud_points_min_);

    // Set Transform
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Set QoS defaults
    auto const qos_subscribers = rclcpp::SystemDefaultsQoS();
    auto const qos_publishers = rclcpp::SystemDefaultsQoS();

    // Create subscribers
    sub_points_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            points_topic_,
            qos_subscribers,
            std::bind(&PathClusterDetectionNode::PointsCallback, this, std::placeholders::_1));
    sub_state_ = create_subscription<aiim_autoware_msgs::msg::VehicleKinematicState>(
            state_topic_,
            qos_subscribers,
            std::bind(&PathClusterDetectionNode::StateCallback, this, std::placeholders::_1));
    // Create publishers
    pub_points_ = create_publisher<sensor_msgs::msg::PointCloud2>(cluster_topic_, qos_publishers);
    pub_bbox_ = create_publisher<aiim_autoware_msgs::msg::BoundingBoxArray>(bbox_topic_, qos_publishers);
    pub_bbox_viz_ = create_publisher<visualization_msgs::msg::MarkerArray>(bbox_topic_ + "_viz", qos_publishers);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PathClusterDetectionNode::FilterPointsXZ(
        pcl::PointCloud<pcl::PointXYZ>::Ptr const input) {
    // Initialize output cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_x(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_xz(new pcl::PointCloud<pcl::PointXYZ>);

    // Filter only in front of car
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(obstacle_distance_min_, obstacle_distance_max_);
    pass.filter(*filtered_x);
    // TODO(tijs.vandersmagt): Replace this with a proper ground filter
    pass.setInputCloud(filtered_x);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(height_min_, height_max_);
    pass.filter(*filtered_xz);

    return filtered_xz;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PathClusterDetectionNode::DownsamplePoints(
        pcl::PointCloud<pcl::PointXYZ>::Ptr const input) {
    // Initialize output cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>);

    // Downsample using voxel grid
    pcl::VoxelGrid<pcl::PointXYZ> downsampler;
    downsampler.setInputCloud(input);
    downsampler.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    downsampler.filter(*downsampled);

    return downsampled;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PathClusterDetectionNode::TransformPointsToVehicle(
        pcl::PointCloud<pcl::PointXYZ>::Ptr const input) {
    // Initialize output cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);

    // Get Transform
    rclcpp::Duration const transform_timeout(1.0);
    geometry_msgs::msg::TransformStamped tf_cloud_to_vehicle;
    try {
        tf_cloud_to_vehicle = tf_buffer_->lookupTransform(
                vehicle_reference_frame_, input->header.frame_id, rclcpp::Time(0), transform_timeout);
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN_STREAM(
                get_logger(),
                "Could not get transform from " << input->header.frame_id << " to " << vehicle_reference_frame_
                                                << ", defaulting to identity.\n"
                                                << ex.what());
    }

    Eigen::Matrix4d const tf_matrix = tf2::transformToEigen(tf_cloud_to_vehicle).matrix();

    pcl::transformPointCloud(*input, *transformed, tf_matrix);

    return transformed;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PathClusterDetectionNode::TransformPointsToMap(
        pcl::PointCloud<pcl::PointXYZ>::Ptr const input) {
    // TODO(tijs.vandersmagt): Instead also use transformListener to get the transform.

    // Initialize output cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Vector3f const transl(latest_state_->state.x, latest_state_->state.y, 0);
    // We build a quaternion assuming the pitch/roll are zero, thus x and y are 0.
    // w (real) and z (complex) are given in the vehicle state
    Eigen::Quaternionf const rot(latest_state_->state.heading.real, 0, 0, latest_state_->state.heading.imag);
    pcl::transformPointCloud(*input, *transformed, transl, rot);

    return transformed;
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> PathClusterDetectionNode::FindClusters(
        pcl::PointCloud<pcl::PointXYZ>::Ptr const input) {
    // This function is used to find clusters in the given cloud.

    // Create filtering objects
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclidian_cluster_extraction;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusters_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    clusters_cloud->header.frame_id = input->header.frame_id;

    // Create vectors to save information
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> out_clusters_ptr;
    std::vector<pcl::PointIndices> cluster_indices;

    // Check if the input cloud is not empty
    if (input->empty() || input->size() < cloud_points_min_) {
        // Publish empty cloud to reset visualization
        auto output = std::make_unique<sensor_msgs::msg::PointCloud2>();
        output->header = cloud_header_;
        pub_points_->publish(std::move(output));
        return out_clusters_ptr;
    }

    // Creating the KdTree object for the search method of the extraction
    tree->setInputCloud(input);

    // Set euclidian_cluster_extraction object parameters
    euclidian_cluster_extraction.setClusterTolerance(
            cluster_tolerance_);  // [m] This tolerance determines how close points can
                                  // be together to be clustered together.
    euclidian_cluster_extraction.setMinClusterSize(cluster_points_min_);  // This is the minimum amount of points
                                                                          // required to form a cluster
    euclidian_cluster_extraction.setMaxClusterSize(cluster_points_max_);  // This is the maximum amount of points a
                                                                          // cluster is allowed to have
    euclidian_cluster_extraction.setSearchMethod(tree);
    euclidian_cluster_extraction.setInputCloud(input);

    // Extract the clusters
    euclidian_cluster_extraction.extract(cluster_indices);

    // Loop over every detected cluster
    for (std::vector<pcl::PointIndices>::const_iterator cluster_it = cluster_indices.begin();
         cluster_it != cluster_indices.end();
         ++cluster_it) {
        // Create pointcloud object to store cluster
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);

        // Fill cloud with cluster points
        for (std::vector<int>::const_iterator point_it = cluster_it->indices.begin();
             point_it != cluster_it->indices.end();
             ++point_it) {
            cloud_cluster->points.push_back(input->points[*point_it]);
        }
        // Set necessary properties
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // Store in output cloud
        out_clusters_ptr.push_back(cloud_cluster);
        *clusters_cloud += *cloud_cluster;
    }
    // Publish found clusters
    auto output = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*clusters_cloud, *output);
    output->header = cloud_header_;
    pub_points_->publish(std::move(output));

    // Return the array with found obstacle clusters
    return out_clusters_ptr;
}

void PathClusterDetectionNode::FindBoundingBoxes(pcl::PointCloud<pcl::PointXYZ>::Ptr const input) {
    if (input->size() <= 0) {
        return;
    }
    // Obtain clusters from the points on the path
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> found_clusters = FindClusters(input);
    // TODO(tijs.vandersmagt): It is unclear whether the positions need to be in the coordinate frame of the vehicle or
    // the map. Maybe it does not matter, otherwise we might have to transform the point cloud.

    aiim_autoware_msgs::msg::BoundingBoxArray bbox_array;

    // Loop over clusters
    for (auto const& cluster : found_clusters) {
        aiim_autoware_msgs::msg::BoundingBox bbox;
        if (use_lfit_){
            // Using lfit method if it is set to True in the yaml file
            bbox = aiim::common::geometry::bounding_box::lfit_bounding_box_2d(cluster->points.begin(), cluster->points.end());
            // RCLCPP_WARN(get_logger(), "using lfit method");
        }else{
            // Using eigenbox_2d method if use_lfit is set to False in the yaml file
            bbox = aiim::common::geometry::bounding_box::eigenbox_2d(cluster->points.begin(), cluster->points.end());
            // RCLCPP_WARN(get_logger(), "using eigen method");
        }
        bbox_array.boxes.push_back(bbox);
    }
    bbox_array.header = cloud_header_;
    bbox_array.header.frame_id = kOutputFrameId;
    PublishBBoxViz(bbox_array);
    pub_bbox_->publish(std::move(bbox_array));
}

void PathClusterDetectionNode::Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr const input) {
    // Filter points behind car
    auto points_in_front = FilterPointsXZ(input);
    if (points_in_front->points.size() < cloud_points_min_) {
        return;
    }
    // Downsample points
    auto points_downsampled = DownsamplePoints(points_in_front);
    if (points_downsampled->points.size() < cloud_points_min_) {
        return;
    }
    // Transform
    auto points_transformed = TransformPointsToMap(points_downsampled);
    // Find and publish bounding boxes
    FindBoundingBoxes(points_transformed);
}

void PathClusterDetectionNode::PointsCallback(sensor_msgs::msg::PointCloud2::SharedPtr input) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    if (!state_received_) {
        RCLCPP_WARN(get_logger(), "Did not receive vehicle state yet, skipping input for cluster detection.");
        return;
    }
    // Store header to keep for output
    cloud_header_ = input->header;
    // Convert from PointCloud2 to pcl::PointCloud
    pcl::fromROSMsg(*input, *pcl_cloud);
    // Call main detection
    Detect(TransformPointsToVehicle(pcl_cloud));
}

void PathClusterDetectionNode::StateCallback(aiim_autoware_msgs::msg::VehicleKinematicState::SharedPtr state) {
    latest_state_ = state;
    state_received_ = true;
}

void PathClusterDetectionNode::PublishBBoxViz(aiim_autoware_msgs::msg::BoundingBoxArray const& boxes) {
    visualization_msgs::msg::MarkerArray marker_array;
    for (const auto& box : boxes.boxes) {
        visualization_msgs::msg::Marker m{};
        m.header.stamp = now();
        m.header.frame_id = boxes.header.frame_id;
        m.ns = "bbox";
        m.id = static_cast<int>(marker_array.markers.size());
        m.type = visualization_msgs::msg::Marker::CUBE;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = static_cast<double>(box.centroid.x);
        m.pose.position.y = static_cast<double>(box.centroid.y);
        m.pose.position.z = static_cast<double>(box.centroid.z);
        m.pose.orientation.x = static_cast<double>(box.orientation.x);
        m.pose.orientation.y = static_cast<double>(box.orientation.y);
        m.pose.orientation.z = static_cast<double>(box.orientation.z);
        m.pose.orientation.w = static_cast<double>(box.orientation.w);
        // X and Y scale are swapped between these two message types
        m.scale.x = static_cast<double>(box.size.y);
        m.scale.y = static_cast<double>(box.size.x);
        m.scale.z = static_cast<double>(box.size.z);
        // These values are inherited from Autoware.
        m.color.r = 1.0;
        m.color.g = 0.5;
        m.color.b = 0.0;
        m.color.a = 0.75;
        m.lifetime.sec = 0;
        m.lifetime.nanosec = 500000000;
        // Add to main array
        marker_array.markers.push_back(m);
    }
    pub_bbox_viz_->publish(marker_array);
}