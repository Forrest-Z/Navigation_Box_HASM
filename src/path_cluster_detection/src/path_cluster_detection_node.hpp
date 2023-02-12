/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights
 * reserved Information classification: Confidential This content is protected
 * by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <aiim_autoware_msgs/msg/bounding_box_array.hpp>
#include <aiim_autoware_msgs/msg/vehicle_kinematic_state.hpp>
#include <aiim_roscpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class PathClusterDetectionNode : public rclcpp::Node {
public:
    PathClusterDetectionNode();

private:
    // Constants
    std::string const kOutputFrameId = "map";

    template <typename T>
    void GetParameter(std::string const& key, T* value) {
        declare_parameter(key);
        RCHECK(get_logger(), "Parameter " << key << " not set.", get_parameter(key, *value));
    }

    // Pointcloud Functions
    pcl::PointCloud<pcl::PointXYZ>::Ptr FilterPointsXZ(pcl::PointCloud<pcl::PointXYZ>::Ptr const input);
    pcl::PointCloud<pcl::PointXYZ>::Ptr DownsamplePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr const input);
    pcl::PointCloud<pcl::PointXYZ>::Ptr TransformPointsToVehicle(pcl::PointCloud<pcl::PointXYZ>::Ptr const input);
    pcl::PointCloud<pcl::PointXYZ>::Ptr TransformPointsToMap(pcl::PointCloud<pcl::PointXYZ>::Ptr const input);
    // Clustering/Detection
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> FindClusters(pcl::PointCloud<pcl::PointXYZ>::Ptr const input);
    void FindBoundingBoxes(pcl::PointCloud<pcl::PointXYZ>::Ptr const input);
    void Detect(pcl::PointCloud<pcl::PointXYZ>::Ptr const input);
    // Visualization
    void PublishBBoxViz(aiim_autoware_msgs::msg::BoundingBoxArray const& boxes);
    // Callback functions
    void PointsCallback(sensor_msgs::msg::PointCloud2::SharedPtr input);
    void StateCallback(aiim_autoware_msgs::msg::VehicleKinematicState::SharedPtr state);

    // Transform Handling
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_;
    rclcpp::Publisher<aiim_autoware_msgs::msg::BoundingBoxArray>::SharedPtr pub_bbox_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_bbox_viz_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
    rclcpp::Subscription<aiim_autoware_msgs::msg::VehicleKinematicState>::SharedPtr sub_state_;

    // Parameters
    // Topics
    std::string points_topic_;
    std::string state_topic_;
    std::string cluster_topic_;
    std::string bbox_topic_;

    // Vehicle Information
    std::string vehicle_reference_frame_;
    bool state_received_ = false;

    // Cloud Filter
    float height_min_;
    float height_max_;
    float obstacle_distance_min_;
    float obstacle_distance_max_;
    float voxel_leaf_size_;

    // Conditional Euclidian Clustering
    bool use_lfit_;
    float cluster_tolerance_;
    int cluster_points_min_;
    int cluster_points_max_;
    size_t cloud_points_min_;

    // Variables
    std_msgs::msg::Header cloud_header_;
    aiim_autoware_msgs::msg::VehicleKinematicState::SharedPtr latest_state_;
};
