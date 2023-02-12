/**
 * Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission.
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <aiim_roscpp/logging.hpp>

// TODO: Could move this as well.
template <typename T>
auto pcl_make_shared() {
    return typename T::Ptr(new T());
}

class VoxelGridFilterNode : public rclcpp::Node {
public:
    VoxelGridFilterNode() : rclcpp::Node("voxel_grid_filter") {
        std::string input_topic;
        std::string output_topic;
        double voxel_grid_leaf_size;

        GetParameter("input_topic", &input_topic);
        GetParameter("output_topic", &output_topic);
        GetParameter("voxel_grid_leaf_size", &voxel_grid_leaf_size);
        GetParameter("filter_min_range", &filter_min_range_);
        GetParameter("filter_max_range", &filter_max_range_);
        GetParameter("filter_retry_enabled", &filter_retry_enabled_);
        GetParameter("filter_retry_min_points", &filter_retry_min_points_);
        GetParameter("filter_retry_max_range", &filter_retry_max_range_);

        // We always check the filter values even if we don't filter. But this prevents surprises when filtering is enabled.
        RCHECK(get_logger(), "filter_retry_min_points<0: [" << filter_retry_min_points_ << "]", filter_retry_min_points_ >= 0);
        RCHECK(get_logger(), "filter_min_range<0: [" <<  filter_min_range_ << "]", filter_min_range_ >= 0.0);
        RCHECK(get_logger(), "filter_min_range>=filter_max_range: [" << filter_min_range_ << ", " << filter_max_range_ << "]", filter_max_range_ > filter_min_range_);
        RCHECK(get_logger(), "filter_min_range>=filter_retry_max_range: [" << filter_min_range_ << ", " << filter_retry_max_range_ << "]", filter_retry_max_range_ > filter_min_range_);
        RCHECK(get_logger(), "filter_max_range>=filter_retry_max_range: [" << filter_max_range_ << ", " << filter_retry_max_range_ << "]", filter_retry_max_range_ > filter_max_range_);

        filter_.setLeafSize(voxel_grid_leaf_size, voxel_grid_leaf_size, voxel_grid_leaf_size);

        publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
        // Because we use a member function, we have to bind the this pointer as the first argument of that function.
        subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(input_topic, 1, std::bind(&VoxelGridFilterNode::Service, this, std::placeholders::_1));
    }

private:
    template <typename T>
    void GetParameter(std::string const& key, T* value) {
        declare_parameter(key);
        RCHECK(get_logger(), "Parameter " << key << " not set.", get_parameter(key, *value));
    }

    // Function to filter on minimum and maximum range
    // Does not filter values based on Z.
    pcl::PointCloud<pcl::PointXYZI>::Ptr CreatedFilteredByRange(
            pcl::PointCloud<pcl::PointXYZI> const& src,
            double const min_range,
            double const max_range) {
        auto dst = pcl_make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        dst->header = src.header;

        double min_range_squared = min_range * min_range;
        double max_range_squared = max_range * max_range;

        for (auto const& p : src.points) {
            double squared_distance = p.x * p.x + p.y * p.y;
            if (min_range_squared <= squared_distance && squared_distance <= max_range_squared) {
                dst->points.push_back(p);
            }
        }

        return dst;
    }

    // The service this node provides is executed here: Receive an input point cloud, filter it and publish the result.
    void Service(sensor_msgs::msg::PointCloud2::ConstSharedPtr input) {
        auto src = pcl_make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        auto dst = pcl_make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        pcl::fromROSMsg(*input, *src);

        filter_.setInputCloud(CreatedFilteredByRange(*src, filter_min_range_, filter_max_range_));
        filter_.filter(*dst);

        // When there are not enough points, the range is set to "large" and we try again, hopefully keeping more of them.
        if (filter_retry_enabled_ && dst->points.size() < filter_retry_min_points_) {
            filter_.setInputCloud(CreatedFilteredByRange(*src, filter_min_range_, filter_retry_max_range_));
            filter_.filter(*dst);
        }

        auto output = std::make_unique<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(*dst, *output);
        output->header = input->header;
        publisher_->publish(std::move(output));
    }

    double filter_min_range_;
    double filter_max_range_;
    bool filter_retry_enabled_;
    std::size_t filter_retry_min_points_;
    double filter_retry_max_range_;
    pcl::VoxelGrid<pcl::PointXYZI> filter_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};
