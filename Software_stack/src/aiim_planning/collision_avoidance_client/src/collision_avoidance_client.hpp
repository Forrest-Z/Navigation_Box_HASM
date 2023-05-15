// Copyright 2020 Arm Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// The changes made in this file, of which a summary is listed below, are copyrighted:
//
// Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
// Information classification: Confidential
// This content is protected by international copyright laws.
// Reproduction and distribution is prohibited without written permission.
//
// List of changes:
// * Changed namespace from autoware to aiim
// * Kept obstacle detection functionality from behavior planning node
// * Changed dependencies
// * Added modified trajectory visualization
#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <aiim_autoware_msgs/msg/trajectory.hpp>
#include <aiim_autoware_msgs/msg/trajectory_point.hpp>
#include <aiim_autoware_msgs/msg/vehicle_kinematic_state.hpp>
#include <aiim_autoware_msgs/srv/modify_trajectory.hpp>
#include <aiim_roscpp/logging.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace aiim {
namespace collision_avoidance_client {
using aiim_autoware_msgs::msg::Trajectory;
using aiim_autoware_msgs::msg::TrajectoryPoint;
using aiim_autoware_msgs::msg::VehicleKinematicState;
using aiim_autoware_msgs::srv::ModifyTrajectory;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

/// \class CollisionAvoidanceClient
/// \brief Modified trajectory publisher
class CollisionAvoidanceClient : public rclcpp::Node {
public:
    /// \brief default constructor, starts the planner
    /// \param[in] options name of the node for rclcpp internals
    explicit CollisionAvoidanceClient(const rclcpp::NodeOptions& options);

private:
    template <typename T>
    void GetParameter(std::string const& key, T* value) {
        declare_parameter(key);
        RCHECK(get_logger(), "Parameter " << key << " not set.", get_parameter(key, *value));
    }
    //  ROS Interface
    rclcpp::Client<ModifyTrajectory>::SharedPtr m_modify_trajectory_client;
    rclcpp::Subscription<VehicleKinematicState>::SharedPtr m_ego_state_sub{};
    rclcpp::Subscription<Trajectory>::SharedPtr m_trajectory_sub{};
    rclcpp::Publisher<Trajectory>::SharedPtr m_trajectory_pub{};
    rclcpp::Publisher<MarkerArray>::SharedPtr m_trajectory_viz_pub{};

    Marker to_marker(
            const TrajectoryPoint& traj_point, const std::string& frame_id, int32_t index, const std::string& ns);

    MarkerArray to_markers(const Trajectory& traj, const std::string& ns);

    // msg cache
    VehicleKinematicState m_ego_state;
    Trajectory m_trajectory;

    // Parameters
    // Topics
    std::string input_trajectory_topic_;
    std::string input_state_topic_;
    std::string output_trajectory_topic_;
    std::string output_trajectory_topic_viz_;

    // transforms
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

    // callbacks
    void on_ego_state(const VehicleKinematicState::SharedPtr& msg);
    void on_planned_trajectory(const Trajectory::SharedPtr& msg);
    void modify_trajectory_response(rclcpp::Client<ModifyTrajectory>::SharedFuture future);

    // other functions
    void init();
    VehicleKinematicState transform_to_map(const VehicleKinematicState& state);
};
}  // namespace collision_avoidance_client
}  // namespace aiim
