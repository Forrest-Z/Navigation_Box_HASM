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
#include "collision_avoidance_client.hpp"

#include <memory>
#include <string>

#include <aiim_geometry/common_2d.hpp>
#include <aiim_motion_common/motion_common.hpp>
#include <aiim_time_utils/time_utils.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2/time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rclcpp_components/register_node_macro.hpp>

namespace aiim {
namespace collision_avoidance_client {

CollisionAvoidanceClient::CollisionAvoidanceClient(const rclcpp::NodeOptions& options)
    : Node("collision_avoidance_client", options) {
    init();
}

void CollisionAvoidanceClient::init() {
    using rclcpp::QoS;
    using namespace std::chrono_literals;

    GetParameter("input_trajectory_topic", &input_trajectory_topic_);
    GetParameter("input_state_topic", &input_state_topic_);
    GetParameter("output_trajectory_topic", &output_trajectory_topic_);
    GetParameter("output_trajectory_topic_viz", &output_trajectory_topic_viz_);

    // Setup Tf Buffer with listener
    rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    m_tf_buffer = std::make_shared<tf2_ros::Buffer>(clock);
    m_tf_listener = std::make_shared<tf2_ros::TransformListener>(
            *m_tf_buffer, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false);

    m_modify_trajectory_client = this->create_client<ModifyTrajectory>("estimate_collision");
    while (!m_modify_trajectory_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service.");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_INFO(get_logger(), "Waiting for service...");
    }

    // Setup subscribers
    using SubAllocT = rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>;
    m_ego_state_sub = this->create_subscription<VehicleKinematicState>(
            input_state_topic_, QoS{10}, [this](const VehicleKinematicState::SharedPtr msg) { on_ego_state(msg); });
    m_trajectory_sub = this->create_subscription<Trajectory>(
            input_trajectory_topic_,
            QoS{10},
            [this](const Trajectory::SharedPtr msg) { on_planned_trajectory(msg); },
            SubAllocT{});

    // Setup publishers
    using PubAllocT = rclcpp::PublisherOptionsWithAllocator<std::allocator<void>>;
    m_trajectory_pub = this->create_publisher<Trajectory>(output_trajectory_topic_, QoS{10}, PubAllocT{});
    m_trajectory_viz_pub = this->create_publisher<MarkerArray>(output_trajectory_topic_viz_, QoS{10});
}

VehicleKinematicState CollisionAvoidanceClient::transform_to_map(const VehicleKinematicState& state) {
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = m_tf_buffer->lookupTransform(
                "map", state.header.frame_id, aiim::common::time_utils::from_message(state.header.stamp));
    } catch (const tf2::ExtrapolationException& ex) {
        RCLCPP_WARN_STREAM(
                get_logger(),
                "Could not get transform from " << state.header.frame_id << " to map, defaulting to identity.\n"
                                                << ex.what());
    }
    VehicleKinematicState transformed_state;
    aiim::motion::motion_common::doTransform(state, transformed_state, tf);
    transformed_state.header.frame_id = "map";
    transformed_state.header.stamp = state.header.stamp;
    return transformed_state;
}

void CollisionAvoidanceClient::on_ego_state(const VehicleKinematicState::SharedPtr& msg) {
    // Do nothing if localization result is not received yet.
    if (!m_tf_buffer->canTransform("map", msg->header.frame_id, tf2::TimePointZero)) {
        RCLCPP_INFO(get_logger(), "Waiting for localization result to become available");
        return;
    }

    m_ego_state = transform_to_map(*msg);

    static auto previous_output = std::chrono::system_clock::now();

    // If object collision estimation is enabled, send trajectory through the collision estimator
    if (m_modify_trajectory_client) {
        auto request = std::make_shared<ModifyTrajectory::Request>();
        request->original_trajectory = m_trajectory;
        auto result = m_modify_trajectory_client->async_send_request(
                request, std::bind(&CollisionAvoidanceClient::modify_trajectory_response, this, std::placeholders::_1));
    } else {
        m_trajectory_pub->publish(m_trajectory);
        auto markers = to_markers(m_trajectory, "modified");
        m_trajectory_viz_pub->publish(markers);
    }
}

void CollisionAvoidanceClient::on_planned_trajectory(const Trajectory::SharedPtr& msg) { m_trajectory = *msg; }

void CollisionAvoidanceClient::modify_trajectory_response(rclcpp::Client<ModifyTrajectory>::SharedFuture future) {
    auto trajectory = future.get()->modified_trajectory;

    // set current position with velocity zero to do emergency stop in case
    // collision estimator fails or if there is obstacle on first point
    if (trajectory.points.empty()) {
        auto stopping_point = m_ego_state.state;
        stopping_point.longitudinal_velocity_mps = 0.0;
        trajectory.points.push_back(stopping_point);
    }
    m_trajectory_pub->publish(trajectory);
    auto markers = to_markers(m_trajectory, "modified");
    m_trajectory_viz_pub->publish(markers);
}

MarkerArray CollisionAvoidanceClient::to_markers(const Trajectory& traj, const std::string& ns) {
    visualization_msgs::msg::MarkerArray markers;
    int32_t index = 0;

    for (const auto& traj_point : traj.points) {
        markers.markers.push_back(to_marker(traj_point, traj.header.frame_id, index, ns));
        index++;
    }

    return markers;
}

Marker CollisionAvoidanceClient::to_marker(
        const TrajectoryPoint& traj_point, const std::string& frame_id, int32_t index, const std::string& ns) {
    Marker marker;

    tf2::Quaternion quat;
    quat.setEuler(0.0, 0.0, aiim::motion::motion_common::to_angle(traj_point.heading));

    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Time(0);
    marker.ns = ns;
    marker.id = index;
    marker.type = Marker::ARROW;
    marker.action = Marker::ADD;
    marker.pose.position.x = traj_point.x;
    marker.pose.position.y = traj_point.y;
    marker.pose.orientation = toMsg(quat);
    marker.scale.x = 0.4;
    marker.scale.y = 0.15;
    marker.scale.z = 0.01;
    marker.color.r = 0.2f;
    marker.color.g = 0.8f;
    marker.color.b = 0.25f;
    marker.color.a = 1.0f;

    marker.frame_locked = false;

    return marker;
}

}  // namespace collision_avoidance_client
}  // namespace aiim

RCLCPP_COMPONENTS_REGISTER_NODE(aiim::collision_avoidance_client::CollisionAvoidanceClient)
