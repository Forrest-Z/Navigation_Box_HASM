/**
 * Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission.
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <aiim_autoware_msgs/msg/vehicle_control_command.hpp>
#include <aiim_autoware_msgs/msg/vehicle_kinematic_state.hpp>
#include <aiim_roscpp/logging.hpp>

#include "agv_dynamic_model.hpp"

namespace aiim {

class AgvSimulatorNode : public rclcpp::Node {
public:
    AgvSimulatorNode();

private:
    void Publish();
    void Subscription(aiim_autoware_msgs::msg::VehicleControlCommand::SharedPtr const msg);
    template <typename T>
    void GetParameter(std::string const& key, T* value) {
        declare_parameter(key);
        RCHECK(get_logger(), "Parameter " << key << " not set.", get_parameter(key, *value));
    }

    std::unique_ptr<AgvDynamicModel> model_;
    rclcpp::Publisher<aiim_autoware_msgs::msg::VehicleKinematicState>::SharedPtr publisher_;
    rclcpp::Subscription<aiim_autoware_msgs::msg::VehicleControlCommand>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::string frame_id_;
    std::string child_frame_id_;
    int queue_size_publisher_;
    int queue_size_subscriber_;
};

}   // namespace aiim
