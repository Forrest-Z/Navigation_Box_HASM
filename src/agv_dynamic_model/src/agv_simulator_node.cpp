/**
 * Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission.
 */
#include "agv_simulator_node.hpp"

#include <chrono>

#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace aiim {

AgvSimulatorNode::AgvSimulatorNode() : rclcpp::Node("agv_simulator"), tf_broadcaster_(this) {
    // States
    SystemStates states;
    GetParameter("speed_initial", &states.speed);
    GetParameter("x_initial", &states.x_pos);
    GetParameter("y_initial", &states.y_pos);
    GetParameter("heading_angle", &states.heading_angle);
    GetParameter("angular_velocity", &states.angular_velocity);
    GetParameter("side_slip_angle", &states.side_slip_angle);

    // Parameters
    SystemParameters parameters;
    GetParameter("mass", &parameters.mass);
    GetParameter("length", &parameters.length);
    GetParameter("length_front", &parameters.length_front);
    GetParameter("length_rear", &parameters.length_rear);
    GetParameter("sampling_rate", &parameters.sampling_rate);
    GetParameter("damping_coefficient", &parameters.damping_coefficient);

    // Inputs
    SystemInputs inputs;
    GetParameter("steering", &inputs.steering);
    GetParameter("throttle", &inputs.throttle);

    std::string input_topic;
    std::string output_topic;
    GetParameter("input_topic", &input_topic);
    GetParameter("output_topic", &output_topic);

    // Frame id
    GetParameter("frame_id", &frame_id_);
    GetParameter("child_frame_id", &child_frame_id_);

    // Queue
    GetParameter("queue_size_publisher", &queue_size_publisher_);
    GetParameter("queue_size_subscriber", &queue_size_subscriber_);

    // Create instance & publisher
    model_ = std::make_unique<aiim::AgvDynamicModel>(states, parameters, inputs);
    publisher_ =
            this->create_publisher<aiim_autoware_msgs::msg::VehicleKinematicState>(output_topic, queue_size_publisher_);
    subscriber_ = this->create_subscription<aiim_autoware_msgs::msg::VehicleControlCommand>(
            input_topic, queue_size_subscriber_, [this](aiim_autoware_msgs::msg::VehicleControlCommand::SharedPtr arg) {
                Subscription(arg);
            });
    timer_ = this->create_wall_timer(
            std::chrono::duration<double>((1 / parameters.sampling_rate)), [this] { Publish(); });
}

void AgvSimulatorNode::Publish() {
    // Publish will create a unique pointer. So, to prevent a copy we create it as a unique pointer
    // here as well.
    auto vehicle_state = std::make_unique<aiim_autoware_msgs::msg::VehicleKinematicState>();

    model_->UpdateStates();
    SystemStates states = model_->GetStates();

    // Set Header
    vehicle_state->header.stamp = now();
    vehicle_state->header.frame_id = "map";

    // Set vehicle pose & convert heading angle to quaternion
    vehicle_state->state.x = states.x_pos;
    vehicle_state->state.y = states.y_pos;
    vehicle_state->state.heading.real = std::cos(states.heading_angle / 2);
    vehicle_state->state.heading.imag = std::sin(states.heading_angle / 2);

    // Vehicle velocity & angular velocity
    vehicle_state->state.longitudinal_velocity_mps = states.speed;
    vehicle_state->state.heading_rate_rps = states.angular_velocity;

    // Convert orientation from euler angle to quaternion
    tf2::Quaternion qt_v;
    qt_v.setRPY(0.0, 0.0, states.angular_velocity);

    // Set velocity and angular velocity.
    vehicle_state->delta.translation.x = states.speed;
    vehicle_state->delta.rotation.w = qt_v.w();
    vehicle_state->delta.rotation.x = qt_v.x();
    vehicle_state->delta.rotation.y = qt_v.y();
    vehicle_state->delta.rotation.z = qt_v.z();

    // Send TF "/base_link" to "/map"
    geometry_msgs::msg::TransformStamped baselink_to_map;

    baselink_to_map.header.stamp = now();
    baselink_to_map.header.frame_id = frame_id_;
    baselink_to_map.child_frame_id = child_frame_id_;
    baselink_to_map.transform.translation.x = vehicle_state->state.x;
    baselink_to_map.transform.translation.y = vehicle_state->state.y;
    baselink_to_map.transform.translation.z = 0;
    // Create orientation quaternion
    tf2::Quaternion q_orientation;
    q_orientation.setRPY(0.0, 0.0, states.heading_angle);
    baselink_to_map.transform.rotation = tf2::toMsg(q_orientation);

    publisher_->publish(std::move(vehicle_state));
    tf_broadcaster_.sendTransform(baselink_to_map);
}

void AgvSimulatorNode::Subscription(aiim_autoware_msgs::msg::VehicleControlCommand::SharedPtr const msg) {
    SystemInputs system_inputs = {msg->front_wheel_angle_rad, msg->long_accel_mps2};
    model_->UpdateInputs(system_inputs);
}

}  // namespace aiim
