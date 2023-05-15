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

namespace aiim {

    struct SystemStates {
        float speed;
        float heading_angle;
        float angular_velocity;
        float side_slip_angle;
        float x_pos;
        float y_pos;
    };

    struct SystemParameters {
        float mass;
        float length;
        float length_front;
        float length_rear;
        float sampling_rate;
        float damping_coefficient;
    };

    struct SystemInputs {
        float steering;
        float throttle;
    };

    class AgvDynamicModel {
    public:
        AgvDynamicModel(SystemStates states, SystemParameters parameters, SystemInputs inputs);
        void UpdateStates();
        SystemStates GetStates();
        void UpdateInputs(SystemInputs inputs);

    private:
        float ThrottleToAcceleration(float throttle);

        SystemStates states_;
        SystemParameters parameters_;
        SystemInputs inputs_;
    };
}   // namespace aiim
