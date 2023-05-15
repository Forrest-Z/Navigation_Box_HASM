/**
 * Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission.
 */
#include "agv_dynamic_model.hpp"

namespace aiim {

    AgvDynamicModel::AgvDynamicModel(SystemStates states, SystemParameters parameters, SystemInputs inputs)
            : states_(states), parameters_(parameters), inputs_(inputs) {}

    float AgvDynamicModel::ThrottleToAcceleration(float throttle) {
        // Dummy function for now, will be implemented when more information from the AGV is available.
        return throttle;
    }

    void AgvDynamicModel::UpdateStates() {
        /*
            A kinematic bicycle model for cornering is used. More details: https://www.youtube.com/watch?v=HqNdBiej23I
            For longitudinal dynamics a single damper describes the relation between throttle setpoint and the
            longitudinal acceleration.
         */

        // States for kinematic bicycle model
        float v_x = states_.speed * std::cos(states_.heading_angle + states_.side_slip_angle);
        float v_y = states_.speed * std::sin(states_.heading_angle + states_.side_slip_angle);
        states_.angular_velocity =
                (states_.speed / parameters_.length) * std::cos(states_.side_slip_angle) * std::tan(inputs_.steering);
        states_.side_slip_angle = std::atan((parameters_.length_rear / parameters_.length) * std::tan(inputs_.steering));

        // Dynamic longitudinal acceleration model
        float acceleration = states_.speed * (-parameters_.damping_coefficient / parameters_.mass) +
                             (parameters_.damping_coefficient / parameters_.mass) * ThrottleToAcceleration(inputs_.throttle);

        states_.speed = states_.speed + acceleration * (1 / parameters_.sampling_rate);

        // Update vehicle pose
        states_.x_pos = states_.x_pos + v_x * (1 / parameters_.sampling_rate);
        states_.y_pos = states_.y_pos + v_y * (1 / parameters_.sampling_rate);
        states_.heading_angle = states_.heading_angle + states_.angular_velocity * (1 / parameters_.sampling_rate);
    }

    SystemStates AgvDynamicModel::GetStates(){
        return states_;
    }

    void AgvDynamicModel::UpdateInputs(SystemInputs inputs) {
        inputs_.throttle = inputs.throttle;
        inputs_.steering = inputs.steering;
    }

}   // namespace aiim