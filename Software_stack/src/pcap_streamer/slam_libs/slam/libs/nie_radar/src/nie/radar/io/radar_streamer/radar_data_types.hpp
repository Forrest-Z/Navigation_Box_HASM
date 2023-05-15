/* Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <Eigen/Dense>
#include "continental.hpp"

namespace nie::io::radar {

struct Object {
    Object() = default;

    Object(continental::Object const& other) {
        f_obj_id = other.f_obj_id;
        position = Eigen::Vector2d(static_cast<double>(other.f_dist_x), static_cast<double>(other.f_dist_y)) *
                   continental::obj_const_res::kPosition;
        abs_velocity = Eigen::Vector2d(static_cast<double>(other.f_v_abs_x), static_cast<double>(other.f_v_abs_y)) *
                       continental::obj_const_res::kVelocity;
        abs_acceleration =
                Eigen::Vector2d(static_cast<double>(other.f_acc_abs_x), static_cast<double>(other.f_acc_abs_y)) *
                continental::obj_const_res::kAcceleration;
        position_std =
                Eigen::Vector2d(static_cast<double>(other.f_dist_x_std), static_cast<double>(other.f_dist_y_std)) *
                continental::obj_const_res::kStdDeviationPos;
        abs_velocity_std =
                Eigen::Vector2d(static_cast<double>(other.f_v_abs_x_std), static_cast<double>(other.f_v_abs_y_std)) *
                continental::obj_const_res::kStdDeviationAcc;
        abs_acceleration_std =
                Eigen::Vector2d(static_cast<double>(other.f_a_abs_x_std), static_cast<double>(other.f_a_abs_y_std)) *
                continental::obj_const_res::kStdDeviationAcc;
        box_left =
                Eigen::Vector2d(static_cast<double>(other.a_l_delta_x[0]), static_cast<double>(other.a_l_delta_y[0])) *
                continental::obj_const_res::kDistance;
        box_center =
                Eigen::Vector2d(static_cast<double>(other.a_l_delta_x[1]), static_cast<double>(other.a_l_delta_y[1])) *
                continental::obj_const_res::kDistance;
        box_right =
                Eigen::Vector2d(static_cast<double>(other.a_l_delta_x[2]), static_cast<double>(other.a_l_delta_y[2])) *
                continental::obj_const_res::kDistance;
        yaw = static_cast<double>(other.f_obj_orientation) * continental::obj_const_res::kYaw;
        score = other.u_score;
    }

    std::uint8_t f_obj_id;
    Eigen::Vector2d position;
    Eigen::Vector2d abs_velocity;
    Eigen::Vector2d abs_acceleration;
    Eigen::Vector2d position_std;
    Eigen::Vector2d abs_velocity_std;
    Eigen::Vector2d abs_acceleration_std;
    Eigen::Vector2d box_left;
    Eigen::Vector2d box_center;
    Eigen::Vector2d box_right;
    double yaw;
    std::uint8_t score;
};

}  // namespace nie::io::radar
