/*
 * Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission.
 */

#pragma once

#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace nie {

namespace ndt {

namespace detail {

/** Calculates the difference between two radian values
 *
 * @note: Legacy autoware code.
 */
double CalcDiffForRadian(double lhs_rad, double rhs_rad);

}  // namespace detail

enum class MethodType {
    PCL_GENERIC = 0,
    PCL_ANH = 1,
    PCL_ANH_GPU = 2,
    PCL_OPENMP = 3,
};

/** Struct to store vehicle pose
 *
 * x,y,z are local coordinates in meters
 * roll, pitch, yaw represent the local orientation in radians
 */
struct Pose {
    friend Pose operator+(Pose lhs, Pose const& rhs);
    friend Pose operator-(Pose lhs, Pose const& rhs);
    friend bool operator==(Pose const& lhs, Pose const& rhs);
    friend bool operator!=(Pose const& lhs, Pose const& rhs);


    geometry_msgs::msg::PoseStamped ToPoseStamped(rclcpp::Time const& stamp, std::string const& frame_id) const;
    geometry_msgs::msg::Pose ToPose() const;
    Eigen::Matrix4f ToEigenMatrix4f() const;

    /** Calculates relative coordinates between two poses
     *
     * @note: Legacy autoware code.
     */
    static Pose ConvertPoseIntoRelativeCoordinate(Pose const& target_pose, Pose const& reference_pose);

    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

/** Struct to store vehicle rate of change for linear and angular movement such as velocity and acceleration
 *
 * absolute is the length of the combined linear rate of change vector
 * linear_x, linear_y, linear_z represent linear rates of change in [m/s^N]
 * roll, pitch, yaw represent angular rates of changes in [rad/s^N]
 *
 * where N is the number of derivations since position (velocity = 1, acceleration = 2, jerk = 3, etc.)
 *
 * @note: It is possible to add different derivations types, but make sure that you multiple the derived version
 * by M times the time difference, where M is the difference in N (see above)
 * e.g. velocity = velocity + acceleration * dt + jerk * dt * dt
 */
struct RateOfChange {
    friend RateOfChange operator+(RateOfChange lhs, RateOfChange const& rhs);
    friend RateOfChange operator-(RateOfChange lhs, RateOfChange const& rhs);
    friend RateOfChange operator/(RateOfChange lhs, double const& rhs);
    friend RateOfChange operator*(RateOfChange lhs, double const& rhs);

    geometry_msgs::msg::TwistStamped ToTwistStamped(rclcpp::Time const& stamp, std::string const& frame_id) const;

    /** Calculates twist (velocities) from the difference between two poses
     *
     * @param current: Current position with orientation
     * @param previous: Previous position with orientation
     * @param dt: time difference between poses in seconds
     *
     * @return NdtRateOfChange structure with calculated velocities
     */
    static RateOfChange FromPoseDifference(Pose const& current, Pose const& previous, double dt);

    /** Calculates derived rate of change (e.g. acceleration, jerk) from the difference between two rates of change
     *
     * @param current: Current rate of change, linear and angular
     * @param previous: Previous rate of change, linear and angular
     * @param dt: time difference between rates of change in seconds
     *
     * @return NdtRateOfChange structure with derived values
     */
    static RateOfChange FromDifference(RateOfChange const& current, RateOfChange const& previous, double dt);

    double absolute;
    double linear_x;
    double linear_y;
    double linear_z;
    double angular_x;
    double angular_y;
    double angular_z;
};

}  // namespace ndt

}  // namespace nie
