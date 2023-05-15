/*
 * Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission.
 */

#include "ndt_matching_types.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace nie {

namespace ndt {

namespace detail {

double CalcDiffForRadian(double const lhs_rad, double const rhs_rad) {
    double diff_rad = lhs_rad - rhs_rad;
    if (diff_rad >= M_PI) {
        diff_rad = diff_rad - 2 * M_PI;
    } else if (diff_rad < -M_PI) {
        diff_rad = diff_rad + 2 * M_PI;
    }
    return diff_rad;
}

}  // namespace detail

Pose operator+(Pose lhs, Pose const& rhs) {
    lhs.x += rhs.x;
    lhs.y += rhs.y;
    lhs.z += rhs.z;
    lhs.roll += rhs.roll;
    lhs.pitch += rhs.pitch;
    lhs.yaw += rhs.yaw;
    return lhs;
}

Pose operator-(Pose lhs, Pose const& rhs) {
    lhs.x -= rhs.x;
    lhs.y -= rhs.y;
    lhs.z -= rhs.z;
    lhs.roll = detail::CalcDiffForRadian(lhs.roll, rhs.roll);
    lhs.pitch = detail::CalcDiffForRadian(lhs.pitch, rhs.pitch);
    lhs.yaw = detail::CalcDiffForRadian(lhs.yaw, rhs.yaw);
    return lhs;
}

bool operator==(Pose const& lhs, Pose const& rhs) {
    return std::tie(lhs.x, lhs.y, lhs.z, lhs.roll, lhs.pitch, lhs.yaw) ==
           std::tie(rhs.x, rhs.y, rhs.z, rhs.roll, rhs.pitch, rhs.yaw);
}

bool operator!=(Pose const& lhs, Pose const& rhs) { return !(lhs == rhs); }

geometry_msgs::msg::PoseStamped Pose::ToPoseStamped(rclcpp::Time const& stamp, std::string const& frame_id) const {
    geometry_msgs::msg::PoseStamped msg;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = z;
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    return msg;
}

geometry_msgs::msg::Pose Pose::ToPose() const {
    geometry_msgs::msg::Pose msg;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();
    msg.orientation.w = q.w();
    return msg;
}

Eigen::Matrix4f Pose::ToEigenMatrix4f() const {
    Eigen::Translation3f translation(x, y, z);
    Eigen::AngleAxisf rot_x(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z(yaw, Eigen::Vector3f::UnitZ());
    return (translation * rot_z * rot_y * rot_x).matrix();
}

/** Calculates relative coordinates between two poses
 *
 * @note: Legacy autoware code.
 */
Pose Pose::ConvertPoseIntoRelativeCoordinate(Pose const& target_pose, Pose const& reference_pose) {
    tf2::Quaternion target_q;
    target_q.setRPY(target_pose.roll, target_pose.pitch, target_pose.yaw);
    tf2::Vector3 target_v(target_pose.x, target_pose.y, target_pose.z);
    tf2::Transform target_tf(target_q, target_v);

    tf2::Quaternion reference_q;
    reference_q.setRPY(reference_pose.roll, reference_pose.pitch, reference_pose.yaw);
    tf2::Vector3 reference_v(reference_pose.x, reference_pose.y, reference_pose.z);
    tf2::Transform reference_tf(reference_q, reference_v);

    tf2::Transform trans_target_tf = reference_tf.inverse() * target_tf;

    Pose trans_target_pose{};
    trans_target_pose.x = trans_target_tf.getOrigin().getX();
    trans_target_pose.y = trans_target_tf.getOrigin().getY();
    trans_target_pose.z = trans_target_tf.getOrigin().getZ();
    tf2::Matrix3x3 tmp_m(trans_target_tf.getRotation());
    tmp_m.getRPY(trans_target_pose.roll, trans_target_pose.pitch, trans_target_pose.yaw);

    return trans_target_pose;
}

RateOfChange operator+(RateOfChange lhs, RateOfChange const& rhs) {
    lhs.absolute += rhs.absolute;
    lhs.linear_x += rhs.linear_x;
    lhs.linear_y += rhs.linear_y;
    lhs.linear_z += rhs.linear_z;
    lhs.angular_x += rhs.angular_x;
    lhs.angular_y += rhs.angular_y;
    lhs.angular_z += rhs.angular_z;
    return lhs;
}

RateOfChange operator-(RateOfChange lhs, RateOfChange const& rhs) {
    lhs.absolute -= rhs.absolute;
    lhs.linear_x -= rhs.linear_x;
    lhs.linear_y -= rhs.linear_y;
    lhs.linear_z -= rhs.linear_z;
    lhs.angular_x -= rhs.angular_x;
    lhs.angular_y -= rhs.angular_y;
    lhs.angular_z -= rhs.angular_z;
    return lhs;
}

RateOfChange operator/(RateOfChange lhs, double const& rhs) {
    lhs.absolute /= rhs;
    lhs.linear_x /= rhs;
    lhs.linear_y /= rhs;
    lhs.linear_z /= rhs;
    lhs.angular_x /= rhs;
    lhs.angular_y /= rhs;
    lhs.angular_z /= rhs;
    return lhs;
}

RateOfChange operator*(RateOfChange lhs, double const& rhs) {
    lhs.absolute *= rhs;
    lhs.linear_x *= rhs;
    lhs.linear_y *= rhs;
    lhs.linear_z *= rhs;
    lhs.angular_x *= rhs;
    lhs.angular_y *= rhs;
    lhs.angular_z *= rhs;
    return lhs;
}

geometry_msgs::msg::TwistStamped RateOfChange::ToTwistStamped(rclcpp::Time const& stamp, std::string const& frame_id) const {
    geometry_msgs::msg::TwistStamped msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    msg.twist.linear.x = linear_x;
    msg.twist.linear.y = linear_y;
    msg.twist.linear.z = linear_z;
    msg.twist.angular.x = angular_x;
    msg.twist.angular.y = angular_y;
    msg.twist.angular.z = angular_z;
    return msg;
}

RateOfChange RateOfChange::FromPoseDifference(Pose const& current, Pose const& previous, double dt) {
    RateOfChange result{};

    // Can only calculate if the time is real
    if (dt <= 0) {
        return result;
    }
    // Compute the velocity and acceleration
    Pose pose_diff = current - previous;

    double const distance = sqrt(pose_diff.x * pose_diff.x + pose_diff.y * pose_diff.y + pose_diff.z * pose_diff.z);
    Pose const trans_current_pose = Pose::ConvertPoseIntoRelativeCoordinate(current, previous);

    result.absolute = distance / dt;
    result.linear_x = pose_diff.x / dt;
    result.linear_y = pose_diff.y / dt;
    result.linear_z = pose_diff.z / dt;
    result.angular_x = pose_diff.roll / dt;
    result.angular_y = pose_diff.pitch / dt;
    result.angular_z = pose_diff.yaw / dt;
    // If the car moved rearwards (x < 0) in its relative position, we are driving backwards
    result.absolute = (trans_current_pose.x >= 0) ? result.absolute : -result.absolute;

    return result;
}

RateOfChange RateOfChange::FromDifference(RateOfChange const& current, RateOfChange const& previous, double dt) {
    RateOfChange result{};

    // Can only calculate if the time is real
    if (dt <= 0) {
        return result;
    }
    // Compute difference
    RateOfChange rate_of_change_difference = current - previous;

    result.absolute = rate_of_change_difference.absolute / dt;
    result.linear_x = rate_of_change_difference.linear_x / dt;
    result.linear_y = rate_of_change_difference.linear_y / dt;
    result.linear_z = rate_of_change_difference.linear_z / dt;
    result.angular_x = rate_of_change_difference.angular_x / dt;
    result.angular_y = rate_of_change_difference.angular_y / dt;
    result.angular_z = rate_of_change_difference.angular_z / dt;

    return result;
}

}  // namespace ndt

}  // namespace nie
