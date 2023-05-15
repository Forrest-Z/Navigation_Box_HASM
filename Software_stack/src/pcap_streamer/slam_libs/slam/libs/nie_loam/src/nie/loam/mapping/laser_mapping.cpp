// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include "laser_mapping.hpp"

#include <nie/core/geometry/isometry3.hpp>
#include <nie/core/geometry/rotation.hpp>

namespace loam {

LaserMapping::LaserMapping(const float& /*scanPeriod*/, const size_t& /*maxIterations*/) {
    _initFrame = "init";
    _mapFrame = "map";

    _odomAftMapped.header.frame_id = _initFrame;
    _odomAftMapped.child_frame = _mapFrame;
}

void LaserMapping::laserCloudCornerLastHandler(const PointCloudMessage& cornerPointsLastMsg) {
    _timeLaserCloudCornerLast = cornerPointsLastMsg.header.stamp;
    laserCloudCornerLast() = cornerPointsLastMsg.point_cloud;
    _newLaserCloudCornerLast = true;
}

void LaserMapping::laserCloudSurfLastHandler(const PointCloudMessage& surfacePointsLastMsg) {
    _timeLaserCloudSurfLast = surfacePointsLastMsg.header.stamp;
    laserCloudSurfLast() = surfacePointsLastMsg.point_cloud;
    _newLaserCloudSurfLast = true;
}

void LaserMapping::laserCloudFullResHandler(const PointCloudMessage& laserCloudFullResMsg) {
    _timeLaserCloudFullRes = laserCloudFullResMsg.header.stamp;
    laserCloud() = laserCloudFullResMsg.point_cloud;
    _newLaserCloudFullRes = true;
}

void LaserMapping::laserOdometryHandler(const Odometry& laserOdometry) {
    _timeLaserOdometry = laserOdometry.header.stamp;

    Eigen::Quaterniond rosQuat = laserOdometry.pose.pose.orientation;
    Eigen::Quaterniond geoQuat = Eigen::Quaterniond(rosQuat.w(), rosQuat.z(), -rosQuat.x(), -rosQuat.y());

    updateOdometry(
        geoQuat,
        laserOdometry.pose.pose.position.x(),
        laserOdometry.pose.pose.position.y(),
        laserOdometry.pose.pose.position.z());

    _newLaserOdometry = true;
}

void LaserMapping::imuHandler(const ImuMessage& imuIn) {
    Eigen::Vector3d rpy_vec = imuIn.orientation.toRotationMatrix().eulerAngles(0, 1, 2);
    updateIMU({imuIn.header.stamp, rpy_vec[0], rpy_vec[1]});
}

void LaserMapping::reset() {
    _newLaserCloudCornerLast = false;
    _newLaserCloudSurfLast = false;
    _newLaserCloudFullRes = false;
    _newLaserOdometry = false;
}

bool LaserMapping::hasNewData() {
    return _newLaserCloudCornerLast && _newLaserCloudSurfLast && _newLaserCloudFullRes && _newLaserOdometry;
}

void LaserMapping::process() {
    if (!hasNewData())  // waiting for new data to arrive...
        return;

    reset();  // reset flags, etc.

    if (!BasicLaserMapping::process(_timeLaserOdometry)) return;

    publishResult();
}

void LaserMapping::publishResult() {
    // "Surround" and "full" point clouds are ready to go after this.
    if (hasFreshMap())  // publish new map cloud
        PointCloudMessage LaserCloudSurroudMessage{
            MessageHeader(-1, _timeLaserOdometry, _odomAftMapped.header.frame_id), laserCloudSurroundDS()};
    PointCloudMessage LaserCloudFullResMessage{MessageHeader(-1, _timeLaserOdometry, _odomAftMapped.header.frame_id),
                                               laserCloud()};

    Eigen::Quaterniond geoQuat = transformAftMapped().getQuaternion();

    auto prevOdomAftMapped = _odomAftMapped;

    _odomAftMapped.header.stamp = _timeLaserOdometry;
    _odomAftMapped.pose.pose.orientation.x() = -geoQuat.y();
    _odomAftMapped.pose.pose.orientation.y() = -geoQuat.z();
    _odomAftMapped.pose.pose.orientation.z() = geoQuat.x();
    _odomAftMapped.pose.pose.orientation.w() = geoQuat.w();
    _odomAftMapped.pose.pose.position.x() = transformAftMapped().pos.x();
    _odomAftMapped.pose.pose.position.y() = transformAftMapped().pos.y();
    _odomAftMapped.pose.pose.position.z() = transformAftMapped().pos.z();
    _odomAftMapped.twist.twist.angular.x() = transformBefMapped().rot_x.rad();
    _odomAftMapped.twist.twist.angular.y() = transformBefMapped().rot_y.rad();
    _odomAftMapped.twist.twist.angular.z() = transformBefMapped().rot_z.rad();
    _odomAftMapped.twist.twist.linear.x() = transformBefMapped().pos.x();
    _odomAftMapped.twist.twist.linear.y() = transformBefMapped().pos.y();
    _odomAftMapped.twist.twist.linear.z() = transformBefMapped().pos.z();

    nie::Isometry3Mapqd curr_odom_map(
        _odomAftMapped.pose.pose.position.data(), _odomAftMapped.pose.pose.orientation.coeffs().data());
    nie::Isometry3Mapqd prev_odom_map(
        prevOdomAftMapped.pose.pose.position.data(), prevOdomAftMapped.pose.pose.orientation.coeffs().data());

    // Defined as T_prev^-1 * T_after
    nie::Isometry3qd const odometry_delta = prev_odom_map.TransformInverseLeft(curr_odom_map);

    // We assume a drift of 2.5% over the length of the motion, as the original paper says.
    double const motion_sd = odometry_delta.translation().norm() * 0.025;
    double const motion_variance = std::pow(motion_sd, 2);

    Eigen::Map<Eigen::Matrix<double, 6, 6>>(_odomAftMapped.pose.covariance.data())
        .diagonal()
        .setConstant(motion_variance);

    // odomAfterMapping is ready to go now
}

}  // end namespace loam
