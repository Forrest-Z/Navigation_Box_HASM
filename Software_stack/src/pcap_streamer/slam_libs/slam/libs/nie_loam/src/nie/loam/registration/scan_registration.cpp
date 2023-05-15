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

#include "scan_registration.hpp"
#include "nie/loam/utils/math_utils.hpp"

namespace loam {

void ScanRegistration::handleIMUMessage(loam::ImuMessage const& imuIn) {
    // rotate IMU data to lidar frame
    loam::ImuMessage imuInRotated = imuIn;
    if (_transformIMU) {
        imuInRotated = loam::ImuMessage();
        transformIMU(imuIn, imuInRotated, _T_lidar_imu);
    }

    // For ROS RPY = XYZ; Which is why we use eulerAngles(0,1,2);
    Eigen::Quaterniond orientation{imuInRotated.orientation};
    Eigen::Vector3d rpy_vec = orientation.toRotationMatrix().eulerAngles(0, 1, 2);
    double roll = rpy_vec[0];
    double pitch = rpy_vec[1];
    double yaw = rpy_vec[2];

    Vector3 acc;
    acc.x() = imuInRotated.linear_acceleration.y() - sin(roll) * cos(pitch) * 9.81;
    acc.y() = imuInRotated.linear_acceleration.z() - cos(roll) * cos(pitch) * 9.81;
    acc.z() = imuInRotated.linear_acceleration.x() + sin(pitch) * 9.81;

    IMUState newState;
    newState.stamp = imuInRotated.header.stamp;
    newState.roll = roll;
    newState.pitch = pitch;
    newState.yaw = yaw;
    newState.acceleration = acc;

    updateIMUData(acc, newState);
}

}  // end namespace loam
