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

#include <pcl/common/common.h>
#include <pcl/filters/filter.h>

#include "laser_odometry.hpp"
#include "nie/loam/ros_interop/odometry.hpp"
#include "nie/loam/utils/math_utils.hpp"

namespace loam {

using std::asin;
using std::atan2;
using std::cos;
using std::fabs;
using std::pow;
using std::sin;
using std::sqrt;

LaserOdometry::LaserOdometry(float scanPeriod, uint16_t ioRatio, size_t maxIterations)
    : BasicLaserOdometry(scanPeriod, maxIterations), _ioRatio(ioRatio) {}

void LaserOdometry::reset() {
    _newCornerPointsSharp = false;
    _newCornerPointsLessSharp = false;
    _newSurfPointsFlat = false;
    _newSurfPointsLessFlat = false;
    _newLaserCloudFullRes = false;
    _newImuTrans = false;
}

void LaserOdometry::laserCloudSharpHandler(const loam::PointCloudMessage& cornerPointsSharpMsg) {
    _timeCornerPointsSharp = cornerPointsSharpMsg.header.stamp;
    cornerPointsSharp() = cornerPointsSharpMsg.point_cloud.makeShared();
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cornerPointsSharp(), *cornerPointsSharp(), indices);
    _newCornerPointsSharp = true;
}

void LaserOdometry::laserCloudLessSharpHandler(const loam::PointCloudMessage& cornerPointsLessSharpMsg) {
    _timeCornerPointsLessSharp = cornerPointsLessSharpMsg.header.stamp;
    cornerPointsLessSharp() = cornerPointsLessSharpMsg.point_cloud.makeShared();
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cornerPointsLessSharp(), *cornerPointsLessSharp(), indices);
    _newCornerPointsLessSharp = true;
}

void LaserOdometry::laserCloudFlatHandler(const loam::PointCloudMessage& surfPointsFlatMsg) {
    _timeSurfPointsFlat = surfPointsFlatMsg.header.stamp;
    surfPointsFlat() = surfPointsFlatMsg.point_cloud.makeShared();
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*surfPointsFlat(), *surfPointsFlat(), indices);
    _newSurfPointsFlat = true;
}

void LaserOdometry::laserCloudLessFlatHandler(const loam::PointCloudMessage& surfPointsLessFlatMsg) {
    _timeSurfPointsLessFlat = surfPointsLessFlatMsg.header.stamp;
    surfPointsLessFlat() = surfPointsLessFlatMsg.point_cloud.makeShared();
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*surfPointsLessFlat(), *surfPointsLessFlat(), indices);
    _newSurfPointsLessFlat = true;
}

void LaserOdometry::laserCloudFullResHandler(const loam::PointCloudMessage& laserCloudFullResMsg) {
    _timeLaserCloudFullRes = laserCloudFullResMsg.header.stamp;
    laserCloud() = laserCloudFullResMsg.point_cloud.makeShared();
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*laserCloud(), *laserCloud(), indices);
    _newLaserCloudFullRes = true;
}

void LaserOdometry::imuTransHandler(const loam::PointCloudMessage& imuTransMsg) {
    _timeImuTrans = imuTransMsg.header.stamp;
    updateIMU(imuTransMsg.point_cloud);
    _newImuTrans = true;
}

bool LaserOdometry::hasNewData() {
    return _newCornerPointsSharp && _newCornerPointsLessSharp && _newSurfPointsFlat && _newSurfPointsLessFlat &&
           _newLaserCloudFullRes && _newImuTrans &&
           fabs((_timeCornerPointsSharp - _timeSurfPointsLessFlat).count()) < 5e6 &&
           fabs((_timeCornerPointsLessSharp - _timeSurfPointsLessFlat).count()) < 5e6 &&
           fabs((_timeSurfPointsFlat - _timeSurfPointsLessFlat).count()) < 5e6 &&
           fabs((_timeLaserCloudFullRes - _timeSurfPointsLessFlat).count()) < 5e6 &&
           fabs((_timeImuTrans - _timeSurfPointsLessFlat).count()) < 5e6;
}

void LaserOdometry::process() {
    if (!hasNewData()) return;  // waiting for new data to arrive...

    reset();  // reset flags, etc.
    BasicLaserOdometry::process();
    publishResult();
}

void LaserOdometry::publishResult() {
    // Original conversion -- remember that in ROS RPY = XYZ -- toQuaternion does the same
    // Eigen::Quaterniond geoQuat = Eigen::Quaterniond::
    // tf::createQuaternionMsgFromRollPitchYaw(transformSum().rot_z.rad(), -transformSum().rot_x.rad(),
    // -transformSum().rot_y.rad());

    Eigen::Quaterniond geoQuat = transformSum().getQuaternion();

    _laserOdometryMsg.header.seq = -1;
    _laserOdometryMsg.header.stamp = _timeSurfPointsLessFlat;
    _laserOdometryMsg.header.frame_id = "init";
    _laserOdometryMsg.child_frame = "laser_odom_frame";
    _laserOdometryMsg.pose.pose.orientation.x() = -geoQuat.y();
    _laserOdometryMsg.pose.pose.orientation.y() = -geoQuat.z();
    _laserOdometryMsg.pose.pose.orientation.z() = geoQuat.x();
    _laserOdometryMsg.pose.pose.orientation.w() = geoQuat.w();
    _laserOdometryMsg.pose.pose.position.x() = transformSum().pos.x();
    _laserOdometryMsg.pose.pose.position.y() = transformSum().pos.y();
    _laserOdometryMsg.pose.pose.position.z() = transformSum().pos.z();

    // Instead of publishing, whoever needs this can just grab it through the member?

    nie::Timestamp_ns sweepTime = _timeSurfPointsLessFlat;

    // laserCloud() will now hold the full point cloud as the rest of the pipeline expects
    transformToEnd(laserCloud());  // transform full resolution cloud to sweep end before sending it

    // Construct these messages which are not really messages anymore -- how to give these to whoever uses them?
    _cloudFullResMessage = {MessageHeader(-1, sweepTime, "lidar_frame"), *laserCloud()};
    _cloudCornerLastMessage = {MessageHeader(-1, sweepTime, "lidar_frame"), *lastCornerCloud()};
    _cloudSurfLastMessage = {MessageHeader(-1, sweepTime, "lidar_frame"), *lastSurfaceCloud()};
}

}  // end namespace loam
