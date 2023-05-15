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

#ifndef LOAM_LASERODOMETRY_H
#define LOAM_LASERODOMETRY_H

#include "basic_laser_odometry.hpp"
#include "nie/loam/ros_interop/odometry.hpp"
#include "nie/loam/ros_interop/point_cloud_message.hpp"
#include "nie/loam/ros_interop/transform_stamped.hpp"
#include "nie/loam/ros_interop/twist.hpp"
#include "nie/loam/utils/nanoflann_pcl.hpp"

namespace loam {

/** \brief Implementation of the LOAM laser odometry component.
 *
 */
class LaserOdometry : public BasicLaserOdometry {
public:
    explicit LaserOdometry(float scanPeriod = 0.1, uint16_t ioRatio = 2, size_t maxIterations = 25);

    /** \brief Handler method for a new sharp corner cloud.
     *
     * @param cornerPointsSharpMsg the new sharp corner cloud message
     */
    void laserCloudSharpHandler(const loam::PointCloudMessage& cornerPointsSharpMsg);

    /** \brief Handler method for a new less sharp corner cloud.
     *
     * @param cornerPointsLessSharpMsg the new less sharp corner cloud message
     */
    void laserCloudLessSharpHandler(const loam::PointCloudMessage& cornerPointsLessSharpMsg);

    /** \brief Handler method for a new flat surface cloud.
     *
     * @param surfPointsFlatMsg the new flat surface cloud message
     */
    void laserCloudFlatHandler(const loam::PointCloudMessage& surfPointsFlatMsg);

    /** \brief Handler method for a new less flat surface cloud.
     *
     * @param surfPointsLessFlatMsg the new less flat surface cloud message
     */
    void laserCloudLessFlatHandler(const loam::PointCloudMessage& surfPointsLessFlatMsg);

    /** \brief Handler method for a new full resolution cloud.
     *
     * @param laserCloudFullResMsg the new full resolution cloud message
     */
    void laserCloudFullResHandler(const loam::PointCloudMessage& laserCloudFullResMsg);

    /** \brief Handler method for a new IMU transformation information.
     *
     * @param laserCloudFullResMsg the new IMU transformation information message
     */
    void imuTransHandler(const loam::PointCloudMessage& imuTransMsg);

    /** \brief Try to process buffered data. */
    void process();

    /** \brief Retreive the lidar odometry results. */
    Odometry const& laserOdometry() const { return _laserOdometryMsg; }

protected:
    /** \brief Reset flags, etc. */
    void reset();

    /** \brief Check if all required information for a new processing step is
     * available. */
    bool hasNewData();

    /** \brief Publish the current result via the respective topics. */
    void publishResult();

private:
    uint16_t _ioRatio;  ///< ratio of input to output frames

    nie::Timestamp_ns _timeCornerPointsSharp;      ///< time of current sharp corner cloud
    nie::Timestamp_ns _timeCornerPointsLessSharp;  ///< time of current less sharp corner cloud
    nie::Timestamp_ns _timeSurfPointsFlat;         ///< time of current flat surface cloud
    nie::Timestamp_ns _timeSurfPointsLessFlat;     ///< time of current less flat surface cloud
    nie::Timestamp_ns _timeLaserCloudFullRes;      ///< time of current full resolution cloud
    nie::Timestamp_ns _timeImuTrans;               ///< time of current IMU transformation information

    bool _newCornerPointsSharp;      ///< flag if a new sharp corner cloud has been
                                     ///< received
    bool _newCornerPointsLessSharp;  ///< flag if a new less sharp corner cloud has
                                     ///< been received
    bool _newSurfPointsFlat;         ///< flag if a new flat surface cloud has been
                                     ///< received
    bool _newSurfPointsLessFlat;     ///< flag if a new less flat surface cloud has
                                     ///< been received
    bool _newLaserCloudFullRes;      ///< flag if a new full resolution cloud has been
                                     ///< received
    bool _newImuTrans;               ///< flag if a new IMU transformation information cloud has
                                     ///< been received

private:
    Odometry _laserOdometryMsg;
    PointCloudMessage _cloudFullResMessage;
    PointCloudMessage _cloudCornerLastMessage;
    PointCloudMessage _cloudSurfLastMessage;
};

}  // end namespace loam

#endif  // LOAM_LASERODOMETRY_H
