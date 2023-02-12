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

#ifndef LOAM_LASERMAPPING_H
#define LOAM_LASERMAPPING_H

#include "basic_laser_mapping.hpp"

#include "nie/loam/ros_interop/imu_message.hpp"
#include "nie/loam/ros_interop/odometry.hpp"
#include "nie/loam/ros_interop/point_cloud_message.hpp"

namespace loam {
/** \brief Implementation of the LOAM laser mapping component.
 *
 */
class LaserMapping : public BasicLaserMapping {
public:
    explicit LaserMapping(const float& scanPeriod = 0.1, const size_t& maxIterations = 10);

    /** \brief Handler method for a new last corner cloud.
     *
     * @param cornerPointsLastMsg the new last corner cloud message
     */
    void laserCloudCornerLastHandler(const PointCloudMessage& cornerPointsLastMsg);

    /** \brief Handler method for a new last surface cloud.
     *
     * @param surfacePointsLastMsg the new last surface cloud message
     */
    void laserCloudSurfLastHandler(const PointCloudMessage& surfacePointsLastMsg);

    /** \brief Handler method for a new full resolution cloud.
     *
     * @param laserCloudFullResMsg the new full resolution cloud message
     */
    void laserCloudFullResHandler(const PointCloudMessage& laserCloudFullResMsg);

    /** \brief Handler method for a new laser odometry.
     *
     * @param laserOdometry the new laser odometry message
     */
    void laserOdometryHandler(const Odometry& laserOdometry);

    /** \brief Handler method for IMU messages.
     *
     * @param imuIn the new IMU message
     */
    void imuHandler(const ImuMessage& imuIn);

    /** \brief Try to process buffered data. */
    void process();

    Odometry const& odomAftMapped() const { return _odomAftMapped; }

protected:
    /** \brief Reset flags, etc. */
    void reset();

    /** \brief Check if all required information for a new processing step is
     * available. */
    bool hasNewData();

    /** \brief Publish the current result via the respective topics. */
    void publishResult();

private:
    nie::Timestamp_ns _timeLaserCloudCornerLast;  ///< time of current last corner cloud
    nie::Timestamp_ns _timeLaserCloudSurfLast;    ///< time of current last surface cloud
    nie::Timestamp_ns _timeLaserCloudFullRes;     ///< time of current full resolution cloud
    nie::Timestamp_ns _timeLaserOdometry;         ///< time of current laser odometry

    bool _newLaserCloudCornerLast;  ///< flag if a new last corner cloud has been
                                    ///< received
    bool _newLaserCloudSurfLast;    ///< flag if a new last surface cloud has been
                                    ///< received
    bool _newLaserCloudFullRes;     ///< flag if a new full resolution cloud has been
                                    ///< received
    bool _newLaserOdometry;         ///< flag if a new laser odometry has been received

    Odometry _odomAftMapped;  ///< mapping odometry message

    std::string _initFrame, _mapFrame;
};

}  // end namespace loam

#endif  // LOAM_LASERMAPPING_H
