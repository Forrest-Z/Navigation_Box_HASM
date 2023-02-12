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

#ifndef LOAM_TRANSFORMMAINTENANCE_H
#define LOAM_TRANSFORMMAINTENANCE_H

#include "basic_transform_maintenance.hpp"
#include "nie/loam/ros_interop/odometry.hpp"

namespace loam {

/** \brief Implementation of the LOAM transformation maintenance component.
 *
 */
class TransformMaintenance : public BasicTransformMaintenance {
public:
    TransformMaintenance();

    /** \brief Handler method for laser odometry messages.
     *
     * @param laserOdometry the new laser odometry
     */
    void laserOdometryHandler(const Odometry& laserOdometry);

    /** \brief Handler method for mapping odometry messages.
     *
     * @param odomAftMapped the new mapping odometry
     */
    void odomAftMappedHandler(const Odometry& odomAftMapped);

    Odometry const& integratedOdometry() const { return _laserOdometry2; }

private:
    Odometry _laserOdometry2;  ///< latest integrated laser odometry message
    std::string _mapOdomTopic, _loamOdomTopic, _lidarOdomTopic, _lidarFrame, _initFrame;
    std::vector<double> _poseCovariance{0.001, 0.001, 0.001, 0.001, 0.001, 0.001};
};

}  // end namespace loam

#endif  // LOAM_TRANSFORMMAINTENANCE_H
