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

#include "multi_scan_registration.hpp"
#include <pcl/common/io.h>
#include "nie/loam/utils/math_utils.hpp"

namespace loam {

MultiScanMapper::MultiScanMapper(const float& lowerBound, const float& upperBound, const uint16_t& nScanRings)
    : _lowerBound(lowerBound),
      _upperBound(upperBound),
      _nScanRings(nScanRings),
      _factor((nScanRings - 1) / (upperBound - lowerBound)) {}

void MultiScanMapper::set(const float& lowerBound, const float& upperBound, const uint16_t& nScanRings) {
    _lowerBound = lowerBound;
    _upperBound = upperBound;
    _nScanRings = nScanRings;
    _factor = (nScanRings - 1) / (upperBound - lowerBound);
}

int MultiScanMapper::getRingForAngle(const float& angle) {
    return int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5);
}

MultiScanRegistration::MultiScanRegistration(const MultiScanMapper& scanMapper) : _scanMapper(scanMapper) {}

void MultiScanRegistration::handleCloudMessage(PointCloudMessage const& laserCloudMsg) {
    if (_systemDelay > 0) {
        --_systemDelay;
        return;
    }

    // fetch new input cloud
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    pcl::copyPointCloud(laserCloudMsg.point_cloud, laserCloudIn);
    process(laserCloudIn, laserCloudMsg.header.stamp);
}

void MultiScanRegistration::process(
        const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn, const nie::Timestamp_ns& scanTime) {
    size_t cloudSize = laserCloudIn.size();

    // determine scan start and end orientations
    float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);
    float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y, laserCloudIn[cloudSize - 1].x) + 2 * float(M_PI);
    if (endOri - startOri > 3 * M_PI) {
        endOri -= 2 * M_PI;
    } else if (endOri - startOri < M_PI) {
        endOri += 2 * M_PI;
    }

    bool halfPassed = false;
    pcl::PointXYZI point;
    _laserCloudScans.resize(_scanMapper.getNumberOfScanRings());
    // clear all scanline points
    std::for_each(_laserCloudScans.begin(), _laserCloudScans.end(), [](auto&& v) { v.clear(); });

    // extract valid points from input cloud
    for (size_t i = 0; i < cloudSize; i++) {
        point.x = laserCloudIn[i].x;
        point.y = laserCloudIn[i].y;
        point.z = laserCloudIn[i].z;

        // skip NaN and INF valued points
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
        }

        // skip zero valued points
        if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
            continue;
        }

        // calculate vertical point angle and scan ID
        float angle = std::atan(point.z / std::sqrt(point.y * point.y + point.x * point.x));
        int scanID = _scanMapper.getRingForAngle(angle);
        if (scanID >= _scanMapper.getNumberOfScanRings() || scanID < 0) {
            continue;
        }

        // calculate horizontal point angle
        float ori = -std::atan2(point.y, point.x);
        if (!halfPassed) {
            if (ori < startOri - M_PI / 2) {
                ori += 2 * M_PI;
            } else if (ori > startOri + M_PI * 3 / 2) {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI) {
                halfPassed = true;
            }
        } else {
            ori += 2 * M_PI;

            if (ori < endOri - M_PI * 3 / 2) {
                ori += 2 * M_PI;
            } else if (ori > endOri + M_PI / 2) {
                ori -= 2 * M_PI;
            }
        }

        // calculate relative scan time based on point orientation
        float relTime = config().scanPeriod * (ori - startOri) / (endOri - startOri);
        point.intensity = scanID + relTime;

        projectPointToStartOfSweep(point, relTime);

        _laserCloudScans[scanID].push_back(point);
    }

    processScanlines(scanTime, _laserCloudScans);
}

}  // end namespace loam
