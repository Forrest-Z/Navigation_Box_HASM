/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef LOAM_POINT_CLOUD_MESSAGE_H
#define LOAM_POINT_CLOUD_MESSAGE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "message_header.hpp"

namespace loam {

struct PointCloudMessage {
    MessageHeader header;
    pcl::PointCloud<pcl::PointXYZI> point_cloud;
};

}  // end namespace loam

#endif  // LOAM_POINT_CLOUD_MESSAGE_H
