/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <pcl/point_types.h>
#include <Eigen/Eigen>

namespace nie {

inline pcl::PointXYZ MakePointXYZ(Eigen::Vector3d const& p) {
    return {static_cast<float>(p(0)), static_cast<float>(p(1)), static_cast<float>(p(2))};
}

inline pcl::PointXYZ& operator+=(pcl::PointXYZ& l, pcl::PointXYZ const& r) {
    l.getVector3fMap() += r.getVector3fMap();
    return l;
}

inline pcl::PointXYZ& operator-=(pcl::PointXYZ& l, pcl::PointXYZ const& r) {
    l.getVector3fMap() -= r.getVector3fMap();
    return l;
}

inline pcl::PointXYZ& operator*=(pcl::PointXYZ& l, float r) {
    l.getVector3fMap() *= r;
    return l;
}

inline pcl::PointXYZ operator+(pcl::PointXYZ const& l, pcl::PointXYZ const& r) {
    pcl::PointXYZ n = l;
    return n += r;
}

inline pcl::PointXYZ operator-(pcl::PointXYZ const& l, pcl::PointXYZ const& r) {
    pcl::PointXYZ n = l;
    return n -= r;
}

inline pcl::PointXYZ operator*(pcl::PointXYZ const& l, float r) {
    pcl::PointXYZ n = l;
    return n *= r;
}

inline pcl::PointXYZ operator-(pcl::PointXYZ const& p) {
    pcl::PointXYZ n;
    n.getVector3fMap() = -p.getVector3fMap();
    return n;
}

}  // namespace nie
