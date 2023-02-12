/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef LOAM_TWIST_H
#define LOAM_TWIST_H

#include <Eigen/Geometry>

#include "angle.hpp"
#include "vector3.hpp"

namespace loam {

/** \brief Twist composed by three angles and a three-dimensional position.
 *
 */
class Twist {
public:
    Twist() : rot_x(), rot_y(), rot_z(), pos(){};

    Angle rot_x;
    Angle rot_y;
    Angle rot_z;
    Vector3 pos;

    static Eigen::Quaterniond getQuaternion(Angle const& rot_x, Angle const& rot_y, Angle const& rot_z) {
        return Eigen::AngleAxisd(-rot_y.rad(), Eigen::Vector3d::UnitZ()) *
               Eigen::AngleAxisd(-rot_x.rad(), Eigen::Vector3d::UnitY()) *
               Eigen::AngleAxisd(rot_z.rad(), Eigen::Vector3d::UnitX());
    }

    Eigen::Quaterniond getQuaternion() const { return Twist::getQuaternion(this->rot_x, this->rot_y, this->rot_z); }

    void updateFromQuaternion(Eigen::Quaterniond const& geoQuat) {
        Eigen::Vector3d xyz_vec = geoQuat.toRotationMatrix().eulerAngles(2, 1, 0);
        double roll = xyz_vec[2];
        double pitch = xyz_vec[1];
        double yaw = xyz_vec[0];
        rot_x = -pitch;
        rot_y = -yaw;
        rot_z = roll;
    }
};

}  // end namespace loam

#endif  // LOAM_TWIST_H
