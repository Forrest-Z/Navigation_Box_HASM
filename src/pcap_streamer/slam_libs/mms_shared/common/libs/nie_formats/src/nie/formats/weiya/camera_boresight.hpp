/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_WEIYA_CAMERA_BORESIGHT_HPP
#define NIE_FORMATS_WEIYA_CAMERA_BORESIGHT_HPP

#include <nie/core/geometry/isometry3.hpp>

namespace nie {

namespace io {

namespace weiya {

/// From the Weiya formats, the all.bs file contains the transformation from the left camera to the IMU. It transforms
/// points within the camera frame to the IMU frame. The translation is therefore the position of the camera from the
/// perspective of the IMU.
/// Practically speaking, it contains a rotation that brings OpenCV coordinates to Aircraft coordinates. The translation
/// is therefore in Aircraft coordinates.
class CameraBoresight {
public:
    static nie::Isometry3qd Read(std::string const& filename);
};

}  // namespace weiya

}  // namespace io

}  // namespace nie

#endif  // NIE_FORMATS_WEIYA_CAMERA_BORESIGHT_HPP
