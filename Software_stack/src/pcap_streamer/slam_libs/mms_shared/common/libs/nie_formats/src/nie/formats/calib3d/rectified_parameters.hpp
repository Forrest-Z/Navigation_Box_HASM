/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_CALIB3D_RECTIFIED_PARAMETERS_HPP
#define NIE_FORMATS_CALIB3D_RECTIFIED_PARAMETERS_HPP

#include <string>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <nie/core/geometry/isometry3.hpp>
#include "helper_calibration_io.hpp"

namespace nie {

namespace io {

struct FrameData {
    std::string id;
    nie::Isometry3qd baseline;

    bool operator==(FrameData const&) const;
};

struct RectifiedCameraParameters {
    static RectifiedCameraParameters Read(std::string const& filename);
    void Write(std::string const& filename) const;

    bool operator==(RectifiedCameraParameters const&) const;

    std::string id;
    Eigen::Vector2i image_size;  // width, height
    Eigen::Matrix3d K;
    std::vector<FrameData> frames;
};

void Read(std::string const& filename, std::vector<RectifiedCameraParameters>*);
void Write(std::string const& filename, std::vector<RectifiedCameraParameters> const&);

}  // namespace io

}  // namespace nie

#endif  // NIE_FORMATS_CALIB3D_RECTIFIED_PARAMETERS_HPP
