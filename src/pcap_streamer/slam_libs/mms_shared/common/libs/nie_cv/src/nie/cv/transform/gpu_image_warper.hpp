/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_TRANSFORM_GPU_IMAGE_WARPER_HPP
#define NIE_CV_TRANSFORM_GPU_IMAGE_WARPER_HPP

#include <opencv2/core.hpp>

#include "nie/cv/transform/helper_cuda.hpp"

namespace nie {

namespace gpu {

// General purpose image warper using a LUT and bi-linear interpolation.
class GpuImageWarper final {
public:
    GpuImageWarper(const cv::Mat& lut);

    void Warp(const cv::Mat& img_in, cv::Mat* img_out) const;

private:
    GpuBuffer2d<float2> gpu_lut_;
    GpuBuffer2d<uchar3> gpu_img_out_;
};

}  // namespace gpu

}  // namespace nie

#endif  // NIE_CV_TRANSFORM_GPU_IMAGE_WARPER_HPP
