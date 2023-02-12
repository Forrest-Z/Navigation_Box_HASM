/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "gpu_image_warper.hpp"

#include <cuda.h>
#include <cuda_texture_types.h>
#include <device_launch_parameters.h>
#include <texture_fetch_functions.h>
#include <texture_types.h>

// TODO(jbr) Maybe check for improved casting.

__global__ void BilinearInterpolationKernel(
    uchar3* __restrict__ out_data,
    const int out_pitch,
    const uchar3* __restrict__ in_data,
    const int in_pitch,
    const int in_width,
    const int in_height,
    const float2* __restrict__ lut_data,
    const int lut_pitch,
    const int lut_width,
    const int lut_height) {
    const int tx = threadIdx.x + blockDim.x * blockIdx.x;
    const int ty = threadIdx.y + blockDim.y * blockIdx.y;

    if ((tx < lut_width) && (ty < lut_height)) {
        const float2* lut_elem_ptr = ((float2*)((char*)lut_data + ty * lut_pitch) + tx);
        const int ix = (int)floor(lut_elem_ptr->x - 0.5f);
        const float a = lut_elem_ptr->x - 0.5f - ix;
        const int iy = (int)floor(lut_elem_ptr->y - 0.5f);
        const float b = lut_elem_ptr->y - 0.5f - iy;

        const uchar3 d00 = ((ix >= 0) && (iy >= 0) && (ix < in_width) && (iy < in_height))
                               ? *((uchar3*)((char*)in_data + iy * in_pitch) + ix)
                               : make_uchar3(0, 0, 0);
        const uchar3 d10 = ((ix + 1 >= 0) && (iy >= 0) && (ix + 1 < in_width) && (iy < in_height))
                               ? *((uchar3*)((char*)in_data + iy * in_pitch) + ix + 1)
                               : make_uchar3(0, 0, 0);
        const uchar3 d01 = ((ix >= 0) && (iy + 1 >= 0) && (ix < in_width) && (iy + 1 < in_height))
                               ? *((uchar3*)((char*)in_data + (iy + 1) * in_pitch) + ix)
                               : make_uchar3(0, 0, 0);
        const uchar3 d11 = ((ix + 1 >= 0) && (iy + 1 >= 0) && (ix + 1 < in_width) && (iy + 1 < in_height))
                               ? *((uchar3*)((char*)in_data + (iy + 1) * in_pitch) + ix + 1)
                               : make_uchar3(0, 0, 0);

        float3 tmp0, tmp1;

        tmp0.x = a * d10.x + (-d00.x * a + d00.x);
        tmp0.y = a * d10.y + (-d00.y * a + d00.y);
        tmp0.z = a * d10.z + (-d00.z * a + d00.z);

        tmp1.x = a * d11.x + (-d01.x * a + d01.x);
        tmp1.y = a * d11.y + (-d01.y * a + d01.y);
        tmp1.z = a * d11.z + (-d01.z * a + d01.z);

        uchar3* out_data_elem_ptr = ((uchar3*)((char*)out_data + ty * out_pitch) + tx);
        out_data_elem_ptr->x = b * tmp1.x + (-tmp0.x * b + tmp0.x);
        out_data_elem_ptr->y = b * tmp1.y + (-tmp0.y * b + tmp0.y);
        out_data_elem_ptr->z = b * tmp1.z + (-tmp0.z * b + tmp0.z);
    }
}

namespace nie {

namespace gpu {

GpuImageWarper::GpuImageWarper(const cv::Mat& lut) : gpu_lut_(lut.cols, lut.rows), gpu_img_out_(lut.cols, lut.rows) {
    gpuErrchk(cudaMemcpy2D(
        gpu_lut_.dev_ptr,
        gpu_lut_.pitch,
        lut.ptr<void>(),
        lut.step[0],                // including padding (like pitch)
        lut.cols * lut.elemSize(),  // no-padding
        lut.rows,
        cudaMemcpyHostToDevice));
}

void GpuImageWarper::Warp(const cv::Mat& img_in, cv::Mat* img_out) const {
    assert(img_in.channels() == 3);

    GpuBuffer2d<uchar3> gpu_img_in(img_in.cols, img_in.rows);
    gpuErrchk(cudaMemcpy2D(
        gpu_img_in.dev_ptr,
        gpu_img_in.pitch,
        img_in.ptr<void>(),
        img_in.step[0],
        img_in.cols * img_in.elemSize(),
        img_in.rows,
        cudaMemcpyHostToDevice));

    int block_size = 32;
    dim3 threads(block_size, block_size);
    dim3 blocks((gpu_lut_.width + (block_size - 1)) / block_size, (gpu_lut_.height + (block_size - 1)) / block_size);

    BilinearInterpolationKernel<<<blocks, threads>>>(
        gpu_img_out_.dev_ptr,
        gpu_img_out_.pitch,
        gpu_img_in.dev_ptr,
        gpu_img_in.pitch,
        gpu_img_in.width,
        gpu_img_in.height,
        gpu_lut_.dev_ptr,
        gpu_lut_.pitch,
        gpu_lut_.width,
        gpu_lut_.height);

    img_out->create(gpu_lut_.height, gpu_lut_.width, img_in.type());
    gpuErrchk(cudaMemcpy2D(
        img_out->ptr<void>(),
        img_out->step[0],
        gpu_img_out_.dev_ptr,
        gpu_img_out_.pitch,
        gpu_img_out_.byte_width(),
        gpu_img_out_.height,
        cudaMemcpyDeviceToHost));
}

}  // namespace gpu

}  // namespace nie
