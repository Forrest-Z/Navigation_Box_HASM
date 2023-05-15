/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_CV_TRANSFORM_HELPER_CUDA_HPP
#define NIE_CV_TRANSFORM_HELPER_CUDA_HPP

#include <iostream>

#include <cuda_runtime.h>

namespace nie {

namespace gpu {

#define gpuErrchk(ans) \
    { gpuAssert((ans), __FILE__, __LINE__); }

inline void gpuAssert(cudaError_t code, const char* file, int line, bool abort = true) {
    if (code != cudaSuccess) {
        std::cerr << "GPUassert: " << cudaGetErrorString(code) << file << line;

        if (abort) {
            exit(code);
        }
    }
}

// RAII style buffer on the GPU
template <typename T>
class GpuBuffer2d {
public:
    using type_t = T;

    GpuBuffer2d(int width, int height);
    GpuBuffer2d(const GpuBuffer2d&) = delete;
    GpuBuffer2d(GpuBuffer2d&&) = default;

    ~GpuBuffer2d();

    GpuBuffer2d& operator=(const GpuBuffer2d&) = delete;
    GpuBuffer2d& operator=(GpuBuffer2d&&) = default;

    std::size_t byte_width() const;

    int width;
    int height;
    std::size_t pitch;
    T* dev_ptr;
};

template <typename T>
GpuBuffer2d<T>::GpuBuffer2d(int width, int height) : width(width), height(height) {
    gpuErrchk(cudaMallocPitch(reinterpret_cast<void**>(&dev_ptr), &pitch, width * sizeof(T), height));
}

template <typename T>
GpuBuffer2d<T>::~GpuBuffer2d() {
    gpuErrchk(cudaFree(dev_ptr));
}

template <typename T>
std::size_t GpuBuffer2d<T>::byte_width() const {
    return width * sizeof(T);
}

}  // namespace gpu

}  // namespace nie

#endif  // NIE_CV_TRANSFORM_HELPER_CUDA_HPP
