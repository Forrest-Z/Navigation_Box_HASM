/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifdef USE_CUDA
#include <gtest/gtest.h>
#include <nie/cv/transform/helper_cuda.hpp>

class HelperCudaTest : public testing::Test {
protected:
    int const kImageWidth = 640;
    int const kImageHeight = 480;
    char const kDummyFilename[9] = "dummy.cu";
};

TEST_F(HelperCudaTest, gpuAssert) {
    testing::FLAGS_gtest_death_test_style = "threadsafe";
    EXPECT_NO_FATAL_FAILURE(nie::gpu::gpuAssert(cudaError::cudaSuccess, kDummyFilename, 1, false));
    EXPECT_NO_FATAL_FAILURE(nie::gpu::gpuAssert(cudaError::cudaErrorMissingConfiguration, kDummyFilename, 1, false));
    EXPECT_EXIT(
        nie::gpu::gpuAssert(cudaError::cudaErrorMissingConfiguration, kDummyFilename, 1, true),
        testing::internal::ExitedUnsuccessfully,
        "GPUassert:.*");
}

TEST_F(HelperCudaTest, GpuBuffer2dConstruction) {
    EXPECT_NO_FATAL_FAILURE(
        nie::gpu::GpuBuffer2d<float2> buffer{nie::gpu::GpuBuffer2d<float2>(kImageWidth, kImageHeight)});
    EXPECT_NO_THROW(nie::gpu::GpuBuffer2d<float2> buffer{nie::gpu::GpuBuffer2d<float2>(kImageWidth, kImageHeight)});
    EXPECT_NO_FATAL_FAILURE(
        nie::gpu::GpuBuffer2d<uchar3> buffer{nie::gpu::GpuBuffer2d<uchar3>(kImageWidth, kImageHeight)});
    EXPECT_NO_THROW(nie::gpu::GpuBuffer2d<uchar3> buffer{nie::gpu::GpuBuffer2d<uchar3>(kImageWidth, kImageHeight)});
}

TEST_F(HelperCudaTest, GpusBuffer2dByteWidth) {
    auto buffer{nie::gpu::GpuBuffer2d<float2>(kImageWidth, kImageHeight)};
    ASSERT_EQ(kImageWidth * sizeof(float2), buffer.byte_width());
}

TEST_F(HelperCudaTest, GpusBuffer2dPitch) {
    // We found out the hard way that the pitch of the buffer (actual width in
    // memory) is not always the same as the byte_width() of the buffer. Here we
    // test the two cases, we can not test for the exact pitch value because
    // that depends on the hardware
    auto buffer_divisible{nie::gpu::GpuBuffer2d<float2>(kImageWidth, kImageHeight)};
    ASSERT_EQ(buffer_divisible.pitch, buffer_divisible.byte_width());
    // we add 1 specificaly to make this go over one 'line'
    auto buffer_padded{nie::gpu::GpuBuffer2d<float2>(kImageWidth + 1, kImageHeight)};
    ASSERT_NE(buffer_padded.pitch, buffer_padded.byte_width());
}
#endif
