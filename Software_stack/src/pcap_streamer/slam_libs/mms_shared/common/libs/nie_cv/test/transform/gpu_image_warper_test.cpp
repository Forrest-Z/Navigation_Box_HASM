/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifdef USE_CUDA
#include <atomic>

#include <gtest/gtest.h>
#include <nie/cv/transform/gpu_image_warper.hpp>
#include <opencv2/core.hpp>

class GpuImageWarperTest : public testing::Test {
protected:
    cv::Mat const control_lut_shrink = cv::Mat::ones(1, 1, CV_32FC2);
    cv::Mat const control_lut_enlarge = cv::Mat::ones(4, 4, CV_32FC2);

    // the following data represents a three channel 2x2 red/blue checkerboard:
    //                         |R|B|
    //                         |B|R|
    // we use 252 instead of 255 so that we get an exact value on the remapped
    // image (63, 0, 63) and do not rely on any specific rounding behavior
    unsigned char input_image22_data[12]{0, 0, 252, 252, 0, 0, 252, 0, 0, 0, 0, 252};
    cv::Vec3b const control_output_pixel{63, 0, 63};
    cv::Mat const control_input_image22{2, 2, CV_8UC3, input_image22_data};  // create mat from the data
};

TEST_F(GpuImageWarperTest, Lifecycle) {
    nie::gpu::GpuImageWarper* giw = nullptr;
    ASSERT_NO_THROW(giw = new nie::gpu::GpuImageWarper(control_lut_shrink));
    ASSERT_NO_THROW(delete giw);
    ASSERT_NO_FATAL_FAILURE(giw = new nie::gpu::GpuImageWarper(control_lut_shrink));
    ASSERT_NO_FATAL_FAILURE(delete giw);
}

TEST_F(GpuImageWarperTest, Reduce) {
    nie::gpu::GpuImageWarper giw(control_lut_shrink);
    cv::Mat experimental_image{control_input_image22.rows, control_input_image22.cols, control_input_image22.type()};
    ASSERT_NO_THROW(giw.Warp(control_input_image22, &experimental_image));
    ASSERT_EQ(experimental_image.at<cv::Vec3b>(0, 0), control_output_pixel);
}

TEST_F(GpuImageWarperTest, Enlarge) {
    nie::gpu::GpuImageWarper giw(control_lut_enlarge);
    cv::Mat experimental_image{control_input_image22.rows, control_input_image22.cols, control_input_image22.type()};
    ASSERT_NO_THROW(giw.Warp(control_input_image22, &experimental_image));
    std::atomic_bool image_ok{true};
    // all the pixels should have our 'magic' purple color
    experimental_image.forEach<cv::Vec3b>([&](cv::Vec3b& pixel, const int []) -> void {
        if (pixel != control_output_pixel) {
            image_ok = false;
        }
    });
    ASSERT_TRUE(image_ok);
}

TEST_F(GpuImageWarperTest, WarpThrow) {
    // gtest detected 4 running threads for this test, and adviced us to use the
    // the 'threadsafe' death test style, which does not call fork()
    testing::FLAGS_gtest_death_test_style = "threadsafe";
    nie::gpu::GpuImageWarper giw(control_lut_shrink);
    // should die if input_image.channels() != 3
    cv::Mat experimental_onechannel_image{control_input_image22.rows, control_input_image22.cols, CV_8UC1};
    ASSERT_DEBUG_DEATH(giw.Warp(experimental_onechannel_image, &experimental_onechannel_image), ".*");
    cv::Mat experimental_twochannel_image{control_input_image22.rows, control_input_image22.cols, CV_8UC2};
    ASSERT_DEBUG_DEATH(giw.Warp(experimental_twochannel_image, &experimental_twochannel_image), ".*");
    cv::Mat experimental_fourchannel_image{control_input_image22.rows, control_input_image22.cols, CV_8UC4};
    ASSERT_DEBUG_DEATH(giw.Warp(experimental_fourchannel_image, &experimental_fourchannel_image), ".*");
}

// separate test case (no fixture) for a 3x3 image and a lut that points to
// the middle of the image
TEST(GpuImageWarperTest33, WarpThreeByThree) {
    // this is a 3x3 red/blue checkerboard:
    //              R|B|R
    //              B|R|B
    //              R|B|R
    // clang-format adds a lot of spaces to this declaration, thus we disable it
    // clang-format off
    unsigned char input_image33_data[27]{0, 0, 252, 252, 0, 0, 0, 0, 252, 252, 0, 0, 0, 0,
                                         252, 252, 0, 0, 0, 0, 252, 252, 0, 0, 0, 0, 252};
    // clang-format on
    cv::Mat const control_lut_shrink = cv::Mat(1, 1, CV_32FC2, cv::Scalar(1.5f, 1.5f));
    cv::Mat const control_input_image33{3, 3, CV_8UC3, input_image33_data};

    // in this case, the resulting color should be that of the central pixel (0, 0, 252)
    nie::gpu::GpuImageWarper giw(control_lut_shrink);
    cv::Mat experimental_image33{control_input_image33.rows, control_input_image33.cols, control_input_image33.type()};
    ASSERT_NO_THROW(giw.Warp(control_input_image33, &experimental_image33));
    ASSERT_EQ(experimental_image33.at<cv::Vec3b>(0, 0), cv::Vec3b(0, 0, 252));
}
#endif
