/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/hardware/camera/camera.hpp>
#include <nie/hardware/camera/file_camera.hpp>
#include <opencv2/core.hpp>

class CameraTest : public testing::Test {
protected:
    std::uint32_t const kTimeoutMs = 2000;
    std::uint32_t const kShortTimeoutMs = 1;
    std::uint16_t const kImagesInFolder = 11;  // no. of files in kImagesPath
    // this folder has a 7.7MB png which most definitely will take more than 1ms to read
    std::string const kBigImagePath = "/data/aiim/unit_tests_data/hardware/camera/stereo_calibration_pico/big_image/";
    // this has a subset of the images which we actually use for calibration
    std::string const kImagesPath = "/data/aiim/unit_tests_data/hardware/camera/stereo_calibration_pico/l/";
    cv::Size const kImageSize{1920, 1200};
    std::unique_ptr<nie::Camera> file_camera = std::make_unique<nie::FileCamera>(kImagesPath, kImageSize);
};

TEST_F(CameraTest, AccessorsTest) {
    ASSERT_EQ(file_camera->id(), "/data/aiim/unit_tests_data/hardware/camera/stereo_calibration_pico/l/");
    ASSERT_EQ(file_camera->image_size(), cv::Size(1920, 1200));
}

TEST_F(CameraTest, GetFrameTimeout) {
    cv::Mat cv_frame;
    std::int64_t timestamp;

    file_camera = std::make_unique<nie::FileCamera>(kBigImagePath, kImageSize);

    file_camera->StartGrabbing(nie::GrabStrategy::kOneByOne);
    ASSERT_THROW(file_camera->GetFrame(kShortTimeoutMs, &cv_frame, &timestamp, true), std::runtime_error);
    file_camera->StopGrabbing();

    file_camera->StartGrabbing(nie::GrabStrategy::kOneByOne);
    ASSERT_NO_THROW(file_camera->GetFrame(kShortTimeoutMs, &cv_frame, &timestamp, false));
    file_camera->StopGrabbing();
}

TEST_F(CameraTest, StartStopGrabbing) {
    // check if various combinations of start/stopping work as expected
    // first, stop without starting:
    ASSERT_NO_THROW(file_camera->StopGrabbing());
    // start then stop
    ASSERT_NO_THROW(file_camera->StartGrabbing(nie::GrabStrategy::kOneByOne));
    ASSERT_NO_THROW(file_camera->StopGrabbing());
    // start twice -- switches the strategy on-the-fly to maintain consistency
    // with the real camera, we can not check if strategy actually changed
    // without exposing some dangerous internals or creating an otherwise
    // pointless accessor so let's just trust that this does not crash
    ASSERT_NO_THROW(file_camera->StartGrabbing(nie::GrabStrategy::kOneByOne));
    ASSERT_NO_THROW(file_camera->StartGrabbing(nie::GrabStrategy::kLatestImageOnly));
    // stop twice -- again, to maintain consistency w.r.t. the real camera, we
    // do nothing and also throw no exceptions
    ASSERT_NO_THROW(file_camera->StopGrabbing());
    ASSERT_NO_THROW(file_camera->StopGrabbing());
}

TEST_F(CameraTest, IsGrabbing) {
    ASSERT_FALSE(file_camera->IsGrabbing());
    file_camera->StartGrabbing(nie::GrabStrategy::kOneByOne);
    ASSERT_TRUE(file_camera->IsGrabbing());
    file_camera->StopGrabbing();
    ASSERT_FALSE(file_camera->IsGrabbing());
    file_camera->StartGrabbing(nie::GrabStrategy::kLatestImageOnly);
    ASSERT_TRUE(file_camera->IsGrabbing());
    file_camera->StopGrabbing();
}
