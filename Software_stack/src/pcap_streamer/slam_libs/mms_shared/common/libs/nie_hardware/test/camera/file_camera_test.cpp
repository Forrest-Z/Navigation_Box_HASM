/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/hardware/camera/camera.hpp>
#include <nie/hardware/camera/file_camera.hpp>
#include <opencv2/core.hpp>

class FileCameraTest : public testing::Test {
protected:
    std::uint32_t const kTimeoutMs = 2000;
    std::uint32_t const kShortTimeoutMs = 1;
    std::uint16_t const kImagesInFolder = 16;  // no. of files in kImagesPath
    std::string const kLeftCameraPath = "/data/aiim/unit_tests_data/hardware/camera/coremap/LeftCamera/";
    std::string const kBigImagePath = "/data/aiim/unit_tests_data/hardware/camera/coremap/big_image/";
    cv::Size const kImageSize{4096, 2168};
    cv::Size const kBigImageSize{3840, 2400};
    std::unique_ptr<nie::Camera> file_camera_ = std::make_unique<nie::FileCamera>(kLeftCameraPath, kImageSize);
};

TEST_F(FileCameraTest, AccessorsTest) {
    EXPECT_EQ(file_camera_->id(), kLeftCameraPath);
    EXPECT_EQ(file_camera_->image_size(), kImageSize);
}

TEST_F(FileCameraTest, GetFrameTimeout) {
    cv::Mat cv_frame;
    std::int64_t timestamp;

    file_camera_ = std::make_unique<nie::FileCamera>(kBigImagePath, kBigImageSize);

    file_camera_->StartGrabbing(nie::GrabStrategy::kOneByOne);
    ASSERT_THROW(file_camera_->GetFrame(kShortTimeoutMs, &cv_frame, &timestamp, true), std::runtime_error);
    file_camera_->StopGrabbing();

    file_camera_->StartGrabbing(nie::GrabStrategy::kOneByOne);
    ASSERT_NO_THROW(file_camera_->GetFrame(kShortTimeoutMs, &cv_frame, &timestamp, false));
    file_camera_->StopGrabbing();
}

TEST_F(FileCameraTest, StartStopGrabbing) {
    // check if various combinations of start/stopping work as expected
    // first, stop without starting:
    ASSERT_NO_THROW(file_camera_->StopGrabbing());
    // start then stop
    ASSERT_NO_THROW(file_camera_->StartGrabbing(nie::GrabStrategy::kOneByOne));
    ASSERT_NO_THROW(file_camera_->StopGrabbing());
    // start twice -- switches the strategy on-the-fly to maintain consistency
    // with the real camera, we can not check if strategy actually changed
    // without exposing some dangerous internals or creating an otherwise
    // pointless accessor so let's just trust that this does not crash
    ASSERT_NO_THROW(file_camera_->StartGrabbing(nie::GrabStrategy::kOneByOne));
    ASSERT_NO_THROW(file_camera_->StartGrabbing(nie::GrabStrategy::kLatestImageOnly));
    // stop twice -- again, to maintain consistency w.r.t. the real camera, we
    // do nothing and also throw no exceptions
    ASSERT_NO_THROW(file_camera_->StopGrabbing());
    ASSERT_NO_THROW(file_camera_->StopGrabbing());
}

TEST_F(FileCameraTest, IsGrabbing) {
    ASSERT_FALSE(file_camera_->IsGrabbing());
    file_camera_->StartGrabbing(nie::GrabStrategy::kOneByOne);
    ASSERT_TRUE(file_camera_->IsGrabbing());
    file_camera_->StopGrabbing();
    ASSERT_FALSE(file_camera_->IsGrabbing());
    file_camera_->StartGrabbing(nie::GrabStrategy::kLatestImageOnly);
    ASSERT_TRUE(file_camera_->IsGrabbing());
    file_camera_->StopGrabbing();
}
