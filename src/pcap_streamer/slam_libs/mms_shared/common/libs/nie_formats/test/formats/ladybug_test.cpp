/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifdef USE_LADYBUG
#include <gtest/gtest.h>
#include <algorithm>
#include <nie/core/filesystem.hpp>
#include <nie/formats/ladybug.hpp>
#include <opencv2/opencv.hpp>
#include <string>

class LadybugTest : public ::testing::Test {
protected:
    boost::filesystem::path const kUnitTestDataDir = "/data/aiim/unit_tests_data/formats/ladybug";

    std::string const kImageStreamFile = "ladybug_16146054_20190228_181105-000000.pgr";
    std::string const kCalibrationFile = "ladybug_16146054.cal";
    std::string const kStitchedPanoFile = "ladybug-16146054_20190228_181105.png";

    boost::filesystem::path const kImageStreamPath = kUnitTestDataDir / kImageStreamFile;
    boost::filesystem::path const kCalibrationPath = kUnitTestDataDir / kCalibrationFile;
    boost::filesystem::path const kStitchedPanoPath = kUnitTestDataDir / kStitchedPanoFile;
    boost::filesystem::path const kForbiddenPath = "/root/forbidden";
};

class LadybugStreamTest : public LadybugTest {
protected:
    LadybugStreamTest() : stream_reader_(kImageStreamPath) {}

    nie::io::ladybug::StreamReader stream_reader_;
};

class LadybugImageProcessorTest : public LadybugStreamTest {
protected:
    LadybugImageProcessorTest() : image_processor_(kCalibrationPath) {}

    nie::io::ladybug::ImageProcessor image_processor_;
};

TEST_F(LadybugTest, StreamReaderLifecycle) {
    using namespace nie::io::ladybug;

    StreamReader* stream_reader = nullptr;
    ASSERT_NO_THROW(stream_reader = new StreamReader(kImageStreamPath));
    ASSERT_NO_THROW(delete stream_reader);
    ASSERT_NO_FATAL_FAILURE(stream_reader = new StreamReader(kImageStreamPath));
    ASSERT_NO_FATAL_FAILURE(delete stream_reader);

    // Should throw upon attempting to read a forbidden or otherwise unavailable path
    ASSERT_THROW(new StreamReader(kForbiddenPath), Exception);
}

TEST_F(LadybugTest, ImageProcessorLifecycle) {
    using namespace nie::io::ladybug;

    ImageProcessor* image_processor = nullptr;
    ASSERT_NO_THROW(image_processor = new ImageProcessor(kCalibrationPath));
    ASSERT_NO_THROW(delete image_processor);
    ASSERT_NO_FATAL_FAILURE(image_processor = new ImageProcessor(kCalibrationPath));
    ASSERT_NO_FATAL_FAILURE(delete image_processor);

    // Should throw upon attempting to read a forbidden or otherwise unavailable path
    ASSERT_THROW(new ImageProcessor(kForbiddenPath), Exception);
}

TEST_F(LadybugStreamTest, WriteCalibrationFile) {
    boost::filesystem::path const kCalibrationTestPath = nie::TemporaryFile(kCalibrationFile);

    stream_reader_.SaveCalibrationFile(kCalibrationTestPath);

    ASSERT_TRUE(nie::BinaryEquals(kCalibrationPath, kCalibrationTestPath));

    std::remove(kCalibrationTestPath.c_str());
}

TEST_F(LadybugStreamTest, BasicReadImage) {
    ASSERT_FLOAT_EQ(stream_reader_.frame_rate(), 8.0);
    ASSERT_EQ(stream_reader_.frame_count(), 793);

    // Stream should start at first image
    ASSERT_EQ(stream_reader_.frame_id(), 0);
    // Read an image
    EXPECT_NO_THROW(stream_reader_.Read());
    // Stream should have advanced with 1
    ASSERT_EQ(stream_reader_.frame_id(), 1);
}

TEST_F(LadybugStreamTest, SeekAndRead) {
    unsigned const frame_count = stream_reader_.frame_count();

    // Seek to first frame and read image
    EXPECT_NO_THROW(stream_reader_.Seek(0));
    ASSERT_EQ(stream_reader_.frame_id(), 0);
    EXPECT_NO_THROW(stream_reader_.Read());

    // Seek forward to intermediate frame and read image
    EXPECT_NO_THROW(stream_reader_.Seek(frame_count / 2));
    ASSERT_EQ(stream_reader_.frame_id(), frame_count / 2);
    EXPECT_NO_THROW(stream_reader_.Read());

    // Seek to last frame and read image
    EXPECT_NO_THROW(stream_reader_.Seek(frame_count - 1));
    ASSERT_EQ(stream_reader_.frame_id(), frame_count - 1);
    EXPECT_NO_THROW(stream_reader_.Read());
    ASSERT_EQ(stream_reader_.frame_id(), frame_count - 1);

    // Seek backward to first frame and read image (again)
    EXPECT_NO_THROW(stream_reader_.Seek(0));
    ASSERT_EQ(stream_reader_.frame_id(), 0);
    EXPECT_NO_THROW(stream_reader_.Read());
    ASSERT_EQ(stream_reader_.frame_id(), 1);

    // Try to seek beyond last frame (should fail, and frame_id should not have changed)
    EXPECT_THROW(stream_reader_.Seek(frame_count), nie::io::ladybug::Exception);
    ASSERT_EQ(stream_reader_.frame_id(), 1);
}

TEST_F(LadybugImageProcessorTest, StichImage) {
    boost::filesystem::path const kStitchedPanoTestPath = nie::TemporaryFile(kStitchedPanoFile);

    // Stitch an image with default settings (Panorama)
    cv::Mat image;
    ASSERT_NO_THROW(image = image_processor_.Process(stream_reader_.Read()));
    cv::imwrite(kStitchedPanoTestPath.c_str(), image);

    ASSERT_TRUE(nie::BinaryEquals(kStitchedPanoPath, kStitchedPanoTestPath));

    std::remove(kStitchedPanoTestPath.c_str());
}
#endif
