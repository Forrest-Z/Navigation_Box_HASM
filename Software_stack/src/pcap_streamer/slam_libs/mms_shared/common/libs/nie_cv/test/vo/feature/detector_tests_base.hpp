/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef VO_FEATURE_DETECTOR_TESTS_BASE_HPP
#define VO_FEATURE_DETECTOR_TESTS_BASE_HPP

#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>
#include <nie/core/filesystem.hpp>
#include <nie/cv/vo/feature/detector.hpp>
#include <nie/cv/vo/feature/keypoint.hpp>
#include <opencv2/opencv.hpp>

/*
 * Test framework that tests the feature detection classes.
 */
class VoFeatureDetectorTest : public ::testing::Test {
protected:
    void SetUp() override {
        std::string path = "/data/aiim/unit_tests_data/cv/vo/";
        std::vector<boost::filesystem::path> const& files = nie::FindFiles(path, ".*.jpg");
        ASSERT_NE(files.size(), 0) << "No files found in '" << path << "', so could not test.";

        for (boost::filesystem::path const& file : files) {
            cv::Mat image = cv::imread(file.string(), cv::IMREAD_GRAYSCALE);
            ASSERT_FALSE(image.data == nullptr) << "Could not read image '" << file.string() << "'";
            images_.emplace_back(file.stem().string(), image);
        }
    }

    void ProcessImages(nie::DetectorPtr const& detector, std::map<std::string, int> const& expectations) const {
        bool anyImageTested = false;
        for (auto const& image : images_) {
            auto const& expectation = expectations.find(image.first);
            if (expectation != expectations.end()) {
                nie::KeypointVector features;
                nie::KeypointTypeVector feature_types;
                detector->Detect(image.second, &features, &feature_types);
                ASSERT_EQ(features.size(), expectation->second) << "Different number of features found.";
                anyImageTested = true;
            }
        }
        if (not anyImageTested) {
            EXPECT_FALSE(true) << "Useless test with no expectations given for the found images.";
        }
    }

private:
    std::vector<std::pair<std::string, cv::Mat>> images_;
};

#endif  // VO_FEATURE_DETECTOR_TESTS_BASE_HPP
