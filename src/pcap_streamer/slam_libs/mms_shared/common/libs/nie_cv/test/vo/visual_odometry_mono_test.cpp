/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <memory>

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>

#include <nie/core/filesystem.hpp>
#include <nie/core/geometry/isometry3.hpp>
#include <nie/cv/vo/feature/descriptor_extractor_lv_edge.hpp>
#include <nie/cv/vo/feature/detector_lv_blob.hpp>
#include <nie/cv/vo/feature/detector_lv_corner.hpp>
#include <nie/cv/vo/feature/match_filter_bucketing.hpp>
#include <nie/cv/vo/feature/match_filter_local_flow.hpp>
#include <nie/cv/vo/feature/matcher_lv.hpp>
#include <nie/cv/vo/visual_odometry_mono.hpp>

class VisualOdometryMonoTest : public ::testing::Test {
protected:
    VisualOdometryMonoTest()
        : path_("/data/aiim/unit_tests_data/cv/vo/"),
          files_(nie::FindFiles(path_, ".*.jpg")),
          vo_(),
          pose_(nie::Isometry3md::Identity()),
          default_pose_(nie::Isometry3md::Identity()),
          image_width_(0),
          image_height_(0) {}

    void SetUp() override {
        // Check the images to process
        ASSERT_NE(files_.size(), 0) << "No files found in '" << path_ << "', so could not test.";
        EXPECT_NE(files_.size(), 1) << "Some tests will be useless as there is only one image in '" << path_ << "'.";

        // Determine the size of the image
        {
            cv::Mat image = cv::imread(files_.front().string(), cv::IMREAD_GRAYSCALE);
            image_width_ = image.cols;
            image_height_ = image.rows;
        }

        BuildVoMono();
    }

    void ResetVo() {
        pose_ = default_pose_;
        vo_.reset();
        BuildVoMono();
    }

    void BuildVoMono() {
        std::vector<nie::DetectorPtr> detectors;
        {
            nie::Detector::Parameters params;
            params.feature_distance = 10;

            detectors.push_back(std::make_unique<nie::DetectorLvBlob>(params));
            detectors.push_back(std::make_unique<nie::DetectorLvCorner>(params));
        }

        std::vector<nie::MatchFilterPtr> filters;
        filters.push_back(std::make_unique<nie::MatchFilterLocalFlow>(image_width_, image_height_));
        {
            nie::MatchFilterBucketing::Parameters params;
            params.strategy = nie::MatchFilterBucketing::ReductionStrategy::kFirst;

            filters.push_back(std::make_unique<nie::MatchFilterBucketing>(image_width_, image_height_, params));
        }

        double f = 1285.;
        double ppx = 953.35;
        double ppy = 570.26;
        Eigen::Matrix3d K = (Eigen::Matrix3d() << f, 0., f, 0., ppx, ppy, 0., 0., 1.).finished();

        nie::VisualOdometryMono::Parameters params(K);
        params.keyframe_window_size = 2;
        params.ba_window_size = 2;
        params.ba_window_overlap = 1;
        params.global_flow_threshold = 5.;

        nie::VisualOdometryConfiguration conf{
                std::move(detectors),
                std::make_unique<nie::DescriptorExtractorLvEdge>(),
                std::make_unique<nie::MatcherLv>(image_width_, image_height_),
                std::move(filters)};
        vo_ = std::make_unique<nie::VisualOdometryMono>(
                image_width_, image_height_, std::move(conf), std::move(params));

        vo_->AddCallback<nie::VisualOdometryMono::Handle::kWritePose>(
                [this](std::int32_t const&, nie::Isometry3md const& pose) { pose_ = pose; });

        ASSERT_TRUE(vo_) << "Initialization of VisualOdometryMono failed.";
    }

    // The matrix is passed by pointer, because ASSERT_* cannot be used with
    // function that do not return void
    void ReadImage(boost::filesystem::path const& file, cv::Mat* p_image) {
        std::string const& path = file.string();
        *p_image = cv::imread(path, cv::IMREAD_GRAYSCALE);
        ASSERT_NE(p_image->data, nullptr) << "Could not read image '" << path << "'";
    }

    std::string const path_;
    std::vector<boost::filesystem::path> files_;
    std::unique_ptr<nie::VisualOdometryMono> vo_;

    nie::Isometry3md pose_;
    nie::Isometry3md const default_pose_;

private:
    int image_width_, image_height_;
};

// Check that the first image does not result in any motion as it is a kind of
// initialization of the visual odometry
TEST_F(VisualOdometryMonoTest, FirstImage) {
    cv::Mat image;
    ReadImage(files_.front(), &image);

    ASSERT_FALSE(vo_->Process(0, image));

    EXPECT_EQ(pose_.rotation(), default_pose_.rotation());
    EXPECT_EQ(pose_.translation(), default_pose_.translation());
}

// Check that camera does not move when the same image is processed twice
TEST_F(VisualOdometryMonoTest, NoMotion) {
    cv::Mat image;
    ReadImage(files_.front(), &image);

    ASSERT_FALSE(vo_->Process(0, image));

    // Reset pose
    pose_ = default_pose_;
    ASSERT_FALSE(vo_->Process(1, image));

    EXPECT_EQ(pose_.rotation(), default_pose_.rotation());
    EXPECT_EQ(pose_.translation(), default_pose_.translation());
}

// Check that camera did move when the two different images are processed
TEST_F(VisualOdometryMonoTest, Motion) {
    // Sanity check
    if (files_.size() == 1) {
        EXPECT_FALSE(true) << "Useless test with only one image.";
        return;
    }

    // Select a sensible image for testing (not too large difference)
    std::size_t index_last = std::min(files_.size() - 1, 6ul);
    std::size_t index_mid = index_last / 2;
    ASSERT_FALSE(index_mid == 0);
    ASSERT_TRUE(index_mid < index_last);

    cv::Mat image;
    ReadImage(files_[0], &image);
    ASSERT_FALSE(vo_->Process(0, image));
    ReadImage(files_[index_mid], &image);
    ASSERT_TRUE(vo_->Process(1, image));
    ReadImage(files_[index_last], &image);
    ASSERT_TRUE(vo_->Process(1, image));

    // Not really a thorough check to perform yet
    EXPECT_NE(pose_, default_pose_);
}
