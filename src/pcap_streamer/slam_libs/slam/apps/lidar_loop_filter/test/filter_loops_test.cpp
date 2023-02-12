/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <filter_loops.hpp>

auto constexpr kInFileLoops = "/data/aiim/unit_tests_data/loops_filter/in_loops.pose";
auto constexpr kInFileBboxPose = "/data/aiim/unit_tests_data/loops_filter/in_bbox.pose";
auto constexpr kInFileBboxBbox = "/data/aiim/unit_tests_data/loops_filter/in_bbox.bbox";
auto constexpr kInFileBboxIref = "/data/aiim/unit_tests_data/loops_filter/in_bbox.iref";
auto constexpr kInFileLoamPose = "/data/aiim/unit_tests_data/loops_filter/in_loam.pose";
auto constexpr kInFileLoamIref = "/data/aiim/unit_tests_data/loops_filter/in_loam.iref";

class FilterLoopsTest : public ::testing::Test {
public:
    FilterLoopsTest() {
        in_loops_ = nie::io::ReadCollection<nie::io::PoseCollection>(kInFileLoops);
        in_bbox_pose_ = nie::io::ReadCollection<nie::io::PoseCollection>(kInFileBboxPose);
        in_bbox_bbox_ = nie::io::ReadCollection<nie::io::BboxCollection>(kInFileBboxBbox);
        in_bbox_iref_ = nie::io::ReadCollection<nie::io::InfoRefCollection>(kInFileBboxIref);
        in_loam_pose_ = nie::io::ReadCollection<nie::io::PoseCollection>(kInFileLoamPose);
        in_loam_iref_ = nie::io::ReadCollection<nie::io::InfoRefCollection>(kInFileLoamIref);
    }

protected:
    nie::io::PoseCollection in_loops_;
    nie::io::PoseCollection in_bbox_pose_;
    nie::io::BboxCollection in_bbox_bbox_;
    nie::io::InfoRefCollection in_bbox_iref_;
    nie::io::PoseCollection in_loam_pose_;
    nie::io::InfoRefCollection in_loam_iref_;
};

TEST_F(FilterLoopsTest, AreaOk) {
    double intersection_area_threshold = 15000;
    double intersection_ratio_threshold = 0.01;
    double minimum_trace_length = 0.01;
    double maximum_trace_cov_score = 0.1;

    auto out_loops = nie::FilterLoops(
            in_loops_,
            in_bbox_pose_,
            in_bbox_bbox_,
            in_bbox_iref_,
            in_loam_pose_,
            in_loam_iref_,
            intersection_area_threshold,
            intersection_ratio_threshold,
            minimum_trace_length,
            maximum_trace_cov_score);

    EXPECT_EQ(out_loops.edges.size(), 3);
}

TEST_F(FilterLoopsTest, AreaFails) {
    double intersection_area_threshold = 50000.0;
    double intersection_ratio_threshold = 0.01;
    double minimum_trace_length = 0.01;
    double maximum_trace_cov_score = 0.1;

    auto out_loops = nie::FilterLoops(
            in_loops_,
            in_bbox_pose_,
            in_bbox_bbox_,
            in_bbox_iref_,
            in_loam_pose_,
            in_loam_iref_,
            intersection_area_threshold,
            intersection_ratio_threshold,
            minimum_trace_length,
            maximum_trace_cov_score);

    EXPECT_EQ(out_loops.edges.size(), 0);
}

TEST_F(FilterLoopsTest, RatioOk) {
    double intersection_area_threshold = 0.01;
    double intersection_ratio_threshold = 0.1;
    double minimum_trace_length = 0.01;
    double maximum_trace_cov_score = 0.1;

    auto out_loops = nie::FilterLoops(
            in_loops_,
            in_bbox_pose_,
            in_bbox_bbox_,
            in_bbox_iref_,
            in_loam_pose_,
            in_loam_iref_,
            intersection_area_threshold,
            intersection_ratio_threshold,
            minimum_trace_length,
            maximum_trace_cov_score);

    EXPECT_EQ(out_loops.edges.size(), 3);
}

TEST_F(FilterLoopsTest, RatioFails) {
    double intersection_area_threshold = 0.01;
    double intersection_ratio_threshold = 0.8;
    double minimum_trace_length = 0.01;
    double maximum_trace_cov_score = 0.1;

    auto out_loops = nie::FilterLoops(
            in_loops_,
            in_bbox_pose_,
            in_bbox_bbox_,
            in_bbox_iref_,
            in_loam_pose_,
            in_loam_iref_,
            intersection_area_threshold,
            intersection_ratio_threshold,
            minimum_trace_length,
            maximum_trace_cov_score);

    EXPECT_EQ(out_loops.edges.size(), 0);
}

TEST_F(FilterLoopsTest, TraceOk) {
    double intersection_area_threshold = 0.01;
    double intersection_ratio_threshold = 0.1;
    double minimum_trace_length = 5.0;
    double maximum_trace_cov_score = 0.1;

    auto out_loops = nie::FilterLoops(
            in_loops_,
            in_bbox_pose_,
            in_bbox_bbox_,
            in_bbox_iref_,
            in_loam_pose_,
            in_loam_iref_,
            intersection_area_threshold,
            intersection_ratio_threshold,
            minimum_trace_length,
            maximum_trace_cov_score);

    EXPECT_EQ(out_loops.edges.size(), 3);
}

TEST_F(FilterLoopsTest, TraceFails) {
    double intersection_area_threshold = 0.01;
    double intersection_ratio_threshold = 0.01;
    double minimum_trace_length = 400.0;
    double maximum_trace_cov_score = 0.1;

    auto out_loops = nie::FilterLoops(
            in_loops_,
            in_bbox_pose_,
            in_bbox_bbox_,
            in_bbox_iref_,
            in_loam_pose_,
            in_loam_iref_,
            intersection_area_threshold,
            intersection_ratio_threshold,
            minimum_trace_length,
            maximum_trace_cov_score);

    EXPECT_EQ(out_loops.edges.size(), 0);
}

TEST_F(FilterLoopsTest, ScoreBelowMaxIsFilteredOutEverything) {
    double intersection_area_threshold = 0.01;
    double intersection_ratio_threshold = 0.1;
    double minimum_trace_length = 5.0;
    double maximum_trace_cov_score = 1000.0;

    auto out_loops = nie::FilterLoops(
            in_loops_,
            in_bbox_pose_,
            in_bbox_bbox_,
            in_bbox_iref_,
            in_loam_pose_,
            in_loam_iref_,
            intersection_area_threshold,
            intersection_ratio_threshold,
            minimum_trace_length,
            maximum_trace_cov_score);

    EXPECT_EQ(out_loops.edges.size(), 0);
}

TEST_F(FilterLoopsTest, ScoreBelowMaxIsFilteredOutSome) {
    double intersection_area_threshold = 0.01;
    double intersection_ratio_threshold = 0.1;
    double minimum_trace_length = 5.0;
    double maximum_trace_cov_score = 0.15;

    auto out_loops = nie::FilterLoops(
            in_loops_,
            in_bbox_pose_,
            in_bbox_bbox_,
            in_bbox_iref_,
            in_loam_pose_,
            in_loam_iref_,
            intersection_area_threshold,
            intersection_ratio_threshold,
            minimum_trace_length,
            maximum_trace_cov_score);

    EXPECT_EQ(out_loops.edges.size(), 1);
}