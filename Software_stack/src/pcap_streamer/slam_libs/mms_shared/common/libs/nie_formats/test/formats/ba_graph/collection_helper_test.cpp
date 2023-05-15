/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <cstdio>

#include <gtest/gtest.h>
#include <nie/formats/ba_graph.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>

class CollectionHelperTest : public testing::Test {
protected:
    CollectionHelperTest() : pose_(CreatePoseCollection()) {}

    static nie::io::PoseCollection CreatePoseCollection() {
        // clang-format off
        std::vector<nie::io::PoseRecord> const poses{
            {2, nie::io::PoseRecord::Category::kGps, 0, nie::Timestamp_ns(std::chrono::seconds(0)),
             nie::Isometry3qd::Identity(), Eigen::Matrix<double,6,6>::Zero()},
            {3, nie::io::PoseRecord::Category::kGps, 0, nie::Timestamp_ns{std::chrono::seconds(0)},
             nie::Isometry3qd::Identity(), Eigen::Matrix<double,6,6>::Zero()},
            {kMaxPoseId, nie::io::PoseRecord::Category::kGps, 0, nie::Timestamp_ns{std::chrono::seconds(0)},
             nie::Isometry3qd::Identity(), Eigen::Matrix<double,6,6>::Zero()}};
        // clang-format on
        nie::io::PoseHeader const header{nie::io::kPoseCollectionVersionLatest, 0, "", 4326, 0, {}, {}};
        return nie::io::PoseCollection{header, poses, {}, {}};
    }

    static nie::io::PoseId constexpr kMaxPoseId = 5;

    nie::io::PoseCollection const pose_;
    nie::io::InfoRefCollection const info_ref_collection_{{}, {{0, 0, "a"}, {1, 0, "b"}, {2, 0, "c"}}};
};

TEST_F(CollectionHelperTest, GetMaxPoseId) { ASSERT_EQ(nie::io::GetMaxPoseId(pose_), kMaxPoseId); }

TEST_F(CollectionHelperTest, CreateRecordMapConst) {
    std::unordered_map<std::int32_t, std::reference_wrapper<nie::io::InfoRefRecord const>> record_map =
            nie::io::CreateRecordMap(info_ref_collection_.info_refs.cbegin(), info_ref_collection_.info_refs.cend());

    for (auto const& info_ref : info_ref_collection_.info_refs) {
        nie::io::InfoRefRecord const& wrapped_ref = record_map.at(info_ref.id);
        ASSERT_EQ(wrapped_ref.id, info_ref.id);
        ASSERT_EQ(wrapped_ref.frame_id, info_ref.frame_id);
        ASSERT_EQ(wrapped_ref.path, info_ref.path);
    }
}

TEST_F(CollectionHelperTest, CreateRecordMapNonConst) {
    nie::io::InfoRefCollection info_ref_collection = info_ref_collection_;

    std::unordered_map<std::int32_t, std::reference_wrapper<nie::io::InfoRefRecord>> record_map =
            nie::io::CreateRecordMap(info_ref_collection.info_refs.begin(), info_ref_collection.info_refs.end());

    for (auto const& info_ref : info_ref_collection_.info_refs) {
        nie::io::InfoRefRecord& wrapped_ref = record_map.at(info_ref.id);
        ASSERT_EQ(wrapped_ref.id, info_ref.id);
        ASSERT_EQ(wrapped_ref.frame_id, info_ref.frame_id);
        ASSERT_EQ(wrapped_ref.path, info_ref.path);
    }
}