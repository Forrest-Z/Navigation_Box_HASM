/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <find_loops.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>

class FindLoopsF : public ::testing::Test {
protected:
    //clang-format off
    // Poses forming a "square" shaped loop
    std::vector<nie::io::PoseRecord> const kPoses{
            {0,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(0)),
             {{0, 0, 0}, Eigen::Quaterniond::UnitRandom()},
             {}},
            {1,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(1)),
             {{1, 0, 100}, Eigen::Quaterniond::UnitRandom()},
             {}},  // intersection
            {2,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(2)),
             {{2, 10, 0}, Eigen::Quaterniond::UnitRandom()},
             {}},
            {3,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(3)),
             {{3, 0, 0}, Eigen::Quaterniond::UnitRandom()},
             {}},
            {4,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(4)),
             {{4, 40, 0}, Eigen::Quaterniond::UnitRandom()},
             {}},
            {5,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(5)),
             {{4, 11, 0}, Eigen::Quaterniond::UnitRandom()},
             {}},
            {6,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(6)),
             {{4, 22, 0}, Eigen::Quaterniond::UnitRandom()},
             {}},
            {7,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(7)),
             {{24, 3, 0}, Eigen::Quaterniond::UnitRandom()},
             {}},
            {8,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(8)),
             {{4, 24, 0}, Eigen::Quaterniond::UnitRandom()},
             {}},
            {9,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(9)),
             {{13, 4, 0}, Eigen::Quaterniond::UnitRandom()},
             {}},
            {10,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(10)),
             {{2, 14, 0}, Eigen::Quaterniond::UnitRandom()},
             {}},
            {11,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(11)),
             {{2, 4, 0}, Eigen::Quaterniond::UnitRandom()},
             {}},
            {12,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(12)),
             {{8, 3, 0}, Eigen::Quaterniond::UnitRandom()},
             {}},
            {13,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(13)),
             {{4, 2, 0}, Eigen::Quaterniond::UnitRandom()},
             {}},
            {14,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(14)),
             {{31, 31, 0}, Eigen::Quaterniond::UnitRandom()},
             {}},
            {15,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(15)),
             {{1, 0, 0}, Eigen::Quaterniond::UnitRandom()},
             {}},  // intersection
            {16,
             nie::io::PoseRecord::Category::kGps,
             0,
             nie::Timestamp_ns(std::chrono::nanoseconds(16)),
             {{1, -1, 0}, Eigen::Quaterniond::UnitRandom()},
             {}},
    };

    std::unordered_map<std::int32_t, std::reference_wrapper<const nie::io::PoseRecord>> const kRecordMap =
            nie::io::CreateRecordMap(kPoses.cbegin(), kPoses.cend());
    //clang-format on
};

TEST_F(FindLoopsF, EmptyPoseVector) {
    auto edges = nie::FindLoops({}, 0.9999, std::chrono::nanoseconds(1));

    EXPECT_EQ(edges.size(), 0);
}

TEST_F(FindLoopsF, FoundSimple) {
    // time threshold very low
    auto edges = nie::FindLoops(kPoses, 0.9999, std::chrono::nanoseconds(1));
    EXPECT_EQ(edges.size(), 1);
    EXPECT_EQ(edges[0].id_begin, 1);
    EXPECT_EQ(edges[0].id_end, 15);
    EXPECT_EQ(edges[0].category, nie::io::PoseEdgeRecord::Category::kLoop);

    EXPECT_TRUE(kRecordMap.at(edges[0].id_begin)
                        .get()
                        .isometry.TransformInverseLeft(kRecordMap.at(edges[0].id_end).get().isometry)
                        .isApprox(edges[0].isometry));

    // time threshold just low enough
    edges = nie::FindLoops(kPoses, 0.9999, std::chrono::nanoseconds(14));
    EXPECT_EQ(edges.size(), 1);
    EXPECT_EQ(edges[0].id_begin, 1);
    EXPECT_EQ(edges[0].id_end, 15);
    EXPECT_EQ(edges[0].category, nie::io::PoseEdgeRecord::Category::kLoop);

    EXPECT_TRUE(kRecordMap.at(edges[0].id_begin)
                        .get()
                        .isometry.TransformInverseLeft(kRecordMap.at(edges[0].id_end).get().isometry)
                        .isApprox(edges[0].isometry));
}

TEST_F(FindLoopsF, FoundSix) {
    // time threshold very low
    auto edges = nie::FindLoops(kPoses, std::sqrt(2), std::chrono::nanoseconds(1));
    EXPECT_EQ(edges.size(), 6);
}

TEST_F(FindLoopsF, NotFoundSimple) {
    // no loop for given distance threshold
    auto edges = nie::FindLoops({kPoses.cbegin() + 2, kPoses.cend()}, 0.9999, std::chrono::nanoseconds(1));
    EXPECT_EQ(edges.size(), 0);

    // no loop for given time threshold
    edges = nie::FindLoops(kPoses, 0.9999, std::chrono::nanoseconds(15));
    EXPECT_EQ(edges.size(), 0);
}
