/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <cstdio>

#include <gtest/gtest.h>
#include <nie/core/filesystem.hpp>
#include <nie/formats/ba_graph.hpp>

class PoseCollectionTest : public testing::Test {
private:
    // Helper functions
    static Eigen::Matrix<double, 6, 6> CreateSymmetricMatrix() {
        Eigen::Matrix<double, 6, 6> m;
        for (std::size_t row = 0; row < 6; ++row) {
            for (std::size_t col = row; col < 6; ++col) {
                std::size_t value = row * 6 + col;
                m(row, col) = value;
                m(col, row) = value;
            }
        }
        return m;
    }
    static nie::io::PoseCollection CreateCollection() {
        // clang-format off
        std::vector<nie::io::PoseRecord> const poses{
            {1, nie::io::PoseRecord::Category::kGps, 1, nie::Timestamp_ns(std::chrono::nanoseconds(1)),
                {{1., 1., 1.}, {1., 1., 1., 1.}}, Eigen::Matrix<double, 6, 6>::Identity()},
            {2, nie::io::PoseRecord::Category::kGps, 2, nie::Timestamp_ns(std::chrono::nanoseconds(2)),
                {{2., 2., 2.}, {2., 2., 2., 2.}}, Eigen::Matrix<double, 6, 6>::Identity()},
            {3, nie::io::PoseRecord::Category::kBbox, 3, nie::Timestamp_ns(std::chrono::nanoseconds(3)),
                {{3., 3., 3.}, {3., 3., 3., 3.}}, CreateSymmetricMatrix()}};

        std::vector<nie::io::PoseEdgeRecord> const edges{
            {1, 2, nie::io::PoseEdgeRecord::Category::kLoop,
                {{4., 4., 4.}, {4., 4., 4., 4.}}, Eigen::Matrix<double, 6, 6>::Identity()}};
        // clang-format on
        std::vector<nie::io::FixedPoseRecord> const fixes{{1}};

        nie::io::PoseHeader header{};
        nie::io::SetNieAuthority(&header);
        header.Set(nie::io::PoseHeader::Flag::kHasCodeZ);
        header.Set(nie::io::PoseHeader::Flag::kHasPoseInformationPerRecord);
        header.Set(nie::io::PoseHeader::Flag::kHasEdgeInformationPerRecord);
        header.Set(nie::io::PoseHeader::Flag::kHasTimestampPerRecord);

        return {header, poses, edges, fixes};
    }

protected:
    nie::io::PoseCollection const kCollection = CreateCollection();

    std::string const kWritePath = "pose_test.pose";
    std::string const kStreamPath = "pose_stream.pose";
    std::string const kValidatedPath = "/data/aiim/unit_tests_data/formats/pose_validated.pose";
    std::string const kHeaderOnlyPath = "pose_header.pose";

    void TearDown() override {
        std::remove(kWritePath.c_str());
        std::remove(kStreamPath.c_str());
        std::remove(kHeaderOnlyPath.c_str());
    }

    static void TestEqual(nie::io::PoseCollection const& lhs, nie::io::PoseCollection const& rhs) {
        ASSERT_TRUE(nie::io::Equals(lhs.header, rhs.header));

        ASSERT_EQ(lhs.poses.size(), rhs.poses.size());
        for (std::size_t i = 0; i < lhs.poses.size(); ++i) {
            ASSERT_TRUE(nie::io::Equals(lhs.poses[i], rhs.poses[i], lhs.header));
        }

        ASSERT_EQ(lhs.edges.size(), rhs.edges.size());
        for (std::size_t i = 0; i < lhs.edges.size(); ++i) {
            ASSERT_TRUE(nie::io::Equals(lhs.edges[i], rhs.edges[i], lhs.header));
        }

        ASSERT_EQ(lhs.fixes.size(), rhs.fixes.size());
        for (std::size_t i = 0; i < lhs.fixes.size(); ++i) {
            ASSERT_TRUE(nie::io::Equals(lhs.fixes[i], rhs.fixes[i]));
        }
    }
};

TEST_F(PoseCollectionTest, Read) {
    nie::io::PoseCollection read_collection{};
    nie::io::Read(kValidatedPath, &read_collection);
    TestEqual(kCollection, read_collection);
}

TEST_F(PoseCollectionTest, ReadStream) {
    nie::io::PoseCollection read_collection{};

    nie::io::PoseCollectionStreamReader reader(kValidatedPath);

    std::function<void(nie::io::PoseRecord)> f_add_pose = [&read_collection](nie::io::PoseRecord r) {
        read_collection.poses.push_back(std::move(r));
    };
    std::function<void(nie::io::PoseEdgeRecord)> f_add_edge = [&read_collection](nie::io::PoseEdgeRecord r) {
        read_collection.edges.push_back(std::move(r));
    };
    std::function<void(nie::io::FixedPoseRecord)> f_add_fix = [&read_collection](nie::io::FixedPoseRecord r) {
        read_collection.fixes.push_back(std::move(r));
    };

    reader.SetCallback(f_add_pose);
    reader.SetCallback(f_add_edge);
    reader.SetCallback(f_add_fix);
    read_collection.header = reader.GetHeader();
    reader.ReadRecords();

    TestEqual(kCollection, read_collection);
}

TEST_F(PoseCollectionTest, ReadHeaderOnly) {
    nie::io::PoseCollection collection_headeronly{kCollection.header, {}, {}, {}};
    ASSERT_NO_THROW(nie::io::Write(collection_headeronly, kHeaderOnlyPath));

    nie::io::PoseHeader read_collection_header{};
    nie::io::Read(kHeaderOnlyPath, &read_collection_header);

    ASSERT_TRUE(nie::io::Equals(kCollection.header, read_collection_header));
}

TEST_F(PoseCollectionTest, Write) {
    ASSERT_NO_THROW(nie::io::Write(kCollection, kWritePath));
    ASSERT_TRUE(nie::BinaryEquals(kValidatedPath, kWritePath));
}

TEST_F(PoseCollectionTest, StreamWrite) {
    // to guarantee that the destructor runs
    {
        nie::io::PoseCollectionStreamWriter stream(kStreamPath, kCollection.header);
        for (auto const& record : kCollection.poses) {
            stream.Write(record);
        }
        for (auto const& record : kCollection.edges) {
            stream.Write(record);
        }
        for (auto const& record : kCollection.fixes) {
            stream.Write(record);
        }
    }
    ASSERT_TRUE(nie::BinaryEquals(kValidatedPath, kStreamPath));
}
