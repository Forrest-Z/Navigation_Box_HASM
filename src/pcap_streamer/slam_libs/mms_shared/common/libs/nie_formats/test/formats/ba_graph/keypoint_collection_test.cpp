/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <cstdio>

#include <gtest/gtest.h>
#include <nie/core/filesystem.hpp>
#include <nie/formats/ba_graph.hpp>

class KeypointCollectionTest : public testing::Test {
private:
    // Helper functions
    static Eigen::Matrix2d CreateSymmetricMatrix() {
        Eigen::Matrix2d m;
        for (std::size_t row = 0; row < 2; ++row) {
            for (std::size_t col = row; col < 2; ++col) {
                std::size_t value = row * 2 + col;
                m(row, col) = value;
                m(col, row) = value;
            }
        }
        return m;
    }
    static nie::io::KeypointHeader::Flags CreateFlags() {
        nie::io::KeypointHeader::Flags flags{0};
        flags |= nie::io::KeypointHeader::Flag::kHasInformation;
        return flags;
    }

protected:
    std::string const kWritePath = "keypoint_test.kpnt";
    std::string const kStreamPath = "keypoint_stream.kpnt";
    std::string const kValidatedPath = "/data/aiim/unit_tests_data/formats/keypoint_validated.kpnt";
    std::string const kHeaderOnlyPath = "keypoint_header.kpnt";

    std::vector<nie::io::KeypointRecord> const records_{
        {1, 0, 1, {1., 1.}, {}}, {2, 0, 2, {2., 2.}, {}}, {3, 0, 3, {3., 3.}, {}}};
    nie::io::KeypointHeader const header_{
        nie::io::kKeypointCollectionVersionLatest, CreateFlags(), CreateSymmetricMatrix()};
    nie::io::KeypointCollection const collection_{header_, records_};

    void TestKeypointCollectionHeader(nie::io::KeypointHeader const& lhs, nie::io::KeypointHeader const& rhs) const {
        ASSERT_EQ(lhs.version, rhs.version);
        ASSERT_EQ(lhs.flags, rhs.flags);
        if (lhs.flags & nie::io::KeypointHeader::Flag::kHasInformation) {
            ASSERT_EQ(lhs.information, rhs.information);
        }
    }

    void TestKeypointRecord(
        nie::io::KeypointHeader const& header,
        nie::io::KeypointRecord const& lhs,
        nie::io::KeypointRecord const& rhs) const {
        ASSERT_EQ(lhs.pose_id, rhs.pose_id);
        ASSERT_EQ(lhs.frame_id, rhs.frame_id);
        ASSERT_EQ(lhs.object_id, rhs.object_id);
        ASSERT_EQ(lhs.position, rhs.position);
        if (header.flags & nie::io::KeypointHeader::Flag::kHasInformationPerRecord) {
            ASSERT_EQ(lhs.information, rhs.information);
        }
    }

    void TearDown() override {
        std::remove(kWritePath.c_str());
        std::remove(kStreamPath.c_str());
        std::remove(kHeaderOnlyPath.c_str());
    }
};

TEST_F(KeypointCollectionTest, Read) {
    nie::io::KeypointCollection read_collection{};
    nie::io::Read(kValidatedPath, &read_collection);

    TestKeypointCollectionHeader(collection_.header, read_collection.header);

    ASSERT_EQ(collection_.keypoints.size(), read_collection.keypoints.size());
    for (std::size_t i = 0; i < collection_.keypoints.size(); ++i) {
        TestKeypointRecord(collection_.header, collection_.keypoints[i], read_collection.keypoints[i]);
    }
}

TEST_F(KeypointCollectionTest, ReadStream) {
    nie::io::KeypointCollection read_collection{};

    nie::io::KeypointCollectionStreamReader reader(kValidatedPath);

    std::function<void(nie::io::KeypointRecord)> f_add = [&read_collection](nie::io::KeypointRecord r) {
        read_collection.keypoints.push_back(std::move(r));
    };
    reader.SetCallback(f_add);
    read_collection.header = reader.GetHeader();
    reader.ReadRecords();

    TestKeypointCollectionHeader(collection_.header, read_collection.header);
    ASSERT_EQ(collection_.keypoints.size(), read_collection.keypoints.size());
    for (std::size_t i = 0; i < collection_.keypoints.size(); ++i) {
        TestKeypointRecord(collection_.header, collection_.keypoints[i], read_collection.keypoints[i]);
    }
}

TEST_F(KeypointCollectionTest, ReadJustHeader) {
    nie::io::KeypointCollection collection_headeronly{header_, {}};
    ASSERT_NO_THROW(nie::io::Write(collection_headeronly, kHeaderOnlyPath));

    nie::io::KeypointHeader read_collection_headeronly{};
    nie::io::Read(kHeaderOnlyPath, &read_collection_headeronly);

    TestKeypointCollectionHeader(collection_headeronly.header, read_collection_headeronly);
}

TEST_F(KeypointCollectionTest, Write) {
    ASSERT_NO_THROW(nie::io::Write(collection_, kWritePath));
    ASSERT_TRUE(nie::BinaryEquals(kValidatedPath, kWritePath));
}

TEST_F(KeypointCollectionTest, StreamWrite) {
    // to guarantee that the destructor runs
    {
        nie::io::KeypointCollectionStreamWriter stream(kStreamPath, header_);
        for (nie::io::KeypointRecord const& record : records_) {
            stream.Write(record);
        }
    }
    ASSERT_TRUE(nie::BinaryEquals(kValidatedPath, kStreamPath));
}
