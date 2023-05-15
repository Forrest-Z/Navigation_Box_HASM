/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>
#include <nie/core/filesystem.hpp>
#include <nie/formats/ba_graph.hpp>

class BboxCollectionTest : public testing::Test {
protected:
    std::string const kWritePath = "bbox_test.bbox";
    std::string const kStreamPath = "bbox_stream.bbox";
    std::string const kValidatedPath = "/data/aiim/unit_tests_data/formats/bbox_validated.bbox";
    std::string const kHeaderOnlyPath = "bbox_header.bbox";

    std::vector<nie::io::BboxRecord> const records_{
        {1, {1, 1, 1}, {2, 2, 2}}, {2, {3, 3, 3}, {5, 5, 5}}, {3, {-1, -1, -1}, {8, 8, 8}}};
    nie::io::BboxHeader const header_{};
    nie::io::BboxCollection const collection_{header_, records_};

    void TestRecord(nie::io::BboxHeader const& lhs, nie::io::BboxHeader const& rhs) const {
        ASSERT_EQ(lhs.version, rhs.version);
    }

    void TestRecord(nie::io::BboxRecord const& lhs, nie::io::BboxRecord const& rhs) const {
        ASSERT_EQ(lhs.id, rhs.id);
        ASSERT_TRUE(lhs.min.isApprox(rhs.min));
        ASSERT_TRUE(lhs.max.isApprox(rhs.max));
    }

    void TearDown() override {
        std::remove(kWritePath.c_str());
        std::remove(kStreamPath.c_str());
        std::remove(kHeaderOnlyPath.c_str());
    }
};

TEST_F(BboxCollectionTest, Write) {
    ASSERT_NO_THROW(nie::io::Write(collection_, kWritePath));
    ASSERT_TRUE(nie::BinaryEquals(kValidatedPath, kWritePath));
}

TEST_F(BboxCollectionTest, Read) {
    nie::io::BboxCollection read_collection{};
    nie::io::Read(kValidatedPath, &read_collection);
    TestRecord(collection_.header, read_collection.header);

    ASSERT_EQ(collection_.boxes.size(), read_collection.boxes.size());
    for (std::size_t i = 0; i < collection_.boxes.size(); ++i) {
        TestRecord(collection_.boxes[i], read_collection.boxes[i]);
    }
}

TEST_F(BboxCollectionTest, ReadStream) {
    nie::io::BboxCollection read_collection{};

    nie::io::BboxCollectionStreamReader reader(kValidatedPath);

    std::function<void(nie::io::BboxRecord)> f_add = [&read_collection](nie::io::BboxRecord r) {
        read_collection.boxes.push_back(std::move(r));
    };
    reader.SetCallback(f_add);
    read_collection.header = reader.GetHeader();
    reader.ReadRecords();

    TestRecord(collection_.header, read_collection.header);
    ASSERT_EQ(collection_.boxes.size(), read_collection.boxes.size());
    for (std::size_t i = 0; i < collection_.boxes.size(); ++i) {
        TestRecord(collection_.boxes[i], read_collection.boxes[i]);
    }
}

TEST_F(BboxCollectionTest, ReadJustHeader) {
    nie::io::BboxCollection collection_headeronly{header_, {}};
    ASSERT_NO_THROW(nie::io::Write(collection_headeronly, kHeaderOnlyPath));

    nie::io::BboxHeader read_collection_headeronly{};
    nie::io::Read(kHeaderOnlyPath, &read_collection_headeronly);
    TestRecord(collection_.header, read_collection_headeronly);
}

TEST_F(BboxCollectionTest, StreamWrite) {
    // to guarantee that the destructor runs
    {
        nie::io::BboxCollectionStreamWriter stream(kStreamPath, header_);
        for (nie::io::BboxRecord const& record : records_) {
            stream.Write(record);
        }
    }
    ASSERT_TRUE(nie::BinaryEquals(kValidatedPath, kStreamPath));
}
