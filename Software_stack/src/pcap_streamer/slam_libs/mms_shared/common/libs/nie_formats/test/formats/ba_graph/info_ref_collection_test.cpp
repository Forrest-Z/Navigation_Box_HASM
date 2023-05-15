/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <cstdio>

#include <gtest/gtest.h>
#include <nie/core/filesystem.hpp>
#include <nie/formats/ba_graph.hpp>

class InfoRefsCollectionTest : public testing::Test {
protected:
    std::string const kWritePath = "inforef_test.iref";
    std::string const kStreamPath = "inforef_stream.iref";
    std::string const kValidatedPath = "/data/aiim/unit_tests_data/formats/inforef_validated.iref";
    std::string const kHeaderOnlyPath = "inforef_header.iref";

    std::vector<nie::io::InfoRefRecord> const records_{{1, 0, "a.png"}, {2, 1, "b.png"}, {3, 2, "c.png"}};
    nie::io::InfoRefHeader const header_{};
    nie::io::InfoRefCollection const collection_{header_, records_};

    void TestRecord(nie::io::InfoRefHeader const& lhs, nie::io::InfoRefHeader const& rhs) const {
        ASSERT_EQ(lhs.version, rhs.version);
    }

    void TestRecord(nie::io::InfoRefRecord const& lhs, nie::io::InfoRefRecord const& rhs) const {
        ASSERT_EQ(lhs.id, rhs.id);
        ASSERT_EQ(lhs.frame_id, rhs.frame_id);
        ASSERT_EQ(lhs.path, rhs.path);
    }

    void TearDown() override {
        std::remove(kWritePath.c_str());
        std::remove(kStreamPath.c_str());
        std::remove(kHeaderOnlyPath.c_str());
    }
};

TEST_F(InfoRefsCollectionTest, Write) {
    ASSERT_NO_THROW(nie::io::Write(collection_, kWritePath));
    ASSERT_TRUE(nie::BinaryEquals(kValidatedPath, kWritePath));
}

TEST_F(InfoRefsCollectionTest, Read) {
    nie::io::InfoRefCollection read_collection{};
    nie::io::Read(kValidatedPath, &read_collection);
    TestRecord(collection_.header, read_collection.header);

    ASSERT_EQ(collection_.info_refs.size(), read_collection.info_refs.size());
    for (std::size_t i = 0; i < collection_.info_refs.size(); ++i) {
        TestRecord(collection_.info_refs[i], read_collection.info_refs[i]);
    }
}

TEST_F(InfoRefsCollectionTest, ReadStream) {
    nie::io::InfoRefCollection read_collection{};

    nie::io::InfoRefCollectionStreamReader reader(kValidatedPath);

    std::function<void(nie::io::InfoRefRecord)> f_add = [&read_collection](nie::io::InfoRefRecord r) {
        read_collection.info_refs.push_back(std::move(r));
    };
    reader.SetCallback(f_add);
    read_collection.header = reader.GetHeader();
    reader.ReadRecords();

    TestRecord(collection_.header, read_collection.header);
    ASSERT_EQ(collection_.info_refs.size(), read_collection.info_refs.size());
    for (std::size_t i = 0; i < collection_.info_refs.size(); ++i) {
        TestRecord(collection_.info_refs[i], read_collection.info_refs[i]);
    }
}

TEST_F(InfoRefsCollectionTest, ReadJustHeader) {
    nie::io::InfoRefCollection collection_headeronly{header_, {}};
    ASSERT_NO_THROW(nie::io::Write(collection_headeronly, kHeaderOnlyPath));

    nie::io::InfoRefHeader read_collection_headeronly{};
    nie::io::Read(kHeaderOnlyPath, &read_collection_headeronly);
    TestRecord(collection_.header, read_collection_headeronly);
}

TEST_F(InfoRefsCollectionTest, StreamWrite) {
    // to guarantee that the destructor runs
    {
        nie::io::InfoRefCollectionStreamWriter stream(kStreamPath, header_);
        for (nie::io::InfoRefRecord const& record : records_) {
            stream.Write(record);
        }
    }
    ASSERT_TRUE(nie::BinaryEquals(kValidatedPath, kStreamPath));
}
