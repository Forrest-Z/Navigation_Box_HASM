/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <cstdio>

#include <gtest/gtest.h>
#include <nie/core/filesystem.hpp>
#include <nie/formats/ba_graph.hpp>

class TwistCollectionTest : public testing::Test {
protected:
    std::string const kWritePath = "twist_test.twis";
    std::string const kStreamPath = "twist_stream.twis";
    std::string const kValidatedPath = "/data/aiim/unit_tests_data/formats/twist_validated.twis";
    std::string const kHeaderOnlyPath = "twist_header.twis";

    // clang-format off
    std::vector<nie::io::TwistRecord> const records_{
        {nie::Timestamp_ns(std::chrono::nanoseconds(1)), 1.0, 1.0},
        {nie::Timestamp_ns(std::chrono::nanoseconds(2)), 2.0, 2.0},
        {nie::Timestamp_ns(std::chrono::nanoseconds(3)), 3.0, 3.0}};
    // clang-format on
    nie::io::TwistHeader const header_{};
    nie::io::TwistCollection const collection_{header_, records_};

    void TestRecord(nie::io::TwistHeader const& lhs, nie::io::TwistHeader const& rhs) const {
        ASSERT_EQ(lhs.version, rhs.version);
    }

    void TestRecord(nie::io::TwistRecord const& lhs, nie::io::TwistRecord const& rhs) const {
        ASSERT_EQ(lhs.timestamp, rhs.timestamp);
        ASSERT_EQ(lhs.velocity, rhs.velocity);
        ASSERT_EQ(lhs.yaw_rate, rhs.yaw_rate);
    }

    void TearDown() override {
        std::remove(kWritePath.c_str());
        std::remove(kStreamPath.c_str());
        std::remove(kHeaderOnlyPath.c_str());
    }
};

TEST_F(TwistCollectionTest, Write) {
    ASSERT_NO_THROW(nie::io::Write(collection_, kWritePath));
    ASSERT_TRUE(nie::BinaryEquals(kValidatedPath, kWritePath));
}

TEST_F(TwistCollectionTest, Read) {
    nie::io::TwistCollection read_collection{};
    nie::io::Read(kValidatedPath, &read_collection);
    TestRecord(collection_.header, read_collection.header);

    ASSERT_EQ(collection_.twists.size(), read_collection.twists.size());
    for (std::size_t i = 0; i < collection_.twists.size(); ++i) {
        TestRecord(collection_.twists[i], read_collection.twists[i]);
    }
}

TEST_F(TwistCollectionTest, ReadStream) {
    nie::io::TwistCollection read_collection{};

    nie::io::TwistCollectionStreamReader reader(kValidatedPath);

    std::function<void(nie::io::TwistRecord)> f_add = [&read_collection](nie::io::TwistRecord r) {
        read_collection.twists.push_back(std::move(r));
    };
    reader.SetCallback(f_add);
    read_collection.header = reader.GetHeader();
    reader.ReadRecords();

    TestRecord(collection_.header, read_collection.header);
    ASSERT_EQ(collection_.twists.size(), read_collection.twists.size());
    for (std::size_t i = 0; i < collection_.twists.size(); ++i) {
        TestRecord(collection_.twists[i], read_collection.twists[i]);
    }
}

TEST_F(TwistCollectionTest, ReadJustHeader) {
    nie::io::TwistCollection collection_headeronly{header_, {}};
    ASSERT_NO_THROW(nie::io::Write(collection_headeronly, kHeaderOnlyPath));

    nie::io::TwistHeader read_collection_headeronly{};
    nie::io::Read(kHeaderOnlyPath, &read_collection_headeronly);
    TestRecord(collection_.header, read_collection_headeronly);
}

TEST_F(TwistCollectionTest, StreamWrite) {
    // to guarantee that the destructor runs
    {
        nie::io::TwistCollectionStreamWriter stream(kStreamPath, header_);
        for (nie::io::TwistRecord const& record : records_) {
            stream.Write(record);
        }
    }
    ASSERT_TRUE(nie::BinaryEquals(kValidatedPath, kStreamPath));
}
