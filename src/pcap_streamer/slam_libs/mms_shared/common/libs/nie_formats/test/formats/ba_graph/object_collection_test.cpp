/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <cstdio>

#include <gtest/gtest.h>
#include <nie/core/filesystem.hpp>
#include <nie/formats/ba_graph.hpp>

class ObjectCollectionTest : public testing::Test {
private:
    // Helper functions
    static Eigen::Matrix3d CreateSymmetricMatrix() {
        Eigen::Matrix3d m;
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t col = row; col < 3; ++col) {
                std::size_t value = row * 3 + col;
                m(row, col) = value;
                m(col, row) = value;
            }
        }
        return m;
    }
    static nie::io::ObjectHeader::Flags CreateFlags() {
        nie::io::ObjectHeader::Flags flags{0};
        flags |= nie::io::ObjectHeader::Flag::kHasInformation;
        return flags;
    }

protected:
    std::string const kWritePath = "object_test.objt";
    std::string const kStreamPath = "object_stream.objt";
    std::string const kValidatedPath = "/data/aiim/unit_tests_data/formats/object_validated.objt";
    std::string const kHeaderOnlyPath = "object_header.objt";

    std::vector<nie::io::ObjectRecord> const records_{
        {1, {1., 1., 1.}, {}}, {2, {2., 2., 2.}, {}}, {3, {3., 3., 3.}, {}}};
    nie::io::ObjectHeader const header_{
        nie::io::kObjectCollectionVersionLatest, CreateFlags(), CreateSymmetricMatrix()};
    nie::io::ObjectCollection const collection_{header_, records_};

    void TestObjectCollectionHeader(nie::io::ObjectHeader const& lhs, nie::io::ObjectHeader const& rhs) const {
        ASSERT_EQ(lhs.version, rhs.version);
        ASSERT_EQ(lhs.flags, rhs.flags);
        if (lhs.flags & nie::io::ObjectHeader::Flag::kHasInformation) {
            ASSERT_EQ(lhs.information, rhs.information);
        }
    }

    void TestObjectRecord(
        nie::io::ObjectHeader const& header, nie::io::ObjectRecord const& lhs, nie::io::ObjectRecord const& rhs) const {
        ASSERT_EQ(lhs.id, rhs.id);
        ASSERT_EQ(lhs.position, rhs.position);
        if (header.flags & nie::io::ObjectHeader::Flag::kHasInformationPerRecord) {
            ASSERT_EQ(lhs.information, rhs.information);
        }
    }

    void TearDown() override {
        std::remove(kWritePath.c_str());
        std::remove(kStreamPath.c_str());
        std::remove(kHeaderOnlyPath.c_str());
    }
};

TEST_F(ObjectCollectionTest, Read) {
    nie::io::ObjectCollection read_collection{};
    nie::io::Read(kValidatedPath, &read_collection);

    TestObjectCollectionHeader(collection_.header, read_collection.header);

    ASSERT_EQ(collection_.objects.size(), read_collection.objects.size());
    for (std::size_t i = 0; i < collection_.objects.size(); ++i) {
        TestObjectRecord(collection_.header, collection_.objects[i], read_collection.objects[i]);
    }
}

TEST_F(ObjectCollectionTest, ReadStream) {
    nie::io::ObjectCollection read_collection{};

    nie::io::ObjectCollectionStreamReader reader(kValidatedPath);

    std::function<void(nie::io::ObjectRecord)> f_add = [&read_collection](nie::io::ObjectRecord r) {
        read_collection.objects.push_back(std::move(r));
    };
    reader.SetCallback(f_add);
    read_collection.header = reader.GetHeader();
    reader.ReadRecords();

    TestObjectCollectionHeader(collection_.header, read_collection.header);

    ASSERT_EQ(collection_.objects.size(), read_collection.objects.size());
    for (std::size_t i = 0; i < collection_.objects.size(); ++i) {
        TestObjectRecord(collection_.header, collection_.objects[i], read_collection.objects[i]);
    }
}

TEST_F(ObjectCollectionTest, ReadJustHeader) {
    nie::io::ObjectCollection collection_headeronly{header_, {}};
    ASSERT_NO_THROW(nie::io::Write(collection_headeronly, kHeaderOnlyPath));

    nie::io::ObjectHeader read_collection_headeronly{};
    nie::io::Read(kHeaderOnlyPath, &read_collection_headeronly);

    TestObjectCollectionHeader(collection_headeronly.header, read_collection_headeronly);
}

TEST_F(ObjectCollectionTest, Write) {
    ASSERT_NO_THROW(nie::io::Write(collection_, kWritePath));
    ASSERT_TRUE(nie::BinaryEquals(kValidatedPath, kWritePath));
}

TEST_F(ObjectCollectionTest, StreamWrite) {
    // to guarantee that the destructor runs
    {
        nie::io::ObjectCollectionStreamWriter stream(kStreamPath, header_);
        for (nie::io::ObjectRecord const& record : records_) {
            stream.Write(record);
        }
    }
    ASSERT_TRUE(nie::BinaryEquals(kValidatedPath, kStreamPath));
}
