/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
//
// Created by robert.vaneerdewijk on 10/24/19.
//
#include <gtest/gtest.h>
#include <string>

#include <nie/core/string.hpp>

#include <nie/formats/csv/csv_columns.hpp>

class CsvColumnsTest : public testing::Test {
protected:
    std::vector<std::string> const kCsvStrings{
        "0,0.0,\"0\"", "1,1.0,\"1\"", "2,2.0,\"2\"", "3,3.0,\"3\"", "4,4.0,\"4\""};
    char const kDelim = ',';

    std::vector<int> const kCol0{0, 1, 2, 3, 4};
    std::vector<float> const kCol1{0, 1, 2, 3, 4};
    std::vector<std::string> const kCol2{"0", "1", "2", "3", "4"};
};

TEST_F(CsvColumnsTest, ConstructFromVectors) {
    nie::CsvColumns<int, float, std::string> csv_columns{kCol0, kCol1, kCol2};

    for (size_t i = 0; i < csv_columns.NumRows(); ++i) {
        auto row = csv_columns.GetRow(i);

        ASSERT_EQ(std::get<0>(row), i);
        ASSERT_EQ(std::get<1>(row), static_cast<float>(i));
        ASSERT_EQ(std::get<2>(row), std::to_string(i));
    }
}

TEST_F(CsvColumnsTest, ConstructFromVectorsRvalue) {
    std::vector<int> col0 = kCol0;
    std::vector<float> col1 = kCol1;
    std::vector<std::string> col2 = kCol2;

    nie::CsvColumns<int, float, std::string> csv_columns{std::move(col0), std::move(col1), std::move(col2)};

    for (size_t i = 0; i < csv_columns.NumRows(); ++i) {
        auto row = csv_columns.GetRow(i);

        ASSERT_EQ(std::get<0>(row), i);
        ASSERT_EQ(std::get<1>(row), static_cast<float>(i));
        ASSERT_EQ(std::get<2>(row), std::to_string(i));
    }
}

TEST_F(CsvColumnsTest, ConstructFromVectorsUnequalSizes) {
    std::vector<int> col0 = kCol0;
    std::vector<float> col1 = kCol1;
    std::vector<std::string> col2 = kCol2;

    col0.push_back(0);

    auto construct_csv_columns = [&]() { nie::CsvColumns<int, float, std::string> csv_columns{col0, col1, col2}; };

    ASSERT_DEATH(construct_csv_columns(), "Not allowed to create CsvColumn from unequally sized vectors.");
}

TEST_F(CsvColumnsTest, GetCol) {
    nie::CsvColumns<int, float, std::string> csv_columns{kCol0, kCol1, kCol2};

    ASSERT_EQ(csv_columns.GetCol<0>(), kCol0);
    ASSERT_EQ(csv_columns.GetCol<1>(), kCol1);
    ASSERT_EQ(csv_columns.GetCol<2>(), kCol2);
}

TEST_F(CsvColumnsTest, PushBack) {
    nie::CsvColumns<int, float, std::string> csv_columns;

    std::tuple<int, float, std::string> row_tuple{0, 0.0, "0"};

    ASSERT_NO_FATAL_FAILURE(csv_columns.PushBack(row_tuple));
}

TEST_F(CsvColumnsTest, PushBackRvalue) {
    nie::CsvColumns<int, float, std::string> csv_columns;

    ASSERT_NO_FATAL_FAILURE(csv_columns.PushBack({0, 0.0, "0"}));
}

TEST_F(CsvColumnsTest, PushBackVariadic) {
    nie::CsvColumns<int, float, std::string> csv_columns;

    ASSERT_NO_FATAL_FAILURE(csv_columns.PushBack(0, 0.0, "0"));
}

TEST_F(CsvColumnsTest, ConstructFromStrings) {
    nie::CsvColumns<int, float, std::string> csv_columns{kCsvStrings.cbegin(), kCsvStrings.cend(), kDelim};

    ASSERT_EQ(csv_columns.GetCol<0>(), kCol0);
    ASSERT_EQ(csv_columns.GetCol<1>(), kCol1);
    ASSERT_EQ(csv_columns.GetCol<2>(), kCol2);
}

TEST_F(CsvColumnsTest, GetRowConst) {
    nie::CsvColumns<int, float, std::string> csv_columns{kCol0, kCol1, kCol2};

    // Get const ref to the csv columns
    auto const& csv_columns_const = csv_columns;

    ASSERT_EQ(csv_columns_const.GetCol<0>(), kCol0);
    ASSERT_EQ(csv_columns_const.GetCol<1>(), kCol1);
    ASSERT_EQ(csv_columns_const.GetCol<2>(), kCol2);

    // Check the values
    for (size_t i = 0; i < csv_columns_const.NumRows(); ++i) {
        auto row = csv_columns_const.GetRow(i);

        ASSERT_EQ(std::get<0>(row), i);
        ASSERT_EQ(std::get<1>(row), static_cast<float>(i));
        ASSERT_EQ(std::get<2>(row), std::to_string(i));
    }
}

TEST_F(CsvColumnsTest, GetRow) {
    nie::CsvColumns<int, float, std::string> csv_columns{kCol0, kCol1, kCol2};

    // Check the values
    ASSERT_EQ(csv_columns.GetCol<0>(), kCol0);
    ASSERT_EQ(csv_columns.GetCol<1>(), kCol1);
    ASSERT_EQ(csv_columns.GetCol<2>(), kCol2);

    // Get values by reference and change them
    for (size_t i = 0; i < 5; ++i) {
        auto row = csv_columns.GetRow(i);

        std::get<0>(row) += 1;
    }

    // Check the values, should have been changed
    for (size_t i = 0; i < csv_columns.NumRows(); ++i) {
        auto row = csv_columns.GetRow(i);

        ASSERT_EQ(std::get<0>(row), i + 1);
        ASSERT_EQ(std::get<1>(row), static_cast<float>(i));
        ASSERT_EQ(std::get<2>(row), std::to_string(i));
    }
}

/// Tests with more complex string values
class CsvColumnsStringTest : public testing::Test {
protected:
    std::vector<std::string> const kCsvStrings{"0,\"0\",\"0,0\"",
                                               "1,\"1\",\"\\\"1,1\\\"\"",
                                               "2,\"2\",\"2\\\"1,2,3\\\",\\\"2,3,4\\\"\"",
                                               "3,\"3\",\"3\"",
                                               "4,\"4\",\"4\""};
    char const kDelim = ',';

    std::vector<int> const kCol0{0, 1, 2, 3, 4};
    std::vector<std::string> const kCol1{"0", "1", "2", "3", "4"};
    std::vector<std::string> const kCol2{"0,0", "\\\"1,1\\\"", "2\\\"1,2,3\\\",\\\"2,3,4\\\"", "3", "4"};
};

TEST_F(CsvColumnsStringTest, ConstructFromStrings) {
    FLAGS_v = 6;
    nie::CsvColumns<int, std::string, std::string> csv_columns{kCsvStrings.cbegin(), kCsvStrings.cend(), kDelim};

    ASSERT_EQ(csv_columns.GetCol<0>(), kCol0);
    ASSERT_EQ(csv_columns.GetCol<1>(), kCol1);
    ASSERT_EQ(csv_columns.GetCol<2>(), kCol2);
}