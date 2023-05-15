/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
//
// Created by robert.vaneerdewijk on 9/18/19.
//

#include <gtest/gtest.h>

#include <nie/core/string.hpp>

#include <nie/formats/csv/csv_read.hpp>
#include <nie/formats/csv/csv_record.hpp>
#include <nie/formats/csv/csv_write.hpp>

class CsvIoTest : public testing::Test {
protected:
    std::string const kDirectoryPath = "/data/aiim/unit_tests_data/position_estimator/tools/csv_merge";

    std::vector<std::string> control_output{
        "a;b;c;d", "1;1;1;1", "2;2;2;2", "2;2;2;2", "3;3;3;3", "3;3;3;3", "3;3;3;3", "4;4;4;4", "4;4;4;4", "4;4;4;4",
        "4;4;4;4", "8;8;8;8", "8;8;8;8", "8;8;8;8", "8;8;8;8", "8;8;8;8", "8;8;8;8", "8;8;8;8", "8;8;8;8", "9;9;9;9",
        "9;9;9;9", "9;9;9;9", "9;9;9;9", "9;9;9;9", "9;9;9;9", "9;9;9;9", "9;9;9;9", "9;9;9;9"};

    struct CsvReference {
        std::string const kFilename;
        std::vector<std::string> const kControlLines;
    };

    // clang-format off
    std::vector<CsvReference> test_vector {
        CsvReference{
            "abcd-1.csv",
            {
                "a;b;c;d",
                "1;1;1;1"
            }
        },
        CsvReference{
            "abcd-2.csv",
            {
                "a;b;c;d",
                "2;2;2;2",
                "2;2;2;2"
            }
        },
        CsvReference{
            "abcd-3.csv",
            {
                "a;b;c;d",
                "3;3;3;3",
                "3;3;3;3",
                "3;3;3;3"
            }
        },
        CsvReference{
            "abcd-4.csv",
            {
                "a;b;c;d",
                "4;4;4;4",
                "4;4;4;4",
                "4;4;4;4",
                "4;4;4;4"
            }
        },
        CsvReference{
            "abcd-8-with-redundant-lines-before-header.csv",
            {
                "a;b;c;d",
                "8;8;8;8",
                "8;8;8;8",
                "8;8;8;8",
                "8;8;8;8",
                "8;8;8;8",
                "8;8;8;8",
                "8;8;8;8",
                "8;8;8;8"
            }
        },
        CsvReference{
            "abcd-9.csv",
            {
                "a;b;c;d",
                "9;9;9;9",
                "9;9;9;9",
                "9;9;9;9",
                "9;9;9;9",
                "9;9;9;9",
                "9;9;9;9",
                "9;9;9;9",
                "9;9;9;9",
                "9;9;9;9"
            }
        }
    };
    // clang-format on
};

TEST_F(CsvIoTest, Read) {
    for (CsvReference const& csv_reference : test_vector) {
        std::vector<std::string> csv_lines = nie::ReadAllCsv<nie::ReadLinesAfterHeaderPredicate>(
            {kDirectoryPath + std::string{"/"} + csv_reference.kFilename},
            "a;b;c;d",
            nie::RegexRejectPredicate{"^(?!(^.*;.*;.*;.*$)).*$"},
            nie::RegexAcceptPredicate{".*"});

        ASSERT_EQ(csv_lines, csv_reference.kControlLines);
    }
}

TEST_F(CsvIoTest, Merge) {
    std::vector<boost::filesystem::path> files = nie::FindFiles(kDirectoryPath, ".*.csv");

    std::vector<std::string> csv_lines = nie::ReadAllCsv<nie::ReadLinesAfterHeaderPredicate>(
        files, "a;b;c;d", nie::RegexRejectPredicate{"^(?!(^.*;.*;.*;.*$)).*$"}, nie::RegexAcceptPredicate{".*"});

    EXPECT_EQ(csv_lines, control_output);
}
