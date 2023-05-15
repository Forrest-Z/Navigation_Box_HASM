/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/formats/inertial_explorer/project_name.hpp>

TEST(ProjectName, ParseInvalidProjectName) {
    // clang-format off
    std::vector<std::string> const invalid_project_names{
        "",                          // Empty
        "abcdef",                    // Invalid format
        "1111111111",                // Invalid format (length)
        "ccccnsvvyymmdd",            // Proper format (excl dashes), but not using numbers
        "cccc-n-svv-yymmdd",         // Proper format (incl dashes), but not using numbers
        "0000-1-006-190229"};        // Proper format, but with invalid date (2019 is not a leap year)
    // clang-format on

    for (auto const& project_name : invalid_project_names) {
        EXPECT_THROW(nie::io::inertial_explorer::ParseDateFromProjectName(project_name), std::runtime_error);
    }
}

TEST(ProjectName, ParseValidProjectName) {
    // clang-format off
    std::vector<std::tuple<std::string, std::chrono::year_month_day>> const valid_project_names{
        {"0000-1-006-190103",       std::chrono::year(2019)/01/03},
        {"0000-0-106-190519",       std::chrono::year(2019)/05/19},
        {"12340106000101",          std::chrono::year(2000)/01/01},
        {"8888-1765991231",         std::chrono::year(2099)/12/31},
        {"00001006190511123456",    std::chrono::year(2019)/05/11}, // Proper project name, but with 6 digit time postfix
        {"0000-1-006-190511-00000", std::chrono::year(2019)/05/11}  // Proper project name, but with 5 digit identifier postfix
    };
    // clang-format on

    for (auto const& project_name : valid_project_names) {
        EXPECT_EQ(
                nie::io::inertial_explorer::ParseDateFromProjectName(std::get<0>(project_name)),
                std::get<1>(project_name));
    }
}
