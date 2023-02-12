/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/core/filesystem.hpp>

class FilesystemF : public ::testing::Test {
protected:
    std::string const base_path = "/data/aiim/unit_tests_data/core/helper_boost_filesystem/";
    std::string const base_path_read_lines = base_path + "ReadLines/";
    std::string const base_path_find_file = base_path + "FindFiles/";
    std::string const empty_dir = "empty/";

    std::vector<std::string> const file_vector{base_path_find_file + "a.txt", base_path_find_file + "b.txt"};
    std::vector<std::string> const file_vector_recursive{base_path_find_file + "a.txt",
                                                         base_path_find_file + "b.txt",
                                                         base_path_find_file + "map/" + "a.txt",
                                                         base_path_find_file + "map/" + "b.txt",
                                                         base_path_find_file + "map/" + "c.txt"};
};

TEST_F(FilesystemF, FindFile) {
    ASSERT_FALSE(nie::FindFile(base_path_find_file, "a\\.txt").empty());
    ASSERT_TRUE(nie::FindFile(base_path_find_file, "c\\.txt").empty());
}

TEST_F(FilesystemF, FindFileRecursive) {
    ASSERT_FALSE(nie::FindFileRecursive(base_path_find_file, ".+a\\.txt").empty());
    ASSERT_FALSE(nie::FindFileRecursive(base_path_find_file, ".+c\\.txt").empty());
}

TEST_F(FilesystemF, FindFilesEmpty) {
    std::vector<boost::filesystem::path> should_be_empty = nie::FindFiles(base_path_find_file + empty_dir, ".*");
    ASSERT_TRUE(should_be_empty.empty());
}

TEST_F(FilesystemF, FindFilesNoMatch) {
    std::vector<boost::filesystem::path> should_be_empty = nie::FindFiles(base_path_find_file, "not_matching");
    ASSERT_TRUE(should_be_empty.empty());
}

TEST_F(FilesystemF, FindFiles) {
    std::vector<boost::filesystem::path> files = nie::FindFiles(base_path_find_file, ".*\\.txt");

    ASSERT_EQ(files.size(), file_vector.size());

    for (auto const& f : files) {
        ASSERT_NE(std::find(file_vector.cbegin(), file_vector.cend(), f.string()), file_vector.cend());
    }
}

TEST_F(FilesystemF, FindFilesRecursive) {
    std::vector<boost::filesystem::path> files = nie::FindFilesRecursive(base_path_find_file, ".*\\.txt");

    ASSERT_EQ(files.size(), file_vector_recursive.size());

    for (auto const& f : files) {
        ASSERT_NE(
            std::find(file_vector_recursive.cbegin(), file_vector_recursive.cend(), f.string()), file_vector.cend());
    }
}

TEST_F(FilesystemF, FindFilesSorted) {
    std::vector<boost::filesystem::path> files = nie::FindFiles(base_path_find_file, ".*\\.txt");

    ASSERT_EQ(files.size(), file_vector.size());

    auto it_str = file_vector.cbegin();

    for (auto const& f : files) {
        ASSERT_EQ(*it_str, f.string());
        ++it_str;
    }
}

TEST(Filesystem, PostFix) {
    std::string const post_fix = "_x";
    std::vector<std::pair<boost::filesystem::path, std::string>> const tests = {
        {"filename", "filename_x"},
        {"filename.txt", "filename_x.txt"},
        {"./filename.txt", "./filename_x.txt"},
        {"/tmp/filename.txt", "/tmp/filename_x.txt"}};

    for (auto const& test : tests) {
        std::string const result = nie::AddPostFix(test.first, post_fix).string();
        std::string const expectation = test.second;
        ASSERT_EQ(result, expectation);
    }
}

TEST(Filesystem, OpenFile) {
    std::fstream test_stream;
    EXPECT_NO_THROW(test_stream = nie::OpenFile("test.txt", std::ios_base::out));
    EXPECT_TRUE(test_stream.good());
    EXPECT_DEATH(test_stream = nie::OpenFile("/root/forbidden", std::ios_base::in), "");
    std::remove("test.txt");
}

TEST(Filesystem, BinaryEquals) {
    EXPECT_TRUE(nie::BinaryEquals(
        "/data/aiim/unit_tests_data/core/helper_io/a.bin", "/data/aiim/unit_tests_data/core/helper_io/a.bin"));
    EXPECT_FALSE(nie::BinaryEquals(
        "/data/aiim/unit_tests_data/core/helper_io/a.bin", "/data/aiim/unit_tests_data/core/helper_io/b.bin"));
}

TEST_F(FilesystemF, ReadLinesStrings) {
    std::vector<std::string> result = nie::ReadLines(base_path_read_lines + "abcd.txt");
    std::vector<std::string> const expected{"a", "b", "c", "d"};
    ASSERT_EQ(result, expected);
}

TEST_F(FilesystemF, ReadLinesChars) {
    std::vector<char> result =
        nie::ReadLines(base_path_read_lines + "abcd.txt", [](std::string const& s) { return s[0]; });
    std::vector<char> const expected{'a', 'b', 'c', 'd'};
    ASSERT_EQ(result, expected);
}

TEST_F(FilesystemF, ReadLinesInts) {
    std::vector<int> result =
        nie::ReadLines(base_path_read_lines + "1234.txt", [](std::string const& s) { return std::stoi(s); });
    std::vector<int> const expected{1, 2, 3, 4};
    ASSERT_EQ(result, expected);
}