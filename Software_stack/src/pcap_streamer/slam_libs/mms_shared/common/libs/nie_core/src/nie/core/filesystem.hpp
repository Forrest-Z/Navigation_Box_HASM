/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <array>
#include <fstream>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <boost/filesystem.hpp>

namespace nie {

namespace detail {

/// struct for checking if a path matches with a pattern
struct FilePathMatcher {
    static bool IsMatch(std::string const& pattern, std::string const& inpath);
};
/// struct for checking if a directory matches with a pattern
struct DirectoryPathMatcher {
    static bool IsMatch(std::string const& pattern, std::string const& inpath);
};

///
/// Note that this function will not follow symbolic links.
///
/// \tparam DirectoryIterator       Used for choosing between recursive or non-recursive.
/// \tparam PathMatcher             To decouple from std::regex.
/// \param directory                Must be directory (precondition).
/// \param pattern                  Matching pattern that works with @tparam{PathMatcher_}.
/// \return                         First matching file encountered.
template <
        class DirectoryIterator = boost::filesystem::recursive_directory_iterator,
        class PathMatcher = FilePathMatcher>
inline DirectoryIterator FindFile(
        boost::filesystem::path const& directory,
        std::string const& pattern,
        DirectoryIterator iter = DirectoryIterator()) {
    CHECK(boost::filesystem::is_directory(directory)) << "Argument 1 to function FindFiles must be a directory.";

    std::string full_pattern = (directory / pattern).string();

    DirectoryIterator const end_iter;  // Default ctor yields past-the-end
    if (iter == end_iter) iter = DirectoryIterator(directory);
    for (; iter != end_iter; ++iter) {
        // Check if file path matches pattern
        if (PathMatcher::IsMatch(full_pattern, iter->path().string())) {
            break;
        }
    }
    return iter;
}

///
/// Note that this function will not follow symbolic links.
///
/// \tparam DirectoryIterator_      Used for choosing between recursive or non-recursive.
/// \tparam PathMatcher_            To decouple from std::regex.
/// \param directory                Must be directory (precondition).
/// \param pattern                  Matching pattern that works with @tparam{PathMatcher_}.
/// \return                         Alphabetically ordered vector of filepaths.
template <
        class DirectoryIterator = boost::filesystem::recursive_directory_iterator,
        class PathMatcher = FilePathMatcher>
inline std::vector<boost::filesystem::path> FindFiles(
        boost::filesystem::path const& directory, std::string const& pattern) {
    std::vector<boost::filesystem::path> all_matching_files;

    DirectoryIterator const end_iter;  // Default ctor yields past-the-end
    for (DirectoryIterator iter(directory); iter != end_iter;) {
        iter = FindFile<DirectoryIterator, PathMatcher>(directory, pattern, iter);
        if (iter != end_iter) {
            // Store file path
            all_matching_files.push_back(iter->path());
            ++iter;  // Increment only when file was found, otherwise we are already at the end
        }
    }

    // Sort files alphabetically
    if (all_matching_files.size() > 1) {
        std::sort(all_matching_files.begin(), all_matching_files.end());
    }

    return all_matching_files;
}

template <typename T, typename Converter = std::function<T const&(T const&)>>
void WriteLines(
        std::fstream file, std::vector<T> const& lines, Converter converter = [](T const& t) { return t; }) {
    for (auto const& line : lines) {
        file << converter(line) << "\n";
    }
}

}  // namespace detail

/// Find directories matching @param{pattern} in @param{directory}.
/// Returns list of alphabetically sorted paths.
std::vector<boost::filesystem::path> FindDirectories(
        boost::filesystem::path const& directory, std::string const& pattern = ".*");

/// Find (first) file matching @param{pattern} in @param{directory}.
/// Will just return the first and not check for others, if there are.
boost::filesystem::path FindFile(boost::filesystem::path const& directory, std::string const& pattern = ".*");

/// Find (first) file matching @param{pattern} in @param{directory} and its sub directories.
/// Will just return the first and not check for others, if there are.
boost::filesystem::path FindFileRecursive(boost::filesystem::path const& directory, std::string const& pattern = ".*");

/// Find files matching @param{pattern} in @param{directory}.
/// Returns list of alphabetically sorted file paths.
std::vector<boost::filesystem::path> FindFiles(
        boost::filesystem::path const& directory, std::string const& pattern = ".*");

/// Recursively find files matching @param{pattern} in @param{directory} and its sub directories.
/// Returns list of alphabetically sorted file paths.
std::vector<boost::filesystem::path> FindFilesRecursive(
        boost::filesystem::path const& directory, std::string const& pattern = ".*");

/// Creates a file named @param{filename} in a temporary location. Makes use of
/// boost::filesystem::temp_directory_path().
boost::filesystem::path TemporaryFile(std::string const& filename);

/// Creates a file name where the base name is post fixed, as in adding "_post_fix" to "/path/file.txt" results in
/// "/path/file_post_fix.txt"
boost::filesystem::path AddPostFix(boost::filesystem::path const& filename, std::string const& post_fix);

std::fstream OpenFile(boost::filesystem::path const& path, std::ios::openmode const& mode);

bool BinaryEquals(boost::filesystem::path const& filename_a, boost::filesystem::path const& filename_b);

template <
        typename Converter = std::function<std::string(std::string const&)>,
        typename T = std::result_of_t<Converter(std::string)>>
std::vector<T> ReadLines(
        boost::filesystem::path const& filepath, Converter converter = [](std::string const& s) { return s; }) {
    auto file = OpenFile(filepath, std::ios::in);
    std::vector<T> result;
    std::string line;
    while (std::getline(file, line)) {
        result.push_back(converter(line));
    }
    return result;
}

template <typename T, typename Converter = std::function<T const&(T const&)>>
void WriteLines(
        boost::filesystem::path const& filepath, std::vector<T> const& lines, Converter converter = [](T const& t) {
            return t;
        }) {
    detail::WriteLines(OpenFile(filepath, std::ios::out), lines, converter);
}

template <typename T, typename Converter = std::function<T const&(T const&)>>
void WriteLines(
        boost::filesystem::path const& filepath,
        std::vector<T> const& lines,
        std::string const& header,
        Converter converter = [](T const& t) { return t; }) {
    auto file = OpenFile(filepath, std::ios::out);
    file << header << "\n";
    detail::WriteLines(std::move(file), lines, converter);
}

}  // namespace nie
