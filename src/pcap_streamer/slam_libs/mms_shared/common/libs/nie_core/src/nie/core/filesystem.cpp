/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "filesystem.hpp"

#include <algorithm>
#include <regex>
#include <sstream>
#include <streambuf>

#include <boost/iostreams/device/mapped_file.hpp>

#include "string.hpp"

namespace nie {

namespace detail {

bool FilePathMatcher::IsMatch(std::string const& pattern, std::string const& inpath) {
    // Check if it is a file
    if (!boost::filesystem::is_regular_file(inpath)) {
        return false;
    }
    std::smatch what;
    return std::regex_match(inpath, what, std::regex{pattern});
}

bool DirectoryPathMatcher::IsMatch(std::string const& pattern, std::string const& inpath) {
    // Check if it is a file
    if (!boost::filesystem::is_directory(inpath)) {
        return false;
    }
    std::smatch what;
    return std::regex_match(inpath, what, std::regex{pattern});
}

}  // namespace detail

std::vector<boost::filesystem::path> FindDirectories(
    boost::filesystem::path const& directory, std::string const& pattern) {
    return detail::FindFiles<boost::filesystem::directory_iterator, detail::DirectoryPathMatcher>(directory, pattern);
}

boost::filesystem::path FindFile(boost::filesystem::path const& directory, std::string const& pattern) {
    auto iter = detail::FindFile<boost::filesystem::directory_iterator>(directory, pattern);
    return iter != boost::filesystem::directory_iterator() ? iter->path() : "";
}

boost::filesystem::path FindFileRecursive(boost::filesystem::path const& directory, std::string const& pattern) {
    auto iter = detail::FindFile<boost::filesystem::recursive_directory_iterator>(directory, pattern);
    return iter != boost::filesystem::recursive_directory_iterator() ? iter->path() : "";
}

std::vector<boost::filesystem::path> FindFiles(boost::filesystem::path const& directory, std::string const& pattern) {
    return detail::FindFiles<boost::filesystem::directory_iterator>(directory, pattern);
}

std::vector<boost::filesystem::path> FindFilesRecursive(
    boost::filesystem::path const& directory, std::string const& pattern) {
    return detail::FindFiles<boost::filesystem::recursive_directory_iterator>(directory, pattern);
}

boost::filesystem::path AddPostFix(boost::filesystem::path const& filename, std::string const& post_fix) {
    boost::filesystem::path result = filename.stem();
    result += post_fix;  // There is no + operator, only +=
    result += filename.extension();
    if (filename.has_parent_path()) {
        result = filename.parent_path() / result;
    }
    return result;
}

boost::filesystem::path TemporaryFile(std::string const& filename) {
    return boost::filesystem::temp_directory_path() / filename;
}

std::fstream OpenFile(boost::filesystem::path const& path, std::ios::openmode const& mode) {
    std::fstream stream;

    // stream.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    stream.open(path.c_str(), mode);

    CHECK(!stream.fail()) << "nie::io::OpenFile(): Unable to open: " << path;

    return stream;
}

bool BinaryEquals(boost::filesystem::path const& filename_a, boost::filesystem::path const& filename_b) {
    // Use memory mapped files for high performance, see http://www.cplusplus.com/forum/general/94032
    boost::iostreams::mapped_file_source file_a(filename_a);
    boost::iostreams::mapped_file_source file_b(filename_b);

    // Compare file size for early out
    return file_a.size() == file_b.size() && std::equal(file_a.data(), file_a.data() + file_a.size(), file_b.data());
}

}  // namespace nie
