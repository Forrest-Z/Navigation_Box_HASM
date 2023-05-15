/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "io.hpp"

#include <iostream>

#include <nie/core/gflags.hpp>
#include <nie/core/string.hpp>

DEFINE_string(in_paths, "", "The input path(s) relevant for the specified tools.");
DEFINE_validator(in_paths, &nie::ValidateStringNotEmpty);

DEFINE_string(out_paths, "", "The output path(s) relevant for the specified tools.");

namespace detail {

auto constexpr kInPaths = "in_paths";
auto constexpr kOutPaths = "out_paths";

std::string NoPathWithExtension(std::string const& flagname, std::string const& extension) {
    return "-" + flagname + " has no path available with extension " + extension + ".";
}

std::string InPathDoesNotExist(std::string const& path) {
    return "-" + std::string(kInPaths) + " contains " + path + " which does not point to an existing file.";
}

std::string OutDirDoesNotExist(std::string const& path) {
    return "-" + std::string(kOutPaths) + " contains " + path + " which does not point to an existing directory.";
}

void GetPathsForExtension(
    std::string const& extension, std::string const& in_or_out_paths, std::vector<boost::filesystem::path>* paths) {
    std::vector<std::string> names = nie::Split<std::string>(in_or_out_paths);

    for (auto&& name : names) {
        boost::filesystem::path t(name);

        if (t.extension().string() == extension) {
            paths->push_back(std::move(t));
        }
    }
}

bool TryGetPathForExtension(
    std::string const& extension, std::string const& in_or_out_paths, boost::filesystem::path* p) {
    std::vector<boost::filesystem::path> paths;
    GetPathsForExtension(extension, in_or_out_paths, &paths);

    if (paths.size() > 0) {
        *p = std::move(paths[0]);
        return true;
    }

    return false;
}

bool GetAndCheckOutPathsForExtension(std::string const& extension, std::vector<boost::filesystem::path>* paths) {
    paths->emplace_back();
    bool const result = detail::TryGetPathForExtension(extension, FLAGS_out_paths, &paths->back());
    return result;
}

bool CheckDirAvailable(boost::filesystem::path const& path) {
    return boost::filesystem::exists(path) && boost::filesystem::is_directory(path);
}

void CheckDirAvailableOrFatal(boost::filesystem::path const& path) {
    CHECK(CheckDirAvailable(path)) << detail::OutDirDoesNotExist(path.string());
}

}  // namespace detail

bool GetAndCheckInPathsForExtension(std::string const& extension, std::vector<boost::filesystem::path>* paths) {
    detail::GetPathsForExtension(extension, FLAGS_in_paths, paths);

    if (paths->empty()) {
        return false;
    }

    for (auto&& path : *paths) {
        if (!boost::filesystem::exists(path)) {
            return false;
        }
    }

    return true;
}

void GetAndCheckInPathsForExtensionOrFatal(std::string const& extension, boost::filesystem::path* path) {
    std::vector<boost::filesystem::path> paths;
    if (GetAndCheckInPathsForExtension(extension, &paths)) {
        CHECK(paths.size() == 1) << "Only expecting one pose file.";
        *path = paths.front();
    }
}

boost::filesystem::path GetAndCheckInPathsForExtensionOrFatal(std::string const& extension) {
    boost::filesystem::path path;
    GetAndCheckInPathsForExtensionOrFatal(extension, &path);
    return path;
}

bool InPathExists(std::string const& extension) {
    boost::filesystem::path p;
    return detail::TryGetPathForExtension(extension, FLAGS_in_paths, &p) && boost::filesystem::exists(p);
}

void InPathExistsOrFatal(std::string const& extension) {
    CHECK(InPathExists(extension)) << detail::NoPathWithExtension(detail::kInPaths, extension);
}

void InPathExistsOrFatal() {
    CHECK(boost::filesystem::exists(boost::filesystem::path(FLAGS_in_paths)))
        << detail::InPathDoesNotExist(FLAGS_in_paths);
}

void OutDirAvailableOrFatal() { detail::CheckDirAvailableOrFatal(FLAGS_out_paths); }

void CheckOutPathsLocationsOrFatal() {
    std::vector<std::string> const paths = nie::Split<std::string>(FLAGS_out_paths);
    for (std::string const& p : paths) {
        std::string const parent_dir = boost::filesystem::path(p).parent_path().string();
        // If not local, then check directory
        if (!parent_dir.empty()) {
            detail::CheckDirAvailableOrFatal(parent_dir);
        }
    }
}

void GetAndCheckOutPathsForExtensionOrFatal(std::string const& extension, boost::filesystem::path* path) {
    std::vector<boost::filesystem::path> paths;
    CHECK(detail::GetAndCheckOutPathsForExtension(extension, &paths))
        << detail::NoPathWithExtension(detail::kOutPaths, extension);
    CHECK(paths.size() == 1) << "Expecting one " << extension << " file.";
    *path = paths.front();
}

boost::filesystem::path GetAndCheckOutPathsForExtensionOrFatal(std::string const& extension) {
    boost::filesystem::path path;
    GetAndCheckOutPathsForExtensionOrFatal(extension, &path);
    return path;
}