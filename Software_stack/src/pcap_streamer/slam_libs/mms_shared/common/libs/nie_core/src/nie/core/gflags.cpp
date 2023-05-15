/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "gflags.hpp"
#include "filesystem.hpp"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <iostream>

namespace nie {

namespace detail {

bool Validate(bool const pre_conditions_valid, bool const valid, std::string const& message) {
    if (not pre_conditions_valid) {
        return false;
    }
    if (not valid) {
        std::cout << message << std::endl;
    }
    return valid;
}

}  // namespace detail

bool ValidateStringNotEmpty(char const* flag_name, std::string const& value) {
    return detail::Validate(true, not value.empty(), std::string("No value set for flag ") + flag_name);
}

bool ValidatePathExists(char const* flag_name, std::string const& value) {
    return detail::Validate(
        ValidateStringNotEmpty(flag_name, value),
        boost::filesystem::exists(value),
        std::string("Path set for flag ") + flag_name + " does not exist");
}

bool ValidateIsDirectory(char const* flag_name, std::string const& value) {
    return detail::Validate(
        ValidatePathExists(flag_name, value),
        boost::filesystem::is_directory(value),
        std::string("Path set for flag ") + flag_name + " is not a directory");
}

bool ValidateIsFile(char const* flag_name, std::string const& value) {
    return detail::Validate(
        ValidatePathExists(flag_name, value),
        boost::filesystem::is_regular_file(value),
        std::string("Path set for flag ") + flag_name + " is not a file");
}

bool ValidateParentDirExists(char const* flag_name, std::string const& value) {
    auto const input_path_parent = boost::filesystem::path(value).parent_path();
    return detail::Validate(
        ValidateStringNotEmpty(flag_name, value),
        input_path_parent.empty() || boost::filesystem::is_directory(input_path_parent),
        std::string("Path set for flag ") + flag_name + " does not point to an existing directory.");
}

bool WriteFlagsFile(std::string const& out_path_flags) {
    std::string flags_string = gflags::CommandlineFlagsIntoString();
    std::ofstream out(out_path_flags);
    nie::OpenFile(out_path_flags, std::ios::in);
    out << flags_string;
    out.close();

    return true;
}

}  // namespace nie
