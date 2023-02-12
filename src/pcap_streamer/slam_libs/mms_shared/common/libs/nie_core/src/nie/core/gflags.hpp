/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <string>

namespace nie {

namespace detail {

bool Validate(bool const pre_conditions_valid, bool const valid, std::string const& message);

}  // namespace detail

// Checks if supplied string is not empty
bool ValidateStringNotEmpty(char const* flag_name, std::string const& value);

// Checks if supplied path exists, but first also checks
//  - ValidateStringNotEmpty
bool ValidatePathExists(char const* flag_name, std::string const& value);

// Checks if supplied path is a directory, but first also checks
//  - ValidatePathExists
bool ValidateIsDirectory(char const* flag_name, std::string const& value);

// Checks if supplied path is a file, but first also checks
//  - ValidatePathExists
bool ValidateIsFile(char const* flag_name, std::string const& value);

// Checks if the directory exists for the supplied path to a file, but first also checks
//  - ValidateStringNotEmpty
bool ValidateParentDirExists(char const* flag_name, std::string const& value);

// Checks if supplied value is larger than zero
template <typename T>
bool ValidateLargerThanZero(char const* flag_name, T value) {
    static_assert(
            std::is_arithmetic<T>::value, "Argument \"value\" to ValidateLargerThanZero() must be arithmetic type.");
    return detail::Validate(
            true,
            value > static_cast<T>(0),
            std::string("Value set for flag ") + flag_name + " is not larger than zero");
}

template <typename T>
bool ValidateLargerOrEqualToZero(char const* flag_name, T value) {
    static_assert(
            std::is_arithmetic<T>::value,
            "Argument \"value\" to ValidateLargerOrEqualToZero() must be arithmetic type.");
    return detail::Validate(
            true,
            value >= static_cast<T>(0),
            std::string("Value set for flag ") + flag_name + " is not larger or equal to zero");
}

// Writes file containing Gflags into out_path_flags
//
bool WriteFlagsFile(std::string const& out_path_flags);

}  // namespace nie
