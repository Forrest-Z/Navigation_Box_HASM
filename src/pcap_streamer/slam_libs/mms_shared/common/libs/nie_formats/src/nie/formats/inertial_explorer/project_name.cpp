/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "project_name.hpp"

#include <regex>

#include <nie/core/glog.hpp>

namespace nie {
namespace io {
namespace inertial_explorer {

std::chrono::year_month_day ParseDateFromProjectName(std::string const& project_name) {
    // Define regular expressions for project name
    static std::regex const project_name_regex("^[0-9]{4}-?[0|1]-?[0-9]{3}-?([0-9]{2})([0-9]{2})([0-9]{2}).*");

    // Try to match the project name with one of the regular expressions
    std::smatch match;
    if (!std::regex_match(project_name, match, project_name_regex)) {
        throw std::runtime_error("Project name '" + project_name + "' has an unexpected format");
    }

    // Sanity check
    CHECK(match.size() == 4);

    // Extract the year, month, and day
    std::chrono::year const yyyy{std::stoi(match[1]) + 2000};
    std::chrono::month const mm{stou(match[2])};
    std::chrono::day const dd{stou(match[3])};

    // Create a date
    // NOTE: Slashes should NOT be interpreted as division operator, but as the standard date separator,
    //       see: https://en.cppreference.com/w/cpp/chrono/operator_slash
    std::chrono::year_month_day const ymd = yyyy / mm / dd;

    // Verify that the matched date is valid
    if (!ymd.ok()) {
        throw std::runtime_error("Project name '" + project_name + "' contains an invalid date");
    }

    return ymd;
}

}  // namespace inertial_explorer
}  // namespace io
}  // namespace nie
