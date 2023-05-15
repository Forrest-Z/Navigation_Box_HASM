/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "export_reader.hpp"
#include "standard_columns.hpp"

#include <glog/logging.h>

namespace nie {

namespace io {

namespace inertial_explorer {

namespace detail {

std::unordered_set<std::string> ColumnNamesWithSpace(StandardExportProfile const&) {
    return StandardColumnNamesWithSpace();
}

// TODO: [EDD] When moving to C++17 we can make this routine significantly faster by making line a std::string_view
std::vector<std::string> SplitAsColumnNames(
    std::string line, std::unordered_set<std::string> const& column_names_with_space) {
    // Right trim whitespace (including EOL)
    nie::RightTrim(&line);

    // Locate all possible column names in the line
    std::vector<std::string> column_names;
    size_t split_pos = 0;
    while (split_pos < line.size()) {
        // Chop of first part of line
        line = line.substr(split_pos);

        // Left trim whitespace
        nie::LeftTrim(&line);

        // Try to match a column name with space to the start of the line
        auto const column_name_it = std::find_if(
            column_names_with_space.cbegin(), column_names_with_space.cend(), [&line](std::string const& column_name) {
                return nie::StartsWith(line, column_name);
            });

        // If no column name with space was found, we split on the first space
        split_pos = column_name_it == column_names_with_space.cend() ? line.find_first_of(' ') : column_name_it->size();

        // Extract column name
        std::string const column_name = line.substr(0, split_pos);

        // Add column name to list
        column_names.push_back(column_name);
    }

    return column_names;
}

}  // namespace detail

}  // namespace inertial_explorer

}  // namespace io

}  // namespace nie
