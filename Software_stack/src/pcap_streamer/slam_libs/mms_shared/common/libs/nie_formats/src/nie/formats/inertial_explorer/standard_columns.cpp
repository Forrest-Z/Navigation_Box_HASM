/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "standard_columns.hpp"

#include <string>

std::unordered_set<std::string> nie::io::inertial_explorer::StandardColumnNamesWithSpace() {
    static std::unordered_set<std::string> column_names_with_space;

    // Only initialize if we have not yet found a column name with space
    // NOTE: This works because we know there is at least 1 standard column name with space
    if (column_names_with_space.empty()) {
        // Loop over all standard columns
        for (auto const& column_info : kStandardColumnInfo) {
            // Get column name
            std::string const column_name = column_info.second.name;

            // Add the column name to the list if it contains a space
            if (column_name.find(' ') != std::string::npos) {
                column_names_with_space.insert(column_name);
            }
        }
    }

    return column_names_with_space;
}
