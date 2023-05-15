/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_INERTIAL_EXPLORER_MAPPED_FIELDS_HPP
#define NIE_FORMATS_INERTIAL_EXPLORER_MAPPED_FIELDS_HPP

#include <string>
#include <unordered_map>
#include <vector>

namespace nie {

namespace io {

namespace inertial_explorer {

/// Utility class to allow indexing fields by column identifier (To make it order independent)
template <typename ColumnId>
class MappedFields {
public:
    // Use an unordered_map to provide a mapping between ColumnId and the index into the fields vector; Note that
    // ColumnId may be any (hasable) type; if ColumnId would be integer we could replace this with a vector
    using Mapping = std::unordered_map<ColumnId, std::size_t>;
    using Fields = std::vector<std::string>;

    MappedFields(Mapping const& mapping, Fields const& fields) : mapping_(mapping), fields_(fields) {}

    std::string const& operator[](ColumnId const& id) const { return fields_.at(mapping_.at(id)); }

private:
    Mapping const& mapping_;
    Fields const& fields_;
};

// Given parsed column names, try to find a mapping from expected columns to parsed column names
template <typename ExportProfile>
bool DetermineColumnMapping(
    std::vector<std::string> const& parsed_column_names, typename ExportProfile::ColumnMapping* column_mapping) {
    // Clear any existing mapping
    column_mapping->clear();

    // Loop over all expected columns
    for (auto const& expected_column_id : ExportProfile::expected_column_ids) {
        // Obtain the name of the expected column
        auto const& column_name = ExportProfile::ColumnName(expected_column_id);

        // Try to find the expected column name in the parsed column names
        auto const it = std::find(parsed_column_names.cbegin(), parsed_column_names.cend(), column_name);

        // If we were unable to find it, the parsed column names do not match the specified export profile
        if (it == parsed_column_names.cend()) return false;

        // Get the index of the column from the iterator
        std::size_t const index = std::distance(parsed_column_names.cbegin(), it);

        // Store the mapping so we can easily parse data rows
        column_mapping->insert({expected_column_id, index});
    }

    return true;
}

}  // namespace inertial_explorer

}  // namespace io

}  // namespace nie

#endif  // NIE_FORMATS_INERTIAL_EXPLORER_MAPPED_FIELDS_HPP
