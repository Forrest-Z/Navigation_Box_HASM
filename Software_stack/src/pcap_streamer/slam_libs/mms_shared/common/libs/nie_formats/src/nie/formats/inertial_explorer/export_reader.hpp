/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_INERTIAL_EXPLORER_EXPORT_READER_HPP
#define NIE_FORMATS_INERTIAL_EXPLORER_EXPORT_READER_HPP

#include <fstream>

#include <nie/core/filesystem.hpp>
#include <nie/core/string.hpp>

#include "export_profile.hpp"

namespace nie {

namespace io {

namespace inertial_explorer {

namespace detail {

// Column names may be separated by one or more spaces, but regrettably a column name itself may also contain spaces
// (e.g. 'Project Name'). This makes it very difficult to parse correctly in all circumstances. The approach we take
// is to determine first which column names may contain a space (Either standard or custom columns). Then, we left trim
// the given line, and split on the first space that makes the head not match any known column names with space. This is
// repeat until the entire line is parsed.

// Obtain column names with spaces for standard export profiles (Parameter is only used for overload resolution)
std::unordered_set<std::string> ColumnNamesWithSpace(StandardExportProfile const&);

// TODO: [EDD] This code looks similar to StandardColumnNamesWithSpace, but it is not obvious how to make it DRY
// Obtain column names with spaces for custom export profiles (Parameter is only used for overload resolution)
// Returns a union of standard column names with space and expected column names with space
template <typename ExportProfile>
std::unordered_set<std::string> ColumnNamesWithSpace(ExportProfile const&) {
    // First obtain the standard column names with space
    auto column_names_with_space = StandardColumnNamesWithSpace();

    // Loop over all expected columns
    for (auto const& column_id : ExportProfile::expected_column_ids) {
        // Get column name
        std::string const column_name = ExportProfile::ColumnName(column_id);

        // Add the column name to the list if it contains a space
        if (column_name.find(' ') != std::string::npos) column_names_with_space.insert(column_name);
    }

    return column_names_with_space;
}

// NOTE: Column names may be separated by one or more spaces, but regrettably a column name itself may also contain
//       spaces (e.g. 'Project Name'). This makes it difficult to parse correctly in all circumstances.
//       To work around this issue, this function must be given a set of column names that contain a space and should
//       not be split if they are encountered.
std::vector<std::string> SplitAsColumnNames(
    std::string line, std::unordered_set<std::string> const& column_names_with_space);

}  // namespace detail

/**
 * @brief Reader for text files that are exported by Inertial Explorer using a specified export profile.
 *
 * Several important things should be noted about the exported files:
 *
 *   1) The files originate from the highly configurable text based exporter of Inertial Explorer
 *      Based on a chosen export profile (That can be saved for reuse):
 *         a) The file may have any filename extension
 *         b) Custom header / footer lines may either be present or not
 *         c) All columns may either have a fixed width, or be token delimited (similarly to CSV files)
 *         d) Specific columns may be present or not (There are 100+ possible output variables)
 *         e) For fixed width columns alignment is variable, and it is unknown if width is constant or data dependent
 *   2) Export profiles are not pre-defined in Inertial explorer, but defined by users for their specific use case
 *   3) Since it is a text based format, postprocessing may add / remove / change information
 *         (e.g. las filenames are not Inertial Explorer output variables, so if present they were added later)
 *
 * See the Inertial Explorer 8.70 User Manual v4, Section 1.9.3 for more info about the Export Wizard
 *
 * This class can only read files that adhere to the following rules:
 *
 *   1) The file may contain zero or more header lines, with any contents (Currently not parsed)
 *   2) The file must contain a row with all expected column names, in any order, separated by one or more spaces
 *   3) Column names may contain single consecutive spaces, but its parts may not equal any other expected column name
 *   4) Each column name may appear only once
 *   5) After the column names, a line must be present without column data (It may contain column units)
 *   6) Subsequent lines contain data rows if they adhere to the following rules:
 *      a) Data values must be separated by one or more spaces
 *      b) Data values may themselves not contain any spaces
 *      c) The number of data values on a line must be greater or equal than the number of expected column names
 *   7) No more data values are considered to be present when an EOF or a line without any values is encountered
 *   8) The file may contain zero or more footer lines, with any contents (Currently not parsed)
 *
 * Parsing of header / footer / data rows is delegated to the export profile that is specified as template parameter
 *
 * Note that export files that are token delimited (like CSV) are NOT supported by this reader
 */
template <typename ExportProfile>
class ExportReader {
public:
    using Header = typename ExportProfile::Header;
    using Row = typename ExportProfile::Row;
    using Footer = typename ExportProfile::Footer;

    /**
     * Upon construction, the reader reads / parses the headers (as specified in the export profile), then it seeks to
     * right _before_ the first row of data. As a result, the client is required to call ReadRow() and verify that
     * reading / parsing of the row (as specified in the export profile) was successful, before accessing row data.
     *
     * @param path  Path of the file to be read
     */
    explicit ExportReader(boost::filesystem::path const& path)
        : path_(path),
          line_nr_(0),
          stream_(nie::OpenFile(path, std::ios::in)),
          row_{},
          column_names_with_spaces_{detail::ColumnNamesWithSpace(ExportProfile())} {
        // Keep reading lines until we have found the column names, since this indicates where data rows will start
        // Limit the maximum number of header lines, to avoid reading entire file if we can not find the column names
        while (line_nr_ < 1024 && ReadLine()) {
            // Obtain possible column names (Take care that some column names may contain spaces)
            auto const& possible_column_names = detail::SplitAsColumnNames(line_, column_names_with_spaces_);

            // Try to determine the mapping between possible and expected column names
            // NOTE: We have found the column names if this mapping is possible
            if (DetermineColumnMapping<ExportProfile>(possible_column_names, &mapping_)) {
                // Read the line with column units
                if (!ReadLine()) {
                    // It is unexpected if we can not read the line with column units
                    LOG(FATAL) << "Unable to read column units at line " << FileLine();
                }
                // TODO:[EDD] Could try to parse line with column units

                return;
            }

            // Strip any EOL token(s)
            nie::Strip(&line_, "\r\n");

            // The line does not contain the expected column names, so we try parse it as a normal header
            try {
                // Parse and store header
                headers_.push_back(ExportProfile::ParseHeader(line_));
            } catch (std::exception const& e) {
                LOG(FATAL) << "Unable to parse header at line " << FileLine() << ": " << e.what();
            }
        }

        // If we arrive here it means that we have not been able to detect the expected column headers
        LOG(FATAL) << "Unable to detect expected column headers in file " << path;
    }

    /**
     * Read a row of data
     *
     * @return Flag that indicates if reading the row was successful
     *         If true, row data is available through @ref row()
     *         If false, there is no row data left to be read, and any footers are available through @ref footers()
     */
    bool ReadRow() {
        // Try to read the next line Read line
        if (!ReadLine()) {
            return false;
        }

        // Check if we encountered the last row (Line only contains whitespace)
        if (line_.find_first_not_of(" \t\r\n") == std::string::npos) {
            // Handle all remaining footer lines
            do {
                // Strip any EOL token(s)
                nie::Strip(&line_, "\r\n");

                // Try to parse footer
                try {
                    // Parse and store footer
                    footers_.push_back(ExportProfile::ParseFooter(line_));
                } catch (std::exception const& e) {
                    LOG(FATAL) << "Unable to parse footer at line " << FileLine() << ": " << e.what();
                }
            } while (ReadLine());

            return false;
        }

        // Extract all fields
        // TODO: [EDD] Reduction of consecutive spaces could lead to incorrect output if we need to support text
        //             fields that may contain contain spaces. (Currently, we do not support it)
        // TODO: [EDD] Write an optimized routine that combines Trim / Reduce / Split --> O(n)
        //             Profiling shows that these operations take the largest portion of run-time for large files.
        nie::Trim(&line_);
        nie::Reduce(&line_, ' ');
        auto const& fields = nie::Split<std::string>(line_, ' ');

        // Construct mapped fields
        // TODO: [EDD] Template argument can be deduced when using C++17
        auto mapped_fields = MappedFields<typename ExportProfile::ColumnId>(mapping_, fields);

        // Try to parse row
        try {
            row_ = ExportProfile::ParseRow(mapped_fields);
        } catch (std::exception const& e) {
            LOG(FATAL) << "Unable to parse row at line " << FileLine() << ": " << e.what();
        }

        return true;
    }

    Row const& row() const { return row_; }

    std::vector<Header> const& headers() const { return headers_; }
    std::vector<Footer> const& footers() const { return footers_; }

    std::string FileLine() const { return path_.string() + ":" + std::to_string(line_nr_); }

private:
    std::istream& ReadLine() {
        ++line_nr_;
        return std::getline(stream_, line_);
    }

    boost::filesystem::path const path_;
    unsigned line_nr_;
    std::string line_;

    std::fstream stream_;

    std::vector<Header> headers_;
    std::vector<Footer> footers_;

    Row row_;

    std::unordered_set<std::string> const column_names_with_spaces_;

    typename ExportProfile::ColumnMapping mapping_;
};

}  // namespace inertial_explorer

}  // namespace io

}  // namespace nie

#endif  // NIE_FORMATS_INERTIAL_EXPLORER_EXPORT_READER_HPP
