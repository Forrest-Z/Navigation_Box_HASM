/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <regex>
#include <string>
#include <tuple>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <nie/core/filesystem.hpp>

#include "csv_handler.hpp"
#include "csv_read_predicates.hpp"

/// The functions in this header file only deal with reading strings.
/// No parsing takes place here.
/// Rows read from a csv line can be forwarded to an instance CsvHandler which provides an interface for various
///  parsing backend implementations.

namespace nie {

/// Read one csv file to a vector of strings.
/// Check predicates for each csv line.
template <class ReadLinesPredicate_>
std::vector<std::string> ReadCsv(
        std::string const& filepath, ReadLinePredicate<ReadLinesPredicate_>* read_lines_predicate) {
    std::ifstream file{filepath};

    std::vector<std::string> result;

    std::string line;

    // Iterate all lines in the file.
    while (std::getline(file, line)) {
        if ((*read_lines_predicate)(line)) {
            DVLOG(9) << line;
            result.push_back(line);
        } else {
            DVLOG(9) << "Rejected line: " << line;
        }
    }

    return result;
}

/// Callback functor for storing all csv rows in a vector of strings.
struct CsvHandlerStringAppender : CsvHandler {
    CsvHandlerStringAppender() = default;

    void operator()(
            std::vector<std::string>::const_iterator begin, std::vector<std::string>::const_iterator end) override {
        if (begin != end) {
            csv_data.insert(csv_data.end(), begin, end);
        } else {
            DVLOG(6) << "Empty csv file.";
        }
    }
    void SetHeader(std::string const& header_line) override {
        CHECK(csv_data.size() == 0);
        csv_data.push_back(header_line);
    }

    std::vector<std::string> csv_data;
};

/// Read all csv files
/// \tparam ReadLinesPredicate_     Functor type to decide if the line is valid, used to skip header line.
/// \tparam T                       Type of argument to construct @tparam{_ReadLinesPredicate}
/// \tparam Predicates_             Predicates to decide if the line is valid.
/// \param files                    Vector of filepaths to csv files.
/// \param header_l                 Argument to construct @tparam{_ReadLinesPredicate}
/// \param csv_handler              Pointer to object inheriting from CsvHandler.
/// \return                         Vector of strings, first string is header if applicable.
template <class ReadLinesPredicate_, typename T, class... Predicates_>
void ReadAllCsvToHandler(
        std::vector<boost::filesystem::path> const& files,
        T header_l,
        CsvHandler* csv_handler,
        Predicates_&&... predicates) {
    // Create predicate of all generic types in Predicates_
    PredicateList<Predicates_...> predicate_list{std::forward<Predicates_>(predicates)...};

    // For the first file we read we also need the header.
    bool also_read_header = true;

    // Iterate all files
    for (auto const& f : files) {
        // Create new predicate for each file because the header is read only once.
        ReadLinesPredicate_ read_lines_predicate2{header_l, also_read_header};

        // Create PredicateList.
        PredicateList<ReadLinesPredicate_&, PredicateList<Predicates_...>> predicate_list2{
                read_lines_predicate2, predicate_list};

        // Read current csv file.
        std::vector<std::string> tmp_result = ReadCsv(f.string(), &predicate_list2);

        // If we could not read anything from the file
        if (tmp_result.empty()) {
            // Did not read anything from file.
            LOG(INFO) << "Could not read anything from file: " << f.string();
            continue;
        }

        // Explicitly set CSV header
        if (also_read_header) {
            csv_handler->SetHeader(tmp_result[0]);
            (*csv_handler)(tmp_result.cbegin() + 1, tmp_result.cend());
        } else {
            (*csv_handler)(tmp_result.cbegin(), tmp_result.cend());
        }

        // In the rest of the files the header is skipped.
        also_read_header = false;
    }
}

/// Read all csv files to a vector of strings.
/// \tparam ReadLinesPredicate_     Functor type to decide if the line is valid, used to skip header line.
/// \tparam T                       Type of argument to construct @tparam{_ReadLinesPredicate}
/// \tparam Predicates_             Predicates to decide if the line is valid.
/// \param files                    Vector of filepaths to csv files.
/// \param header_l                 Argument to construct @tparam{_ReadLinesPredicate}
/// \return                         Vector of strings, first string is header if applicable.
template <class ReadLinesPredicate_, typename T, class... Predicates_>
std::vector<std::string> ReadAllCsv(
        std::vector<boost::filesystem::path> const& files, T header_l, Predicates_&&... predicates) {
    std::vector<std::string> result;

    CsvHandlerStringAppender csv_handler_string_appender;

    ReadAllCsvToHandler<ReadLinesPredicate_>(
            files, header_l, &csv_handler_string_appender, std::forward<Predicates_>(predicates)...);

    return csv_handler_string_appender.csv_data;
}

}  // namespace nie
