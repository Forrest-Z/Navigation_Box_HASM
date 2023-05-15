/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "csv_row_handler_tokenizer.hpp"

#include <regex>

#include <nie/core/string.hpp>

// TODO: Add simplified code for tokenizing in absence of strings. This will avoid regex.

namespace nie {

namespace {

/// Find position of the next delimiter after a string value token.
size_t FindTokenEndString(std::string const& csv_row, size_t const& index, char const& delim) {
    // Find end of string. The patterns to look for (assuming delim == ;):
    // - ";
    // - \\";
    std::string rx;

    // match with NOT backslash or with TWO backslashes
    // followed by double quotation
    rx += "([^(\\\\)]|(\\\\\\\\))(\")([";
    // match with delimiter
    rx += delim;
    rx += "])";

    std::regex pattern{rx};

    std::smatch match;
    std::regex_search(csv_row.begin() + index, csv_row.end(), match, pattern);

    size_t position = match.position();

    size_t end_index = index + position + match.str().size();

    // A csv line does not end with a delimiter.
    // This will result in regex not being able to match and just give us the end of string as index,
    //  in which case we do not need to subtract 1 from the end_index.
    if (index + position < csv_row.size()) {
        end_index -= 1;
    }

    return end_index;
}

///
size_t FindTokenEndNumeric(std::string const& csv_row, size_t const& index, char const& delim) {
    size_t end_index = csv_row.find(delim, index);
    if (end_index == std::string::npos) {
        return csv_row.size();
    } else {
        return end_index;
    }
}

/// Find position of the next delimiter.
/// If this is the last token in the line, returns size of string.
/// \param csv_row      string representing csv data.
/// \param index        position of the first character of this token.
/// \param delim        delimiter of csv data.
size_t FindTokenEnd(std::string const& csv_row, size_t const& index, char const& delim) {
    // index must not point beyond string length
    CHECK(index < csv_row.size());

    // index must point to element beyond the previous delimiter (if not first token in string)
    if (index != 0) {
        DVLOG(9) << "Finding first token in: " << csv_row.substr(index);
        CHECK(csv_row[index - 1] == delim);
    }

    // if index points to the last character in the string, return size of string
    if (index + 1 == csv_row.size()) {
        return csv_row.size();
    }

    // Check if first char is double quotation "
    if (csv_row[index] == '\"') {
        // The token is a string value
        return FindTokenEndString(csv_row, index, delim);
    } else {
        // The token is a numerical value
        return FindTokenEndNumeric(csv_row, index, delim);
    }
}

}  // namespace

///
CsvRowHandlerTokenizerSafe::CsvRowHandlerTokenizerSafe(
        char const& delimiter, CsvTokenHandler* csv_token_handler, size_t const& number_of_tokens)
    : CsvRowHandlerTokenizer<CsvRowHandlerTokenizerSafe>{delimiter, csv_token_handler, number_of_tokens} {}

std::vector<std::string> CsvRowHandlerTokenizerSafe::Split(std::string const& csv_row, char const delim) {
    std::vector<std::string> tokens;
    size_t index = 0;

    DVLOG(6) << "-";
    DVLOG(6) << "Split row: " << csv_row;

    while (index < csv_row.size()) {
        size_t index_next_delim = FindTokenEnd(csv_row, index, delim);

        std::string token = csv_row.substr(index, index_next_delim - index);

        tokens.push_back(token);

        DVLOG(6) << "Split token (" << index << "," << index_next_delim << "): " << token;

        index = index_next_delim + 1;
    }
    return tokens;
}

///
CsvRowHandlerTokenizerFast::CsvRowHandlerTokenizerFast(
        char const& delimiter, CsvTokenHandler* csv_token_handler, size_t const& number_of_tokens)
    : CsvRowHandlerTokenizer<CsvRowHandlerTokenizerFast>{delimiter, csv_token_handler, number_of_tokens} {}

std::vector<std::string> CsvRowHandlerTokenizerFast::Split(std::string const& csv_row, char const delim) {
    DVLOG(6) << "-";
    DVLOG(6) << "Split row: " << csv_row;

    return nie::Split<std::string>(csv_row, delim);
}

}  // namespace nie