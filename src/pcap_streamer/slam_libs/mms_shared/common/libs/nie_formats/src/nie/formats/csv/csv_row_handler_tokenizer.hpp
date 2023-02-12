/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <string>
#include <vector>

#include <glog/logging.h>

#include "csv_row_handler.hpp"
#include "csv_token_handler.hpp"

namespace nie {

// TODO: Pass expected number of columns to tokenizer and create array<std::string> in stead of vector.
// TODO: Create multithreaded tokenizer.

template <class Derived>
struct CsvRowHandlerTokenizer : CsvRowHandler {
    CsvRowHandlerTokenizer(char const& delimiter, CsvTokenHandler* csv_token_handler, size_t const& number_of_tokens)
        : delim{delimiter}, csv_token_handler{csv_token_handler}, number_of_tokens{number_of_tokens} {}

    void operator()(std::string const& csv_row) override {
        // Split string
        std::vector<std::string> tokens = Derived::Split(csv_row, delim);

        if (tokens.size() != number_of_tokens) {
            constexpr char kUnexpectedNumberOfTokens[] = "Found an unexpected number of tokens.";
            LOG(ERROR) << kUnexpectedNumberOfTokens << " Expected " << number_of_tokens << " but got " << tokens.size()
                       << ".";
            throw std::runtime_error(kUnexpectedNumberOfTokens);
        }

        // Callback
        (*csv_token_handler)(tokens);
    }

    char const delim;
    CsvTokenHandler* csv_token_handler;
    size_t const number_of_tokens;
};

/// Split single csv row to vector of string tokens.
/// String values are allowed to contain special characters (including csv delimiter) if enclosed with double quotes.
struct CsvRowHandlerTokenizerSafe : CsvRowHandlerTokenizer<CsvRowHandlerTokenizerSafe> {
    CsvRowHandlerTokenizerSafe(
            char const& delimiter, CsvTokenHandler* csv_token_handler, size_t const& number_of_tokens);

    /// Note: string valued tokens are still enclosed by double quotation signs!
    static std::vector<std::string> Split(std::string const& csv_row, char delim);
};

/// Split single csv row to vector of string tokens.
/// String values cannot contain delimiter characters.
struct CsvRowHandlerTokenizerFast : CsvRowHandlerTokenizer<CsvRowHandlerTokenizerFast> {
    CsvRowHandlerTokenizerFast(
            char const& delimiter, CsvTokenHandler* csv_token_handler, size_t const& number_of_tokens);

    /// Note: string valued tokens are still enclosed by double quotation signs!
    static std::vector<std::string> Split(std::string const& csv_row, char delim);
};

}  // namespace nie
