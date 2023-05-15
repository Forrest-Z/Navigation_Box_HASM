/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <glog/logging.h>

#include <nie/core/string.hpp>

#include "csv_columns.hpp"
#include "csv_handler.hpp"
#include "csv_row_handler_tokenizer.hpp"
#include "csv_token_handler_parser.hpp"

namespace nie {

/// Extends CsvColumns by adding header information.
/// \tparam Ts          pack of types, one type for each csv column.
template <typename... Ts>
struct CsvRecord : CsvColumns<Ts...> {
public:
    CsvRecord() = default;

    /// Create CsvRecord from CsvColumns.
    CsvRecord(CsvColumns<Ts...>&& csv_columns, char const& delim, std::string const& header)
        : CsvColumns<Ts...>{std::forward<CsvColumns<Ts...>>(csv_columns)},
          delim_{delim},
          header_{header},
          header_tokens_{nie::Split<std::string>(header, delim)} {}

    /// Create CsvRecord from text data.
    CsvRecord(
            std::vector<std::string>::const_iterator it_begin,
            std::vector<std::string>::const_iterator it_end,
            char const& delim,
            std::string const& header)
        : CsvColumns<Ts...>{it_begin, it_end, delim},
          delim_{delim},
          header_{header},
          header_tokens_{nie::Split<std::string>(header, delim)} {}

    /// Construct CsvColumns from vectors.
    explicit CsvRecord(std::vector<Ts> const&... vectors, char const& delim, std::string const& header)
        : CsvColumns<Ts...>{vectors...},
          delim_{delim},
          header_{header},
          header_tokens_{nie::Split<std::string>(header, delim)} {}

    /// Construct CsvColumns from vectors.
    explicit CsvRecord(std::vector<Ts>&&... vectors, char const& delim, std::string const& header)
        : CsvColumns<Ts...>{std::move(vectors)...},
          delim_{delim},
          header_{header},
          header_tokens_{nie::Split<std::string>(header, delim)} {}

    /// Return header name of the @param{index}-th column.
    std::string const& GetColName(size_t const& index) const { return header_tokens_.at(index); }

    /// Return header row.
    std::string const& header() const { return header_; }

    /// Return vector of header names.
    std::vector<std::string> const& header_tokens() const { return header_tokens_; }

    /// Return delim
    char const& delim() const { return delim_; }

    // TODO: serialization method.
    // TODO: method for re-ording columns. See same todo in csv_columns.hpp.

private:
    char delim_;
    std::string header_;
    std::vector<std::string> header_tokens_;
};

/// Creates CsvRecord. Pass an instance of this type to ReadAllCsvToHandler()
template <typename Tokenizer, typename... Ts>
struct CsvRecorder : CsvHandler {
    explicit CsvRecorder(char const& delim)
        : csv_columns_{},
          csv_token_handler_parser_{&csv_columns_},
          csv_row_handler_tokenizer_{delim, &csv_token_handler_parser_, sizeof...(Ts)},
          delim_{delim} {}

    /// Handle vector of csv rows.
    void operator()(
            std::vector<std::string>::const_iterator begin, std::vector<std::string>::const_iterator end) override {
        // Pass each row to the pipeline.
        for (auto it = begin; it != end; ++it) {
            csv_row_handler_tokenizer_(*it);
        }
    }

    /// Handle header row.
    void SetHeader(std::string const& header_line) override { header_ = header_line; }

    /// Return CsvRecord object.
    /// Destroys the data contained inside the recorder because csv_columns_ is moved.
    CsvRecord<Ts...> ConvertToRecord() { return CsvRecord<Ts...>{std::move(csv_columns_), delim_, header_}; }

private:
    CsvColumns<Ts...> csv_columns_;
    CsvTokenHandlerParser<Ts...> csv_token_handler_parser_;
    CsvRowHandlerTokenizer<Tokenizer> csv_row_handler_tokenizer_;
    char const delim_;
    std::string header_;
};

template <typename... Ts>
using CsvRecorderSafe = CsvRecorder<CsvRowHandlerTokenizerSafe, Ts...>;

template <typename... Ts>
using CsvRecorderFast = CsvRecorder<CsvRowHandlerTokenizerFast, Ts...>;

}  // namespace nie
