/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <fstream>

#include <nie/core/string.hpp>

#include "csv_record.hpp"

namespace nie {

/// Wrapper of std::basic_ostream for std::tuple and CsvRecord
template <typename CharT, typename Traits = std::char_traits<CharT>>
struct CsvOutStream {
    explicit CsvOutStream(std::basic_ostream<CharT, Traits>& out_stream, char delim = ';', int float_precision = 6)
        : delim{delim}, float_precision{float_precision}, out_stream_{out_stream} {}

    char delim;
    int float_precision;

private:
    std::basic_ostream<CharT, Traits>& out_stream_;

    // Wrappers for converting different datatypes to string.
    template <typename T>
    inline void WriteValue(T const& value) {
        out_stream_ << value;
    }
    inline void WriteValue(std::string const& value) { out_stream_ << "\"" << value << "\""; }
    inline void WriteValue(void* const&) { /* Nothing to write */
    }
    inline void WriteValue(double const& value) { out_stream_ << std::setprecision(float_precision) << value; }
    inline void WriteValue(float const& value) { out_stream_ << std::setprecision(float_precision) << value; }

    // Serialization logic for std::tuple
    template <size_t Index = 0, typename... Ts>
    inline void SerializeTuple(std::tuple<Ts...> const& tup) {
        if constexpr (sizeof...(Ts) == 0u) {
            return;
        } else {
            WriteValue(std::get<Index>(tup));
            if constexpr (Index + 1 < sizeof...(Ts)) {
                out_stream_ << delim;
                SerializeTuple<Index + 1>(tup);
            }
        }
    }

    // Stream operator for tuples
    template <typename... Ts, typename CharT_, typename Traits_>
    friend inline CsvOutStream<CharT_, Traits_>& operator<<(
            CsvOutStream<CharT_, Traits_>& csv_out_stream, std::tuple<Ts...> const& tup);

    // Stream operator for CsvRecord
    template <typename... Ts, typename CharT_, typename Traits_>
    friend inline CsvOutStream<CharT_, Traits_>& operator<<(
            CsvOutStream<CharT_, Traits_>& csv_out_stream, CsvRecord<Ts...> const& csv_record);
};

template <typename... Ts, typename CharT, typename Traits>
inline CsvOutStream<CharT, Traits>& operator<<(
        CsvOutStream<CharT, Traits>& csv_out_stream, std::tuple<Ts...> const& tup) {
    csv_out_stream.SerializeTuple(tup);
    return csv_out_stream;
}
template <typename... Ts, typename CharT, typename Traits>
inline CsvOutStream<CharT, Traits>& operator<<(
        CsvOutStream<CharT, Traits>& csv_out_stream, CsvRecord<Ts...> const& csv_record) {
    csv_out_stream.out_stream_ << nie::Join(csv_record.header_tokens(), csv_out_stream.delim) << "\n";

    for (size_t i = 0; i < csv_record.NumRows(); ++i) {
        csv_out_stream << csv_record.GetRow(i);
        csv_out_stream.out_stream_ << '\n';
    }

    return csv_out_stream;
}

template <typename... Ts>
void WriteCsvRecord(std::string const& filepath, CsvRecord<Ts...> const& csv_record, int float_precision = 6) {
    std::ofstream outfile{filepath, std::ios::out};
    nie::CsvOutStream<char> csv_out_stream{outfile, csv_record.delim(), float_precision};
    csv_out_stream << csv_record;
}

}  // namespace nie
