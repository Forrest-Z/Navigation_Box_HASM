/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <string>
#include <tuple>
#include <vector>

#include "csv_token_handler.hpp"
#include "csv_tuple_handler.hpp"

// TODO: Create multithreaded parser.

namespace nie {

/// Convert string to @tparam{T}
/// Note that type bool cannot be supported as std::vector<bool> returns bool value copy instead of a const&.
template <typename T>
struct StringToValue;
template <>
struct StringToValue<void*> {
    static void* Convert(std::string const& s) {
        DVLOG(9) << "Skip token: " << s;
        return nullptr;
    }
};
template <>
struct StringToValue<double> {
    static double Convert(std::string const& s) {
        DVLOG(9) << "Parsing double: " << s;
        return std::stod(s);
    }
};
template <>
struct StringToValue<long> {
    static long Convert(std::string const& s) {
        DVLOG(9) << "Parsing long: " << s;
        return std::stol(s);
    }
};
template <>
struct StringToValue<float> {
    static float Convert(std::string const& s) {
        DVLOG(9) << "Parsing float: " << s;
        return std::stof(s);
    }
};
template <>
struct StringToValue<int> {
    static int Convert(std::string const& s) {
        DVLOG(9) << "Parsing "
                 << "int: " << s;
        return std::stoi(s);
    }
};
template <>
struct StringToValue<std::size_t> {
    static std::size_t Convert(std::string const& s) {
        DVLOG(9) << "Parsing std::size_t: " << s;
        return std::stoull(s);
    }
};
/// This method will strip first and last character because they are always double quotes (").
template <>
struct StringToValue<std::string> {
    static std::string Convert(std::string const& s) {
        DVLOG(9) << "Parsing string: " << s;
        if (s[0] == '\"' && *(s.crbegin()) == '\"') {
            // Our format requires that string values are enclosed with double quotation signs.
            return s.substr(1, s.size() - 2);  // strip first and last char
        } else {
            // If there was no double quote the token is invalid but return it anyway.
            DVLOG(6) << "Parsing string token without double quotes: " << s;
            return s;
        }
    }
};

template <size_t Index = 0, typename... Ts>
void ParseTokens(std::vector<std::string> const& tokens [[maybe_unused]], std::tuple<Ts...>* csv_tup [[maybe_unused]]) {
    if constexpr (Index == sizeof...(Ts)) {
        return;
    } else {
        using T = std::tuple_element_t<Index, std::tuple<Ts...>>;

        std::get<Index>(*csv_tup) = tokens[Index].empty() ? T{} : StringToValue<T>::Convert(tokens[Index]);

        ParseTokens<Index + 1>(tokens, csv_tup);
    }
}

template <typename... Ts>
struct CsvTokenHandlerParser : CsvTokenHandler {
    explicit CsvTokenHandlerParser(CsvTupleHandler<Ts...>* csv_tuple_handler) : csv_tuple_handler{csv_tuple_handler} {}

    void operator()(std::vector<std::string> const& tokens) override {
        CHECK(tokens.size() == kNumberOfTokens)
                << "Expected " << kNumberOfTokens << " tokens but got " << tokens.size() << ".";

        DVLOG(6) << "Parsing " << tokens.size() << " tokens.";
        std::tuple<Ts...> csv_tup;

        ParseTokens(tokens, &csv_tup);

        (*csv_tuple_handler)(std::move(csv_tup));
    }

    CsvTupleHandler<Ts...>* csv_tuple_handler;

    static constexpr size_t kNumberOfTokens = sizeof...(Ts);
};

}  // namespace nie
