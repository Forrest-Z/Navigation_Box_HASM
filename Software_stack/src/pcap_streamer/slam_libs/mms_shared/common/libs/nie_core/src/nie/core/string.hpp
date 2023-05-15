/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace nie {

/// Joining a vector to a string with the given separator token, inverse of Split()
template <typename T>
std::string Join(std::vector<T> const& v, char const token) {
    auto it = v.cbegin();
    auto end_it = v.cend();

    std::stringstream ss;
    while (true) {
        ss << *it;
        if (++it == end_it) break;
        ss << token;
    }

    return ss.str();
}

/// String splitting into required type. Does not assume any white space between elements.
template <typename T>
std::vector<T> Split(std::string const& s, char const token = ',') {
    std::vector<T> split;

    if (!s.empty()) {
        std::stringstream ss(s);
        T v;

        while (ss >> v) {
            split.push_back(v);

            if (ss.peek() == token) ss.ignore();
        }
    }

    return split;
}

template <>
std::vector<std::string> Split(std::string const& s, char const token);

/// Left trims all whitespace
inline void LeftTrim(std::string* s) {
    std::string& r = *s;
    r.erase(r.begin(), std::find_if(r.begin(), r.end(), [](int ch) { return !std::isspace(ch); }));
}

/// Left trims all whitespace
inline std::string LeftTrim(std::string const& s) {
    std::string r = s;
    LeftTrim(&r);
    return r;
}

/// Right trims all whitespace
inline void RightTrim(std::string* s) {
    std::string& r = *s;
    r.erase(std::find_if(r.rbegin(), r.rend(), [](int ch) { return !std::isspace(ch); }).base(), r.end());
}

/// Right trims all whitespace
inline std::string RightTrim(std::string const& s) {
    std::string r = s;
    RightTrim(&r);
    return r;
}

/// Left and right trims all whitespace
inline void Trim(std::string* s) {
    LeftTrim(s);
    RightTrim(s);
}

inline std::string Trim(std::string const& s) {
    std::string r = s;
    Trim(&r);
    return r;
}

/// @brief Remove all occurrences of @param{token}.
/// @param token  Token to remove.
std::string Strip(std::string const& s, char const token);
void Strip(std::string* s, char const token);

/// @brief Remove all occurrences of characters in @param{tokens}.
/// @param tokens  Tokens to remove (order of characters does not matter).
std::string Strip(std::string const& s, std::string const& tokens);
void Strip(std::string* s, std::string const& tokens);

/// @brief Find string of consecutive @param{token} and replace with one @param{token}
/// @param token  Token to reduce.
std::string Reduce(std::string const& s, char const token);
void Reduce(std::string* s, char const token);

/// @brief For each character in @param{tokens} find string of consecutive characters and replace with one.
/// @param tokens  Tokens to reduce (order of characters does not matter).
std::string Reduce(std::string const& s, std::string const& tokens);
void Reduce(std::string* s, std::string const& tokens);

/// @brief Convert all uppercase letters to their lowercase counterpart.
std::string ToLower(std::string const& s);
void ToLower(std::string* s);

/// @brief Convert all lowercase letters to their uppercase counterpart.
std::string ToUpper(std::string const& s);
void ToUpper(std::string* s);

/**
 * @brief Check whether the string starts with the given substring.
 * @param s  the string to check.
 * @param sub  the substring to find.
 */
// NOTE: This function can be replaced with std::string::starts_with() from C++20
bool StartsWith(std::string const& s, std::string const& sub);
/**
 * @brief Check whether the string ends with the given substring.
 * @param s  the string to check.
 * @param sub  the substring to find.
 */
// NOTE: This function can be replaced with std::string::ends_with() from C++20
bool EndsWith(std::string const& s, std::string const& sub);

/**
 * @brief Convert an integer based value to its hexadecimal string representation
 * @param value  value to convert
 * @param width  width of the result in characters
 * @return hexadecimal representation of the value
 *
 * Required width is padded with leading zeros. If unspecified, width will be derived from the type of the argument.
 * Usable for enum and integer types; pointers, floating-point numbers and such are not supported, they will give a
 * compiler error. Based on: https://stackoverflow.com/questions/5100718/integer-to-hex-string-in-c
 */
template <typename T>
inline std::string ToHex(T value, size_t width = sizeof(T) * 2) {
    static_assert(std::is_integral<T>::value || std::is_enum<T>::value, "Integral or enum type required");
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(width) << std::hex << value;
    return ss.str();
}

/// Annoyingly, the standard library does not provide std::stou (in symmetry with stoul, stoull, stoi, stol, and stoll),
/// see: https://en.cppreference.com/w/cpp/string/basic_string/stol
/// Implemented according to: https://stackoverflow.com/questions/8715213/why-is-there-no-stdstou
unsigned stou(std::string const& str, size_t* idx = 0, int base = 10);

}  // namespace nie
