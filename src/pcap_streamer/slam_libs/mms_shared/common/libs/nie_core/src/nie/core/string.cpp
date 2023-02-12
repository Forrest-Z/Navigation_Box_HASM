/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "string.hpp"

#include <algorithm>

namespace nie {

template <>
std::vector<std::string> Split(std::string const& s, char const token) {
    std::vector<std::string> split;

    if (!s.empty()) {
        std::stringstream ss(s);
        std::string v;

        while (std::getline(ss, v, token)) {
            split.push_back(v);
        }
        if (s.back() == token) {
            split.emplace_back();
        }
    }

    return split;
}

std::string Strip(std::string const& s, char const token) {
    // First count how many elements should be removed so that the output string can be allocated at once.
    size_t count =
        static_cast<size_t>(std::count_if(s.cbegin(), s.cend(), [token](char const x) { return (x == token); }));
    std::string result(s.size() - count, '\0');
    std::copy_if(s.cbegin(), s.cend(), result.begin(), [token](char const x) { return (x != token); });
    return result;
}

std::string Strip(std::string const& s, std::string const& tokens) {
    // First count how many elements should be removed so that the output string can be allocated at once.
    size_t count = static_cast<size_t>(
        std::count_if(s.cbegin(), s.cend(), [tokens](char const x) { return (tokens.find(x) != std::string::npos); }));
    std::string result(s.size() - count, '\0');
    std::copy_if(
        s.cbegin(), s.cend(), result.begin(), [tokens](char const x) { return (tokens.find(x) == std::string::npos); });
    return result;
}

void Strip(std::string* s, char const token) {
    s->erase(std::remove_if(s->begin(), s->end(), [token](char const x) { return (x == token); }), s->end());
}

void Strip(std::string* s, std::string const& tokens) {
    s->erase(
        std::remove_if(s->begin(), s->end(), [tokens](char const x) { return (tokens.find(x) != std::string::npos); }),
        s->end());
}

std::string Reduce(std::string const& s, char const token) {
    std::string result(s.cbegin(), s.cend());
    Reduce(&result, token);
    return result;
}

std::string Reduce(std::string const& s, std::string const& tokens) {
    std::string result(s.cbegin(), s.cend());
    Reduce(&result, tokens);
    return result;
}

void Reduce(std::string* s, char const token) {
    auto predicate = [token](char const lhs, char rhs) { return (lhs == rhs) && (lhs == token); };
    s->erase(std::unique(s->begin(), s->end(), predicate), s->end());
}

void Reduce(std::string* s, std::string const& tokens) {
    auto predicate = [tokens](char const lhs, char rhs) {
        return (lhs == rhs) && (tokens.find(lhs) != std::string::npos);
    };
    s->erase(std::unique(s->begin(), s->end(), predicate), s->end());
}

std::string ToLower(std::string const& s) {
    std::string out = s;
    ToLower(&out);

    return out;
}

void ToLower(std::string* s) {
    std::transform(s->begin(), s->end(), s->begin(), [](unsigned char c) -> unsigned char { return std::tolower(c); });
}

std::string ToUpper(std::string const& s) {
    std::string out = s;
    ToUpper(&out);

    return out;
}

void ToUpper(std::string* s) {
    std::transform(s->begin(), s->end(), s->begin(), [](unsigned char c) -> unsigned char { return std::toupper(c); });
}

bool StartsWith(std::string const& s, std::string const& sub) {
    if (s.size() < sub.size()) {
        return false;
    }
    return s.substr(0, sub.size()) == sub;
}

bool EndsWith(std::string const& s, std::string const& sub) {
    if (s.size() < sub.size()) {
        return false;
    }
    return s.substr(s.size() - sub.size()) == sub;
}

unsigned stou(std::string const& str, size_t* idx, int base) {
    unsigned long result = std::stoul(str, idx, base);
    if (result > std::numeric_limits<unsigned>::max()) {
        throw std::out_of_range("stou");
    }
    return result;
}

}  // namespace nie
