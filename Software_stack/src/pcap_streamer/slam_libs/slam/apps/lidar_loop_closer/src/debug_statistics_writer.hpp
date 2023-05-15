/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <tuple>

#include <nie/core/algorithm.hpp>

namespace nie {

class DebugStatisticsWriter {
private:
    using DataType =
        std::tuple<int, int, int, int, int, int, float, float, float, float, int, int, float, float, float>;

public:
    enum Property : size_t {
        kIdA,
        kIdB,
        kPointCountA,
        kPointCountB,
        kPointCountFilteredA,
        kPointCountFilteredB,
        kAreaIntersection,
        kAreaIntersectionRatio,
        kLengthTraceA,
        kLengthTraceB,
        kPointDensity,
        kCorrespondences,
        kFitnessScore,
        kTranslationSize,
        kRotationSize
    };

    DebugStatisticsWriter(nie::mt::MtStream<std::ofstream>* ofs, std::string sep = ",")
        : ofs_(ofs), separator_(sep), data_(-1, -1, -1, -1, -1, -1, -1.f, -1.f, -1.f, -1.f, -1, -1, -1.f, -1.f, -1.f) {}
    ~DebugStatisticsWriter() { Write(); }

    template <size_t index, typename T>
    void Set(T const& value) {
        std::get<index>(data_) = value;
    }

private:
    void Write() {
        // Output the properties, first create the full string and then add it to stream in one go, otherwise the string
        // could still be split by multi threading
        std::stringstream ss;
        nie::ForEach(data_, [sep = separator_, &ss](auto const& v) { ss << v << sep; });
        std::string line = ss.str();
        // Not so nice: replace the last separator with a new line
        line.erase(line.end() - separator_.size(), line.end());
        line.push_back('\n');
        *ofs_ << line;
    }

    nie::mt::MtStream<std::ofstream>* ofs_;
    std::string const separator_;
    DataType data_;
};

}  // namespace nie
