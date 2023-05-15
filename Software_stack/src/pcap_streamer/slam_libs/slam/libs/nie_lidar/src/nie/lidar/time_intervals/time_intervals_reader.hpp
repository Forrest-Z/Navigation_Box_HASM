/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <algorithm>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <glog/logging.h>

#include <nie/core/time.hpp>
#include <nie/formats/csv/csv_read.hpp>
#include <nie/formats/csv/csv_record.hpp>

namespace nie {
namespace io {

nie::Timestamp_ns ConvertGPSWeekSecondsToTimeStamps(double const time_in_week, int weeks) {
    GPSWeekTime<std::chrono::nanoseconds> gps_week_time{
            std::chrono::weeks(weeks), nie::RepresentDoubleAsDuration<std::chrono::nanoseconds>(time_in_week)};
    return ToGPSTime(gps_week_time);
}

namespace {

void ParseWeekNumberLine(std::ifstream* in_file, size_t* week_number) {
    std::string first_line;
    CHECK(std::getline(*in_file, first_line)) << "Could not read the week number line in the intervals file.";
    std::stringstream ss(first_line);
    CHECK(!(ss >> *week_number).fail()) << "Invalid week number in the intervals file.";
}

void ValidateHeaderLine(std::ifstream* in_file, std::string const& expected_header) {
    std::string header;
    CHECK(std::getline(*in_file, header)) << "Could not read the header line in the intervals file.";
    VLOG(5) << "Read header: " << header;
    CHECK(header == expected_header) << "Invalid header in the intervals file.";
}

}  // namespace

auto ReadIntervalsList(std::string const& filepath) {
    using TimeRange = std::pair<nie::Timestamp_ns, nie::Timestamp_ns>;
    std::vector<TimeRange> intervals;

    std::ifstream in_file(filepath);
    CHECK(in_file.good()) << "Could not open file \"" << filepath << "\"";

    size_t week_number = 0;
    ParseWeekNumberLine(&in_file, &week_number);

    std::string const header{"begin,end"};
    ValidateHeaderLine(&in_file, header);

    nie::CsvRecorderFast<double, double> recorder{','};
    nie::ReadAllCsvToHandler<nie::ReadLinesAfterHeaderPredicate>({filepath}, header, &recorder);
    auto const records = recorder.ConvertToRecord();
    intervals.reserve(records.NumRows());
    for (std::size_t i = 0; i < records.NumRows(); ++i) {
        intervals.emplace_back(
                ConvertGPSWeekSecondsToTimeStamps(std::get<0>(records.GetRow(i)), week_number),
                ConvertGPSWeekSecondsToTimeStamps(std::get<1>(records.GetRow(i)), week_number));
    }

    LOG(INFO) << "Read " << intervals.size() << " intervals from file \"" << filepath << "\"";
    return intervals;
}

}  // namespace io
}  // namespace nie
