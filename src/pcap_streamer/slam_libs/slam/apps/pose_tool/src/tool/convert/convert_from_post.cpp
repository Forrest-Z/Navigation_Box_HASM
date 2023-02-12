/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "convert_from_post.hpp"

#include <nie/core/geometry/covariance.hpp>
#include <nie/core/time.hpp>
#include <nie/formats/inertial_explorer/export_reader.hpp>
#include <nie/formats/inertial_explorer/isometry.hpp>
#include <nie/formats/inertial_explorer/project_name.hpp>
#include <nie/formats/inertial_explorer/timestamp.hpp>

#include "tool/io.hpp"

DEFINE_bool(
        classic_post,
        false,
        "Set this flag to only convert the following PosT columns: SequenceNumber, GPSTime, Northing, Easting, "
        "HeightEllipsoid, Roll, Pitch, Heading. WARNING: classic post interprets GPSTime as time in the day (not "
        "week).");
DEFINE_double(
        stationary_threshold,
        0.1,
        "If the vehicle speed (m/s) is lower than this threshold, it is considered to be stationary."
        "Default is 0.1, which is the value used in inertial explorer.");

namespace {

/// Convert with covariances
void Convert(
        nie::io::inertial_explorer::AiimFile::Row const& row,
        Eigen::Matrix<double, 6, 6>* information,
        nie::Isometry3qd* isometry) {
    Eigen::Matrix<double, 6, 6> covariance;
    nie::io::inertial_explorer::CovIsometryAircraftFromPosT(row, &covariance, isometry);
    nie::CovarianceToInformation(covariance, information);
}

/// Convert only poses
void Convert(
        nie::io::inertial_explorer::PosTClassic::Row const& row,
        Eigen::Matrix<double, 6, 6>*,
        nie::Isometry3qd* isometry) {
    *isometry = nie::io::inertial_explorer::IsometryAircraftFromPosT(row);
}

void UpdateTimeStamps(
        std::vector<nie::io::inertial_explorer::AiimFile::Row> const& rows,
        std::vector<nie::io::PoseRecord>* p_records) {
    std::vector<nie::io::PoseRecord>& records = *p_records;

    for (size_t i = 0; i < rows.size(); ++i) {
        records[i].timestamp = nie::io::inertial_explorer::TimestampFromPosT(rows[i]);
    }
}

void UpdateTimeStamps(
        std::vector<nie::io::inertial_explorer::PosTClassic::Row> const& rows,
        std::vector<nie::io::PoseRecord>* p_records) {
    std::vector<nie::io::PoseRecord>& records = *p_records;

    // Extract project name from PosT path
    std::string const project_name = boost::filesystem::path(FLAGS_in_paths).stem().string();

    // Extract UTC date from project name in filename
    auto const utc_zoned_date = nie::io::inertial_explorer::ParseDateFromProjectName(project_name);

    for (size_t i = 0; i < rows.size(); ++i) {
        records[i].timestamp = nie::io::inertial_explorer::TimestampFromPosT(rows[i], utc_zoned_date);
    }
}

template <typename Profile>
std::vector<typename Profile::Row> ExtractPostRows() {
    using Row = typename Profile::Row;

    InPathExistsOrFatal();
    CheckOutPathsLocationsOrFatal();

    LOG(INFO) << "Reading PosT file: " << FLAGS_in_paths;

    DVLOG(3) << "Creating export_reader...";
    nie::io::inertial_explorer::ExportReader<Profile> export_reader{FLAGS_in_paths};
    DVLOG(3) << "Created export_reader.";

    // Read all rows.
    std::vector<Row> post_rows;
    size_t count = 0;
    while (export_reader.ReadRow()) {
        DVLOG(9) << "Reading row " << count;
        Row row = export_reader.row();
        post_rows.push_back(row);

        ++count;
    }
    LOG(INFO) << "Read " << count << " rows from PosT file.";

    return post_rows;
}

template <typename Row>
void GenerateStationaryIntervalsFile(std::vector<Row> const& rows, std::vector<nie::io::PoseRecord> const& records) {
    CHECK(rows.size() == records.size());

    boost::filesystem::path out_file_path;
    GetAndCheckOutPathsForExtensionOrFatal(".stop", &out_file_path);

    std::vector<std::pair<double, double>> time_intervals;
    bool in_sequence = false;
    double sequence_start = 0.0;

    for (std::size_t index = 0; index < rows.size(); ++index) {
        auto const& row = rows[index];
        auto const& record = records[index];

        double const time_in_week = std::chrono::duration_cast<std::chrono::duration<double>>(
                                            nie::ToGPSWeekTime(record.timestamp).time_in_week)
                                            .count();
        if (row.horizontal_speed < FLAGS_stationary_threshold) {
            if (!in_sequence) {
                sequence_start = time_in_week;
                in_sequence = true;
            }
        } else if (in_sequence) {
            in_sequence = false;
            time_intervals.emplace_back(sequence_start, time_in_week);  // Left-bound interval
        }
    }

    LOG(INFO) << "Writing stationary intervals file: " << out_file_path;
    std::ofstream out_file(out_file_path.string());

    out_file << nie::ToGPSWeekTime(records.front().timestamp).week.count() << std::endl;
    out_file << "begin,end" << std::endl;
    for (const auto& [t1, t2] : time_intervals) {
        out_file << std::fixed << std::setprecision(4) << t1 << "," << t2 << std::endl;
    }
    out_file.close();
}

template <typename Row>
void ConvertFromPostProfile(std::vector<Row> const& post_rows) {
    nie::io::PoseCollection pc{};
    if (!FLAGS_classic_post) {
        pc.header.flags = nie::io::PoseHeader::kHasPoseInformationPerRecord;
    }
    pc.header.flags |= nie::io::PoseHeader::kHasTimestampPerRecord;

    // Currently we cannot obtain an EPSG code from a PosT file
    nie::io::SetNieAuthority(&pc.header);

    // Convert rows to PoseCollection
    pc.poses.resize(post_rows.size());
    for (size_t i = 0; i < post_rows.size(); ++i) {
        Row row = post_rows[i];
        pc.poses[i].id = static_cast<nie::io::PoseId>(row.seq_num);
        Convert(row, &pc.poses[i].information, &pc.poses[i].isometry);
    }

    UpdateTimeStamps(post_rows, &pc.poses);

    WriteData(pc);

    // Extract and output the stationary intervals
    GenerateStationaryIntervalsFile(post_rows, pc.poses);
}

}  // namespace

void ConvertFromPost() {
    if (FLAGS_classic_post) {
        ConvertFromPostProfile(ExtractPostRows<nie::io::inertial_explorer::PosTClassic>());
    } else {
        ConvertFromPostProfile(ExtractPostRows<nie::io::inertial_explorer::AiimFile>());
    }
}
