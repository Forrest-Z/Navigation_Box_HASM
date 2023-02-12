/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <boost/filesystem/path.hpp>
#include <nie/formats/inertial_explorer/export_profile_defs.hpp>
#include <nie/formats/inertial_explorer/export_reader.hpp>

namespace nie {
namespace io {
namespace inertial_explorer {

/// Old PosTDefault
struct PosTTestProfile : public StandardExportProfile {
    constexpr static std::array<ColumnId, 13> expected_column_ids{
        StandardColumn::SequenceNumber,
        StandardColumn::GPSTime,
        StandardColumn::Northing,
        StandardColumn::Easting,
        StandardColumn::HeightEllipsoid,
        StandardColumn::Latitude,
        StandardColumn::Longitude,
        StandardColumn::HorizontalSpeed,
        StandardColumn::Roll,
        StandardColumn::Pitch,
        StandardColumn::Heading,
        StandardColumn::ProjectName,
        StandardColumn::Quality,
    };

    struct Row {
        std::size_t seq_num;
        double time_in_day;
        double northing;
        double easting;
        double height;
        double latitude;
        double longitude;
        double horizontal_speed;
        double roll;
        double pitch;
        double heading;
        int quality;
    };

    static Row ParseRow(MappedFields<ColumnId> const& fields) {
        // clang-format off
        return {
            std::stoull(fields[StandardColumn::SequenceNumber]),
            std::stod  (fields[StandardColumn::GPSTime]),
            std::stod  (fields[StandardColumn::Northing]),
            std::stod  (fields[StandardColumn::Easting]),
            std::stod  (fields[StandardColumn::HeightEllipsoid]),
            std::stod  (fields[StandardColumn::Latitude]),
            std::stod  (fields[StandardColumn::Longitude]),
            std::stod  (fields[StandardColumn::HorizontalSpeed]),
            std::stod  (fields[StandardColumn::Roll]),
            std::stod  (fields[StandardColumn::Pitch]),
            std::stod  (fields[StandardColumn::Heading]),
            std::stoi  (fields[StandardColumn::Quality])
        };
        // clang-format on
    }
};

constexpr std::array<PosTTestProfile::ColumnId, 13> PosTTestProfile::expected_column_ids;

}  // namespace inertial_explorer
}  // namespace io
}  // namespace nie

using ExportProfile = nie::io::inertial_explorer::PosTTestProfile;

struct ExportReaderTest : public ::testing::Test {
    ExportReaderTest()
        : post_path{"/data/aiim/unit_tests_data/formats/1001-0003-190403-03-base-veryshort.post"},
          export_reader{post_path},
          rows_in_file{16} {
        // clang-format off
        row_00 = { 0, 14529.000, 3370056.5865, 545320.4611, 18.005, 30.44966898897, 114.47184804020, 0.0029, 1.12412, -5.29674, 175.51456, 3};
        row_08 = { 8, 14529.080, 3370056.5865, 545320.4611, 18.005, 30.44966898897, 114.47184804020, 0.0027, 1.12236, -5.29782, 175.51422, 2};
        row_15 = {15, 14529.150, 3370056.5865, 545320.4611, 18.005, 30.44966898897, 114.47184804020, 0.0016, 1.12405, -5.29542, 175.51562, 1};

        headers = {"Project:     1001-0003-190403-03-base",
                   "Program:     Inertial Explorer Version 8.60.5025",
                   "Profile:     HUACE_Pos",
                   "Source:      GNSS/INS Epochs(Smoothed TC Combined)",
                   "",
                   "Datum:       WGS84, (processing datum)",
                   "Master 1:    Name 1054189, Status ENABLED",
                   "             Antenna height 0.119 m, to L1PC [CHCI80(NONE)]",
                   "             Position 30 27 23.39873, 114 23 49.23157, 77.631 m (WGS84, Ellipsoidal hgt)",
                   "",
                   "Map projection Info:",
                   "  Defined grid: Gauss Kruger (3 deg), Zone 38",
                   ""};

        footers = {"",
                   "Footer lines",
                   "   - Just for testing",
                   "   - No real information"};
        // clang-format on
    }

    boost::filesystem::path const post_path;
    std::string const project_name;

    nie::io::inertial_explorer::ExportReader<ExportProfile> export_reader;

    unsigned const rows_in_file;

    ExportProfile::Row row_00;
    ExportProfile::Row row_08;
    ExportProfile::Row row_15;

    std::vector<std::string> headers;
    std::vector<std::string> footers;
};

void CompareRows(ExportProfile::Row const& lhs, ExportProfile::Row const& rhs) {
    // clang-format off
    ASSERT_EQ      (lhs.seq_num,          rhs.seq_num);
    ASSERT_FLOAT_EQ(lhs.time_in_day,      rhs.time_in_day);
    ASSERT_FLOAT_EQ(lhs.northing,         rhs.northing);
    ASSERT_FLOAT_EQ(lhs.easting,          rhs.easting);
    ASSERT_FLOAT_EQ(lhs.height,           rhs.height);
    ASSERT_FLOAT_EQ(lhs.latitude,         rhs.latitude);
    ASSERT_FLOAT_EQ(lhs.longitude,        rhs.longitude);
    ASSERT_FLOAT_EQ(lhs.horizontal_speed, rhs.horizontal_speed);
    ASSERT_FLOAT_EQ(lhs.roll,             rhs.roll);
    ASSERT_FLOAT_EQ(lhs.pitch,            rhs.pitch);
    ASSERT_FLOAT_EQ(lhs.heading,          rhs.heading);
    ASSERT_EQ      (lhs.quality,          rhs.quality);
    // clang-format on
}

TEST(ExportReaderCreationalTest, Lifecycle) {
    using Reader = nie::io::inertial_explorer::ExportReader<ExportProfile>;

    constexpr char const* path = "/data/aiim/unit_tests_data/formats/1001-0003-190403-03-base-veryshort.post";

    Reader* reader = nullptr;
    ASSERT_NO_THROW(reader = new Reader(path));
    ASSERT_NO_THROW(delete reader);
    ASSERT_NO_FATAL_FAILURE(reader = new Reader(path));
    ASSERT_NO_FATAL_FAILURE(delete reader);

    // Should throw upon attempting to read a forbidden or otherwise unavailable path
    ASSERT_DEATH(reader = new Reader("/root/forbidden.post"), "");
}

TEST_F(ExportReaderTest, Header) { ASSERT_EQ(export_reader.headers(), headers); }

TEST_F(ExportReaderTest, Footer) {
    // Read and discard all rows; any footers are available when all rows are read
    while (export_reader.ReadRow())
        ;

    ASSERT_EQ(export_reader.footers(), footers);
}

TEST_F(ExportReaderTest, GetFirstRow) {
    ExportProfile::Row row;

    ASSERT_TRUE(export_reader.ReadRow());
    ASSERT_NO_THROW(row = export_reader.row());
    CompareRows(row_00, row);
}

TEST_F(ExportReaderTest, GetAllRows) {
    std::vector<ExportProfile::Row> rows;

    while (export_reader.ReadRow()) rows.push_back(export_reader.row());

    ASSERT_EQ(rows.size(), rows_in_file);

    CompareRows(row_00, rows[0]);
    CompareRows(row_08, rows[8]);
    CompareRows(row_15, rows[15]);
}
