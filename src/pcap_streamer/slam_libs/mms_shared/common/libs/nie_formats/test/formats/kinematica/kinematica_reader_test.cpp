/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <string>

#include <boost/filesystem.hpp>
#include <nie/formats/kinematica/kinematica_reader.hpp>

struct KinematicaReaderTest : public ::testing::Test {
    KinematicaReaderTest()
        : test_path{"/data/aiim/unit_tests_data/formats/kinematica/"},
          short_csv_path{test_path / "kinematica_short.csv"} {}

    boost::filesystem::path const test_path;
    boost::filesystem::path const short_csv_path;
    size_t const num_rows = 3;
};

TEST_F(KinematicaReaderTest, ReadKinematicaCsv) {
    nie::io::KinematicaRecordCollection const kinematica_csv{nie::io::ReadKinematicaCsv(short_csv_path.string())};
    ASSERT_EQ(kinematica_csv.size(), num_rows);
}
