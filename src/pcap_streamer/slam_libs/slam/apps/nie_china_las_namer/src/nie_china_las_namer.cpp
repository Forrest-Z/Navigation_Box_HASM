/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <boost/filesystem.hpp>
#include <nie/core/gflags.hpp>
#include <nie/formats/ba_graph/info_ref_collection.hpp>
#include <nie/formats/inertial_explorer/export_profile.hpp>
#include <nie/formats/inertial_explorer/export_reader.hpp>

DEFINE_string(in_file_aiim, "", "The inertial explorer file.");
DEFINE_string(in_file_iref, "", "The iref file with the las files to be renamed.");
DEFINE_string(out_file_iref, "", "The output iref file with the renamed las files names.");

DEFINE_validator(in_file_aiim, nie::ValidateIsFile);
DEFINE_validator(in_file_iref, nie::ValidateIsFile);
DEFINE_validator(out_file_iref, nie::ValidateParentDirExists);

// Custom inertial explorer export profile
struct PostLatLon : public nie::io::inertial_explorer::StandardExportProfile {
    constexpr static std::array<ColumnId, 2> expected_column_ids{
            nie::io::inertial_explorer::StandardColumn::Latitude,
            nie::io::inertial_explorer::StandardColumn::Longitude};

    struct Row {
        double lat;
        double lon;
    };

    static Row ParseRow(nie::io::inertial_explorer::MappedFields<ColumnId> const& fields) {
        return {std::stod(fields[nie::io::inertial_explorer::StandardColumn::Latitude]),
                std::stod(fields[nie::io::inertial_explorer::StandardColumn::Longitude])};
    }
};

// Determine the Gauss Krueger zone
std::size_t DetermineGkZone(std::string const& path) {
    nie::io::inertial_explorer::ExportReader<PostLatLon> export_reader{path};
    if (export_reader.ReadRow()) {
        return std::llround(export_reader.row().lon / 3.);
    } else {
        LOG(FATAL) << "Could not read any record from the aiim file to determine the Gauss Krueger zone.";
    }
    return 0;
}

// Function to construct the new las filename, like 40-10021005180120-105832-00003
// where:
//     40               = gauss kruger zone
//     10021005180120   = project name
//     105832           = hhmmss in china time
//     00003            = sequence number of las in the whole data set
//
// The supplied filename (which is based on the pcap filename) is formatted like 10021005180120105832_000001 where the
// project name and time are concatenated in the front, followed by a different sequence number (which is not needed).
std::string GetNewLasFilename(std::string const& filename, std::size_t const gk_zone, std::size_t const seq_nr) {
    boost::filesystem::path const directory = boost::filesystem::path(filename).parent_path();
    std::string const extension = boost::filesystem::path(filename).extension().string();
    std::string const basename = boost::filesystem::path(filename).stem().string();

    // Reuse the project name (includes date) and time from the las filename (was based on pcap filename)
    constexpr std::size_t kProjectNameLength = 8 + 6;
    constexpr std::size_t kTimestampLength = 6;
    std::string const project_name = basename.substr(0, kProjectNameLength);
    std::string const time = basename.substr(kProjectNameLength, kTimestampLength);

    std::ostringstream ss;
    ss << gk_zone << '-' << project_name << '-' << time << '-' << std::setw(5) << std::setfill('0') << seq_nr;
    return (directory / ss.str()).string() + extension;
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    // Read main input
    LOG(INFO) << "Reading iref file from: " << FLAGS_in_file_iref;
    auto iref_collection = nie::io::ReadCollection<nie::io::InfoRefCollection>(FLAGS_in_file_iref);
    if (iref_collection.info_refs.empty()) {
        LOG(INFO) << "Nothing to do as there are no iref records is the supplied file.";
        return 0;
    } else if (!std::is_sorted(
                       iref_collection.info_refs.cbegin(),
                       iref_collection.info_refs.cend(),
                       [](auto const& a, auto const& b) { return a.path < b.path; })) {
        LOG(FATAL) << "The iref records are not sorted.";
    }

    // Read gps file and determine the Gauss Krueger zone
    LOG(INFO) << "Read gps file and extract the Gauss Krueger zone from: " << FLAGS_in_file_aiim;
    std::size_t const gk_zone = DetermineGkZone(FLAGS_in_file_aiim);

    // Update the las file names and update the references in the iref collection
    std::string prev_las_file;
    std::string new_las_file_name;
    std::size_t seq_nr = 0;
    for (auto& iref_record : iref_collection.info_refs) {
        // If the las file is not encountered yet, then determine the new name and rename the file
        if (iref_record.path != prev_las_file) {
            new_las_file_name = GetNewLasFilename(iref_record.path, gk_zone, seq_nr);

            // Rename the las file
            boost::filesystem::rename(iref_record.path, new_las_file_name);

            prev_las_file = iref_record.path;
            ++seq_nr;
        }
        // Update the iref record
        iref_record.path = new_las_file_name;
    }

    // Read outputs
    LOG(INFO) << "Writing new iref file to: " << FLAGS_out_file_iref;
    nie::io::Write(iref_collection, FLAGS_out_file_iref);

    return 0;
}
