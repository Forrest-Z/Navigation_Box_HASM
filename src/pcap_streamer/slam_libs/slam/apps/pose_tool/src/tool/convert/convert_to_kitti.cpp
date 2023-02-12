/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "convert_to_kitti.hpp"

#include <iostream>
#include <unordered_set>

#include <nie/formats/ba_graph/pose_collection.hpp>

#include "tool/io.hpp"

namespace detail {

std::string writeIsometry(nie::Isometry3qd const& isometry) {
    // The kitti pose format stores the 3x4 transformation [R|t] in a row major format, so
    //   [ 1  2  3 |  4 ]
    //   [ 5  6  7 |  8 ]  ->  1 2 3 4 5 6 7 8 9 10 11 12
    //   [ 9 10 11 | 12 ]
    // moreover, Eigen matrices are stored column major by default.

    std::stringstream ss;
    Eigen::Transform<double, 3, Eigen::Isometry, Eigen::RowMajor> const& T = isometry.ToTransform();
    ss << T.data()[0];
    for (std::size_t i = 1; i < 12; ++i) {
        ss << " " << std::fixed << std::setprecision(8) << T.data()[i];
    }
    return ss.str();
}

}  // namespace detail

void ConvertToKitti() {
    std::string const& pose_ext = nie::io::graph::Extension<nie::io::PoseCollection>();

    InPathExistsOrFatal(pose_ext);
    CheckOutPathsLocationsOrFatal();

    boost::filesystem::path path;
    GetAndCheckInPathsForExtensionOrFatal(pose_ext, &path);
    std::string const in_file_name = path.string();
    GetAndCheckOutPathsForExtensionOrFatal(".txt", &path);
    std::string const out_file_name = path.string();

    // Initialize output
    std::ofstream output(out_file_name, std::ios::out);

    LOG(INFO) << "Reading " << pose_ext << " file '" << in_file_name << "'.";
    nie::io::PoseCollectionStreamReader reader(in_file_name);

    // Read and write pose records
    std::function<void(nie::io::PoseRecord)> add_pose_func = [&output](nie::io::PoseRecord const& p) {
        output << detail::writeIsometry(p.isometry) << '\n';
    };
    reader.SetCallback(add_pose_func);
    reader.ReadRecords();

    // Finalize output
    output.close();
    LOG(INFO) << "Kitti pose file '" << out_file_name << "' written.";
}
