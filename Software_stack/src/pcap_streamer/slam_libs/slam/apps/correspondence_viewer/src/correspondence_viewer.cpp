/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <nie/core/filesystem.hpp>
#include <nie/core/gflags.hpp>
#include <nie/core/string.hpp>
#include <nie/lidar/cloud.hpp>
#include <nie/lidar/helper_cloud.hpp>
#include <nie/lidar/shared_constants.hpp>

#include "helper_io.hpp"
#include "viewer.hpp"

DEFINE_string(debug_in_folder, "", "Input debug directory.");
DEFINE_validator(debug_in_folder, nie::ValidateIsDirectory);

// Helper functions
std::pair<int, int> GetIds(std::string const& s) {
    std::vector<int> set = nie::Split<int>(s, '-');
    CHECK(set.size() >= 2);
    return {set[0], set[1]};
}

template <typename PointT>
void Viewer() {
    std::string const folder_name = FLAGS_debug_in_folder + "/" + kFilteredCloudFolderName;
    CHECK(boost::filesystem::exists(folder_name))
            << "Expected folder '" << kFilteredCloudFolderName << "' to be present in the supplied debug folder.";
    auto files = nie::FindFiles(folder_name);

    nie::Viewer<PointT> viewer("correspondence_viewer");
    for (size_t index = 0; index < files.size(); index += 3) {
        std::string const file_name_cloud = files[index].string();
        std::string const file_name_correspondences = files[index + 2].string();

        std::pair<int, int> ids = GetIds(files[index].stem().string());
        CHECK(ids == GetIds(files[index + 2].stem().string()))
                << "Expected 3 files (2 cloud files and one correspondence file) for every loop.";
        LOG(INFO) << "Looking at the filtered clouds with id's " << ids.first << " and " << ids.second << ".";

        nie::Cloud<PointT> cloud = nie::ReadPlyFile<PointT>(file_name_cloud);

        std::vector<nie::Correspondence> correspondences = nie::ReadCorrespondenceCsv(file_name_correspondences);
        LOG_IF(INFO, correspondences.empty()) << "No correspondences are supplied.";

        viewer.Update(cloud, correspondences);
        viewer.View();
    }
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, false);

    Viewer<pcl::PointXYZI>();

    return 0;
}
