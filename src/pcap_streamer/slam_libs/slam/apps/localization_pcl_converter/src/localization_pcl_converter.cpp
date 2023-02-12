/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
/** Localization Pointcloud Converter
 *
 * Reads pointcloud files, merges them, downsamples, and split them.
 * The output is given in a series of files that each represent a square area in a grid.
 * This allows the files to be read dynamically while the vehicle is moving to reduce the load on memory of loading a
 * whole map.
 *
 * The current desired implementation accepts as input:
 * - LAS pointcloud files (.las)
 * And outputs:
 * - PCD binary pointcloud files (.pcd)
 * - AUTOWARE area map information file (.csv)
 * - LAS binary pointcloud files (.las)
 * - NIE bounding box information file (.bbox)
 * - NIE pose information file (.pose)
 *
 * These outputs are only processed once the output dir is defined as argument.
 *
 * It also has some (optional) tuning arguments:
 * - voxel_grid_distance: This distance is used to downsample the point clouds. This downsampling is required to reduce
 * the computational load for the algorithm that uses the localization point cloud.
 * - map_grid_size: This is the size into which the final point clouds have to be split.
 *
 * TODO: Check if storing all the points of the listed las files is not too heavy on memory usage. Final point cloud
 * should not be bigger than 1GB anyway so it should be fine if we downsample the clouds before adding to the main
 * cloud?
 * TODO: (optional) Add toggle for pcd or las input
 */
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>

#include <nie/core/filesystem.hpp>
#include <nie/core/gflags.hpp>
#include <nie/formats/ba_graph/bbox_collection.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>

#include "nie/lidar/cloud.hpp"
#include "nie/lidar/cloud_filter.hpp"
#include "nie/lidar/io/las.hpp"
#include "nie/lidar/io/las_reader.hpp"

#include "cloud_grid_splitter.hpp"

DEFINE_string(dir_in, "", "Directory for input .las files.");
DEFINE_string(las_out, "", "Directory for output las files.");
DEFINE_string(pcd_out, "", "Directory for output pcd files.");
DEFINE_string(nie_out, "", "Directory for output nie binary files.");
DEFINE_double(voxel_grid_distance, 0.15, "Downsampling distance for output map.");
DEFINE_double(map_grid_size, 100.0, "Size of map file grid.");

DEFINE_validator(dir_in, nie::ValidateIsDirectory);
DEFINE_validator(voxel_grid_distance, nie::ValidateLargerThanZero);
DEFINE_validator(map_grid_size, nie::ValidateLargerThanZero);

using PclPoint = pcl::PointXYZ;

void write_pcd_files(std::vector<nie::Cloud<PclPoint>> const& cloud_split) {
    VLOG(2) << "Writing pcd files";
    auto const output_dirpath = boost::filesystem::path(FLAGS_pcd_out);
    // Open csv writer for area map
    auto const grid_information_filepath = output_dirpath / "grid_information.csv";
    std::ofstream ofs(grid_information_filepath.c_str());
    // Loop over split clouds
    for (size_t cloud_it = 0; cloud_it < cloud_split.size(); ++cloud_it) {
        std::string const pcd_filename = "pcd_" + std::to_string(cloud_it) + ".pcd";
        auto const pcd_filepath = output_dirpath / pcd_filename;
        VLOG(4) << "Writing PCD file " << pcd_filepath;
        pcl::io::savePCDFileBinary(pcd_filepath.c_str(), cloud_split[cloud_it].point_cloud());

        // Add line with bounding box information to arealist csv file
        nie::PoseBbox const& bounds = cloud_split[cloud_it].bounds();
        ofs << pcd_filepath.c_str() << "," << bounds.bbox().min.x() << "," << bounds.bbox().min.y() << ","
            << bounds.bbox().min.z() << "," << bounds.bbox().max.x() << "," << bounds.bbox().max.y() << ","
            << bounds.bbox().max.z() << "\n";
    }
}

void write_las_files(std::vector<nie::Cloud<PclPoint>> const& cloud_split) {
    VLOG(2) << "Writing las files";
    auto const output_dirpath = boost::filesystem::path(FLAGS_las_out);
    for (size_t cloud_it = 0; cloud_it < cloud_split.size(); ++cloud_it) {
        nie::PoseBbox const& bounds = cloud_split[cloud_it].bounds();
        std::string const las_filename = "las_" + std::to_string(cloud_it) + ".las";
        auto const las_filepath = output_dirpath / las_filename;
        // Convert from nie::cloud to las cloud (without timing or intensity information)
        std::vector<nie::io::LasPoint<pdal::Dimension::Id::X, pdal::Dimension::Id::Y, pdal::Dimension::Id::Z>>
                las_points;
        for (auto point_it = cloud_split[cloud_it].point_cloud().begin();
             point_it < cloud_split[cloud_it].point_cloud().end();
             ++point_it) {
            las_points.emplace_back(
                    point_it->x + bounds.origin().translation()[0],
                    point_it->y + bounds.origin().translation()[1],
                    point_it->z + bounds.origin().translation()[2]);
        }
        // Write to file
        VLOG(4) << "Writing file " << las_filepath;
        nie::io::WriteLas(las_filepath, las_points);
    }
}

void write_nie_files(std::vector<nie::Cloud<PclPoint>> const& cloud_split) {
    VLOG(2) << "Writing nie binary files";
    auto const output_dirpath = boost::filesystem::path(FLAGS_nie_out);
    std::string const bbox_bbox = "bbox" + nie::io::graph::Extension<nie::io::BboxCollection>();
    std::string const bbox_pose = "bbox" + nie::io::graph::Extension<nie::io::PoseCollection>();
    nie::io::BboxCollectionStreamWriter bbox_collection_writer{(output_dirpath / bbox_bbox).c_str(), {}};
    nie::io::PoseCollectionStreamWriter pose_collection_writer{(output_dirpath / bbox_pose).c_str(), {}};
    for (size_t cloud_it = 0; cloud_it < cloud_split.size(); ++cloud_it) {
        nie::PoseBbox const& bounds = cloud_split[cloud_it].bounds();
        // Write bounds and pose info to nie binary files
        // We are using the cloud iterator as pose id.
        nie::io::PoseId pose_id = cloud_it;
        pose_collection_writer.Write(
                nie::io::PoseRecord{pose_id, nie::io::PoseRecord::Category::kBbox, {}, {}, bounds.origin(), {}});
        bbox_collection_writer.Write(
                nie::io::BboxRecord{pose_id, bounds.bbox().min.cast<double>(), bounds.bbox().max.cast<double>()});
    }
}

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, false);
    google::InitGoogleLogging(argv[0]);

    // Check output directories if output flag is set
    CHECK(FLAGS_las_out.empty() || nie::ValidateIsDirectory("dir_out_las", FLAGS_las_out));
    CHECK(FLAGS_pcd_out.empty() || nie::ValidateIsDirectory("dir_out_pcd", FLAGS_pcd_out));
    CHECK(FLAGS_nie_out.empty() || nie::ValidateIsDirectory("dir_out_nie", FLAGS_nie_out));
    CHECK(!FLAGS_las_out.empty() || !FLAGS_pcd_out.empty() || !FLAGS_nie_out.empty()) << "No desired output is given";

    // Create storage variables
    nie::Cloud<PclPoint> cloud_combined;
    std::chrono::weeks gps_week_in;
    bool cloud_initialized = false;

    // Initialize CloudFilter for Downsampling the point cloud
    nie::CloudFilter<PclPoint> downsampler{nie::CloudFilter<PclPoint>::Parameters(FLAGS_voxel_grid_distance)};

    // Find files in directory
    auto files_in_dir = nie::FindFiles(FLAGS_dir_in, ".*.las");
    VLOG(2) << "Found " << files_in_dir.size() << " las files in directory " << FLAGS_dir_in;

    // Loop through files
    for (auto const& filename : files_in_dir) {
        // Read from las file and downsample
        nie::Cloud<PclPoint> cloud_single =
                downsampler.Filter(nie::io::ReadLas<PclPoint>(filename.string(), gps_week_in));

        // Change origin to make everything in line with original origin
        if (cloud_initialized) {
            cloud_single.ChangeOrigin(cloud_combined.origin().translation());
            cloud_combined.point_cloud() += cloud_single.point_cloud();
        } else {
            cloud_combined = cloud_single;
            cloud_initialized = true;
        }
        // Add downsampled cloud to the total point cloud
        VLOG(3) << "Added " << cloud_single.point_cloud().points.size() << " (downsampled) points to map from "
                << filename.string();
    }

    // Split point cloud into grids
    nie::CloudGridSplitter<PclPoint> cloud_grid_splitter(static_cast<float>(FLAGS_map_grid_size));
    std::vector<nie::Cloud<PclPoint>> cloud_split = cloud_grid_splitter.Split(std::move(cloud_combined));

    // Downsample the split pointclouds to remove overlapping input clouds
    for (auto& cloud : cloud_split) {
        cloud = downsampler.Filter(cloud);
    }

    // Write output files
    if (!FLAGS_pcd_out.empty()) {
        write_pcd_files(cloud_split);
    }
    if (!FLAGS_las_out.empty()) {
        write_las_files(cloud_split);
    }
    if (!FLAGS_nie_out.empty()) {
        write_nie_files(cloud_split);
    }

    return 0;
}
