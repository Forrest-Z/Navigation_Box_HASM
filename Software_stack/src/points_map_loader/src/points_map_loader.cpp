/**
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * ------------------------------------------------------------------------------------
 * This is a modified version of the original point_map_loader.cpp code from Autoware.
 * The changes made to this code, of which a summary is listed below, are copyrighted:
 * ------------------------------------------------------------------------------------
 * Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission.
 * ------------------------------------------------------------------------------------
 *
 * List of changes:
 * * A lot of work has been done to refactor this file, with an focus on readability and maintainability.
 * * The code style has been adjusted in many places to conform to the AIIM Code Style.
 * * The input/tuning parameters are generalized and the defaults have been moved to the accompanying launch file.
 * * These parameters are now also all checked whether they are present, so the launch file is considered mandatory to
 * be used.
 * * All elements related to downloading the files from an http server have been removed as these are not relevant for
 * us.
 * * Duplicate pieces of code have been refactored to their own function which is called instead.
 * * Big functions have been split up into smaller atomic functions.
 * * The dynamic loading has been changed from a square range to circular
 * * The map is only updated when the reference position is X away from the previous position, where X is a tunable
 * parameter
 * * The map is only published if it is different from the previous map
 * * Functions have been renamed where necessary to make more sense
 * * Docstrings have been added to functions, where also is indicated whether those functions were already present in
 * the original version.
 * * The use of sensor_msgs::PointCloud2 has been changed to pcl::PointCloud<pcl::PointXYZ>
 * * Moved to ROS 2.0
 * * Use aiim_autoware_msgs for main pose update
 * * Remove initial z position as it was not used
 *
 * NOTE: Because this is modified code, the code style does not match the AIIM Code Style everywhere!
 *
 * TODO: (optional): Move to class structure
 */

#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <thread>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <aiim_autoware_msgs/msg/vehicle_kinematic_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace {

/** Structure to contain grid tile information
 * @param path: path to the file that contains the tile
 * @param x_min: minimum x value for points in the grid
 * @param y_min: minimum y value for points in the grid
 * @param z_min: minimum z value for points in the grid
 * @param x_max: maximum x value for points in the grid
 * @param y_max: maximum y value for points in the grid
 * @param z_max: maximum z value for points in the grid
 */
struct Area {
    std::string path;
    double x_min;
    double y_min;
    double z_min;
    double x_max;
    double y_max;
    double z_max;
};

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using AreaList = std::vector<Area>;
using Tbl = std::vector<std::vector<std::string>>;

//
rclcpp::Node::SharedPtr node;

// ROS parameters
static bool dynamic;                  // dynamic mode boolean, true = dynamic updating, false = static map
static double fallback_threshold_ms;  // Timeout when to use fallback signal
static double map_range;              // Range how far to load the map dynamically
static double map_range_sqrd;         // Squared value of above to speed up processing
static double update_distance;        // Minimum distance between updates
static double update_distance_sqrd;   // Squared value of above to speed up processing
static std::string reference_pose;    // topic name for reference pose
static std::string fallback_pose;     // topic name for fallback pose

// Map publisher
static rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub;

// Map shift publishing
static bool shift_published = false;                                       // Boolean if shift has been published yet
static rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr shift_pub;  // Publisher for publishing the map shift

// Dynamic map variables
static AreaList listed_areas;                    // Unique list of areas from the arealist file
static std::vector<size_t> current_map_indices;  // Indices of the tiles currently in the map
static float previous_x = 0.0;                   // Last input x position for which the map has been published
static float previous_y = 0.0;                   // Last input y position for which the map has been published
static rclcpp::Time time_since_main_callback;    // Time since the last reference signal received

// TODO (Optional): Move this to general AIIM ROS helper library?
template <typename T>
static void GetParamOrFail(std::string const& param_name, T& val) {
    if (node->get_parameter(param_name, val) == false) {
        RCLCPP_FATAL_STREAM(node->get_logger(), param_name << " is not set.");
        rclcpp::shutdown();
    }
}

/** Reads information from csv file
 *
 * @param path: Path to the grid information file
 * @return: 2d vector of strings
 * @note: Legacy autoware code
 */
static Tbl ReadCsv(std::string const& path) {
    std::ifstream ifs(path.c_str());
    if (!ifs) {
        RCLCPP_FATAL_STREAM(node->get_logger(), "Could not read the input csv file: " << path);
        rclcpp::shutdown();
    }
    std::string line;
    Tbl ret;
    while (std::getline(ifs, line)) {
        std::istringstream iss(line);
        std::string col;
        std::vector<std::string> cols;
        while (std::getline(iss, col, ',')) cols.push_back(col);
        ret.push_back(cols);
    }
    return ret;
}

/** Reads the Arealist from the grid_information file present in path
 *
 * @param path: Path to the grid information file
 * @return: List of areas found in the file
 * @note: Legacy Autoware Code!
 */
static AreaList ReadArealist(std::string const& path) {
    Tbl tbl = ReadCsv(path);
    AreaList ret;
    for (std::vector<std::string> const& cols : tbl) {
        Area area;
        area.path = cols[0];
        try {
            area.x_min = std::stod(cols[1]);
            area.y_min = std::stod(cols[2]);
            area.z_min = std::stod(cols[3]);
            area.x_max = std::stod(cols[4]);
            area.y_max = std::stod(cols[5]);
            area.z_max = std::stod(cols[6]);
        } catch (const std::invalid_argument&) {
            RCLCPP_WARN_STREAM(
                    node->get_logger(), "Arealist file has invalid values for item with path: " << area.path);
        } catch (const std::out_of_range&) {
            RCLCPP_WARN_STREAM(
                    node->get_logger(),
                    "Arealist file has a value that is out of range for item with path: " << area.path);
        }
        ret.push_back(area);
    }
    return ret;
}

/** Checks if the area is in range for the given point
 *
 * @param current_x: Reference x position to search around
 * @param current_y: Reference y position to search around
 * @param area: The area to compare
 * @param range_threshold: The threshold distance for an area to be considered in range
 * @returns true if it is in range, false if it is not
 */
static bool AreaIsInRange(double const current_x, double const current_y, Area const& area) {
    // Get closest distance to the map
    auto const calculate_delta = [](double const val, double const min, double const max) -> double {
        if (val < min) {
            return min - val;
        }
        if (val > max) {
            return val - max;
        }
        return 0.0;
    };

    double const dx = calculate_delta(current_x, area.x_min, area.x_max);
    double const dy = calculate_delta(current_y, area.y_min, area.y_max);
    return (dx * dx + dy * dy) <= map_range_sqrd;
}

/** Adds area to the list if it is not already in there
 *
 * @param area: The area to be added
 * @param areas: The current list of areas
 * @note: Legacy Autoware Code!
 */
static void UpdateArealist(Area const& area, AreaList& areas) {
    for (Area const& a : areas) {
        if (a.path == area.path) return;
    }
    areas.push_back(area);
}

/** Publishes the given pointcloud if its not empty
 *
 * @param pcd: Input (moved) point cloud
 */
static void PublishPcl(PointCloud::Ptr cloud) {
    if (cloud->width == 0) {
        RCLCPP_WARN(node->get_logger(), "Empty point cloud, not publishing");
        return;
    }

    auto cloud2 = sensor_msgs::msg::PointCloud2();
    pcl::toROSMsg(*cloud, cloud2);
    cloud2.header.frame_id = "map";
    cloud2.header.stamp = node->get_clock()->now();

    pcd_pub->publish(std::move(cloud2));
}

/** Adds given pcd file to the given cloud
 *
 * @param path [in]: Path to pcd file to add
 * @param cloud [in/out]: Cloud to be created or added
 */
static void AppendPcdToPcl(std::string const& path, PointCloud::Ptr cloud) {
    PointCloud::Ptr part(new PointCloud);
    // Load cloud from file
    if (pcl::io::loadPCDFile(path.c_str(), *part) == -1) {
        RCLCPP_WARN_STREAM(node->get_logger(), "Failed to load " << path);
        return;
    }
    // Check if the loaded cloud is valid
    if (part->width == 0) {
        RCLCPP_WARN_STREAM(node->get_logger(), "Loaded empty file " << path);
        return;
    }
    // Check if the cloud already has data
    if (cloud->width == 0) {
        *cloud = *part;
        if (!shift_published) {
            geometry_msgs::msg::Point msg;
            msg.x = cloud->sensor_origin_[0];
            msg.y = cloud->sensor_origin_[1];
            msg.z = cloud->sensor_origin_[2];
            shift_pub->publish(msg);
        }
        return;
    }
    // Add part to main cloud
    *cloud += *part;
}

/** Finds map around given point and publishes if different from current map
 *
 * @param current_x: Reference x position to search around
 * @param current_y: Reference y position to search around
 * @returns: vector of indices to the maps which are in range
 */
std::vector<size_t> GetMapsInRange(double const current_x, double const current_y) {
    std::vector<size_t> map_indices;
    for (size_t area_index = 0; area_index < listed_areas.size(); ++area_index) {
        if (AreaIsInRange(current_x, current_y, listed_areas[area_index])) {
            map_indices.push_back(area_index);
        }
    }
    return map_indices;
}

/** Finds map around given point and publishes if different from current map
 *
 * @param current_x: Reference x position to search around
 * @param current_y: Reference y position to search around
 */
static void PublishMapFromPosition(double const current_x, double const current_y) {
    // List the current maps in range
    std::vector<size_t> new_map_indices = GetMapsInRange(current_x, current_y);

    // Only publish the new map if it has changed since last iteration
    if (new_map_indices == current_map_indices) {
        RCLCPP_DEBUG(node->get_logger(), "Map has not changed, not publishing");
        return;
    }
    // Load new point cloud
    PointCloud::Ptr pcl(new PointCloud);
    for (auto const& area_index : new_map_indices) {
        AppendPcdToPcl(listed_areas[area_index].path, pcl);
    }
    RCLCPP_INFO_STREAM(
            node->get_logger(),
            "Found " << new_map_indices.size() << " grids in range with a total of " << pcl->points.size()
                     << " points.");
    // Update current state
    current_map_indices = new_map_indices;
    previous_x = current_x;
    previous_y = current_y;
    // Publish
    PublishPcl(pcl);
}

/** Updates map from pose update message
 *
 * @param current_x: Reference x position to search around
 * @param current_y: Reference y position to search around
 */
static void UpdateMap(double const current_x, double const current_y) {
    // Check if the car has moved enough
    // Note: For this we consider it to be a 2d distance from above, ignoring height.
    double const dpos_x = current_x - previous_x;
    double const dpos_y = current_y - previous_y;
    if (dpos_x * dpos_x + dpos_y * dpos_y < update_distance_sqrd) {
        RCLCPP_DEBUG_STREAM(
                node->get_logger(),
                "Update cancelled, car moved " << sqrt(dpos_x * dpos_x + dpos_y * dpos_y) << " which is less than "
                                               << update_distance);
        return;
    }
    PublishMapFromPosition(current_x, current_y);
}

/** Updates map from main pose update message
 *
 * @param msg: Vehicle state update message
 */
static void UpdateMapMain(aiim_autoware_msgs::msg::VehicleKinematicState::SharedPtr msg) {
    time_since_main_callback = node->get_clock()->now();
    UpdateMap(msg->state.x, msg->state.y);
}

/** Updates map from fallback pose update message
 *
 * @param msg: Pose update message
 */
static void UpdateMapFallback(geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Check time since last main callback time
    double const time_diff_ms = (node->get_clock()->now() - time_since_main_callback).seconds() * 1000;
    if (time_diff_ms < fallback_threshold_ms) {
        RCLCPP_DEBUG_STREAM(
                node->get_logger(),
                "Fallback update ignored, last update was " << time_diff_ms << " ms ago, the threshold is at "
                                                            << fallback_threshold_ms);
        return;
    }
    UpdateMap(msg->pose.position.x, msg->pose.position.y);
}

/** Loads a pointcloud from a set of paths, to be used for loading the map all at once
 *
 * @param pcd_paths: List of paths to pcd files
 * @note: Legacy Autoware Code!
 */
static void PublishFullMap(std::vector<std::string> const& pcd_paths) {
    PointCloud::Ptr pcl(new PointCloud);
    for (std::string const& path : pcd_paths) {
        AppendPcdToPcl(path, pcl);
        // Check if master is still up, as this might take a while and it could crash in the meantime.
        if (!rclcpp::ok()) break;
    }

    PublishPcl(pcl);
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("points_map_loader");

    // Assign parameters to node
    node->declare_parameter("dynamic");
    node->declare_parameter("map_range");
    node->declare_parameter("fallback_threshold_ms");
    node->declare_parameter("update_distance");
    node->declare_parameter("reference_pose");
    node->declare_parameter("fallback_pose");
    node->declare_parameter("pcd_dir");
    node->declare_parameter("arealist");
    node->declare_parameter("initial_x");
    node->declare_parameter("initial_y");
    node->declare_parameter("initial_z");

    // Update parameters
    GetParamOrFail("dynamic", dynamic);
    GetParamOrFail("map_range", map_range);
    GetParamOrFail("fallback_threshold_ms", fallback_threshold_ms);
    GetParamOrFail("update_distance", update_distance);
    GetParamOrFail("reference_pose", reference_pose);
    GetParamOrFail("fallback_pose", fallback_pose);
    // Derived parameters
    update_distance_sqrd = update_distance * update_distance;
    map_range_sqrd = map_range * map_range;

    // Get input files
    std::vector<std::string> pcd_paths;
    std::string pcd_file_dir;
    GetParamOrFail("pcd_dir", pcd_file_dir);

    // experimental for C++14, C++17 uses just std::filesystem
    for (auto const& entry : std::filesystem::directory_iterator(pcd_file_dir)) {
        if (entry.path().extension() == ".pcd") {
            pcd_paths.push_back(entry.path());
        }
    }

    std::string arealist_path;  // path to arealist file
    // Get input mode
    if (!dynamic) {
        RCLCPP_INFO(node->get_logger(), "Points Map Loader initialized in static mode (no dynamic update)");
    } else {
        GetParamOrFail("arealist", arealist_path);
        // Prepend source directory
        arealist_path = pcd_file_dir + "/" + arealist_path;
        RCLCPP_INFO(node->get_logger(), "Points Map Loader initialized in dynamic mode");
    }

    // Initialize publishers
    // Note: Because we are assigning a QoS here with a different durability (transient_local),
    // to subscribe to this topic, the subscriber will have to match this durability.
    rclcpp::QoS qos_pub(rclcpp::KeepLast(1));
    auto qos_subscribers = rclcpp::SystemDefaultsQoS();
    qos_pub.transient_local();
    pcd_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("points_map", qos_pub);
    shift_pub = node->create_publisher<geometry_msgs::msg::Point>("map_offset", qos_pub);

    // Initialize subscribers here otherwise they will go out of scope
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr fallback_sub;
    rclcpp::Subscription<aiim_autoware_msgs::msg::VehicleKinematicState>::SharedPtr current_sub;

    // Load map
    if (!dynamic) {
        // We are loading all of them at once.
        PublishFullMap(pcd_paths);
    } else {
        // We are loading the point clouds dynamically!
        // Subscribe to poses
        fallback_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
                fallback_pose, qos_subscribers, UpdateMapFallback);
        current_sub = node->create_subscription<aiim_autoware_msgs::msg::VehicleKinematicState>(
                reference_pose, qos_subscribers, UpdateMapMain);

        AreaList areas = ReadArealist(arealist_path);
        RCLCPP_INFO_STREAM(node->get_logger(), "Indexed " << areas.size() << " pcd files");
        for (Area const& area : areas) {
            for (std::string const& path : pcd_paths) {
                if (path == area.path) UpdateArealist(area, listed_areas);
            }
        }

        if (listed_areas.empty()) {
            RCLCPP_FATAL_STREAM(node->get_logger(), "Paths listed in arealist file are not present in pcd_file_dir");
            rclcpp::shutdown();
        }

        time_since_main_callback = node->get_clock()->now();

        // Get initial
        double initial_x;
        double initial_y;
        GetParamOrFail("initial_x", initial_x);
        GetParamOrFail("initial_y", initial_y);

        RCLCPP_INFO_STREAM(
                node->get_logger(),
                "Publishing map around initial position [" << initial_x << ", " << initial_y << "]");
        PublishMapFromPosition(initial_x, initial_y);
    }

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
