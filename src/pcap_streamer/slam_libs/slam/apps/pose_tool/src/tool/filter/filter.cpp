/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "filter.hpp"

#include <iostream>
#include <unordered_set>

#include <nie/core/gflags.hpp>
#include <nie/core/string.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>

#include "tool/io.hpp"

DEFINE_string(
    selection,
    "",
    "The comma-separated pose categories that are selected to be kept after filtering. Values expected are 'gps', "
    "'odom' and 'bbox'.");

// The category parsing can easily be extended in case edge filtering is required

template <typename Category>
Category ParseCategory(std::string const&) {
    static_assert(
        std::is_same_v<Category, nie::io::PoseRecord::Category>, "ParseCategory: Not implemented for supplied type.");
}
template <>
nie::io::PoseRecord::Category ParseCategory<nie::io::PoseRecord::Category>(std::string const& value) {
    if (value == "gps") {
        return nie::io::PoseRecord::Category::kGps;
    } else if (value == "odom") {
        return nie::io::PoseRecord::Category::kOdom;
    } else if (value == "bbox") {
        return nie::io::PoseRecord::Category::kBbox;
    }
    LOG(FATAL) << "The supplied category '" << value
               << "' is unknown. Expecting one of the values 'gps', 'odom' and 'bbox'.";
}

template <typename Category>
std::unordered_set<Category> ParseCategories(std::string values) {
    nie::ToLower(&values);
    auto const split_values = nie::Split<std::string>(values, ',');
    std::unordered_set<nie::io::PoseRecord::Category> result;
    std::transform(
        split_values.cbegin(), split_values.cend(), std::inserter(result, result.end()), ParseCategory<Category>);
    return result;
}

void Filter() {
    LOG_IF(FATAL, FLAGS_selection.empty()) << "Parameters 'selection' should be set.";

    std::string const& pose_ext = nie::io::graph::Extension<nie::io::PoseCollection>();

    CheckOutPathsLocationsOrFatal();

    boost::filesystem::path path;
    GetAndCheckInPathsForExtensionOrFatal(pose_ext, &path);
    std::string const in_pose_file_name = path.string();
    GetAndCheckOutPathsForExtensionOrFatal(pose_ext, &path);
    std::string const out_pose_file_name = path.string();

    // Parse the categories that should are selected to be kept
    auto const pose_categories = ParseCategories<nie::io::PoseRecord::Category>(FLAGS_selection);
    LOG_IF(FATAL, FLAGS_selection.empty()) << "Parameters 'selection' should be set.";

    LOG(INFO) << "Reading " << pose_ext << " file '" << in_pose_file_name << "'.";
    auto const input = nie::io::ReadCollection<nie::io::PoseCollection>(in_pose_file_name);
    nie::io::PoseCollection output{input.header, {}, {}, {}};

    auto const input_pose_map = nie::io::CreateRecordMap(input.poses.cbegin(), input.poses.cend());

    // Create function that determine which records should be copies
    auto const copy_pose = [&pose_categories](nie::io::PoseRecord const& p) -> bool {
        return pose_categories.count(p.category) > 0;
    };
    auto const copy_pose_id = [&input_pose_map, &copy_pose](nie::io::PoseId const& id) -> bool {
        return copy_pose(input_pose_map.at(id));
    };
    auto const copy_fix = [&copy_pose_id](nie::io::FixedPoseRecord const& f) -> bool { return copy_pose_id(f.id); };
    auto const copy_edge = [&copy_pose_id](nie::io::PoseEdgeRecord const& e) -> bool {
        return copy_pose_id(e.id_begin) && copy_pose_id(e.id_end);
    };

    // Copy the records applicable
    std::copy_if(input.poses.cbegin(), input.poses.cend(), std::back_inserter(output.poses), copy_pose);
    std::copy_if(input.fixes.cbegin(), input.fixes.cend(), std::back_inserter(output.fixes), copy_fix);
    std::copy_if(input.edges.cbegin(), input.edges.cend(), std::back_inserter(output.edges), copy_edge);

    // Actually read the records from the input and conditionally write the records to the output
    nie::io::Write(output, out_pose_file_name);
    LOG(INFO) << "Filtered output " << pose_ext << " file '" << out_pose_file_name << "' written.";
}
