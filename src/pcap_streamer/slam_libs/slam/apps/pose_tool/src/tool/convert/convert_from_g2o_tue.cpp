/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "convert_from_g2o_tue.hpp"

#include <unordered_set>

#include <nie/core/gflags.hpp>
#include <nie/formats/ba_graph/collection_helper.hpp>
#include <nie/formats/csv/csv_read.hpp>
#include <nie/formats/csv/csv_record.hpp>
#include <nie/formats/weiya/image_time_in_day.hpp>

#include "tool/io.hpp"

#include "g2o_tue/g2o_tue.hpp"

DEFINE_string(in_dir_images, "", "The folder with the original image that were used to generate the g2o file.");
DEFINE_bool(convert_objects, false, "Also create objt and kpnt files.");

namespace detail {

//
// All collection-filtering functionality
//

using PoseIdSet = std::unordered_set<nie::io::PoseId>;
using ObjectIdSet = std::unordered_set<decltype(nie::io::KeypointRecord::object_id)>;

PoseIdSet GetPoseIds(nie::io::InfoRefCollection const& iref) {
    PoseIdSet ids{};
    ids.reserve(iref.info_refs.size());
    std::for_each(iref.info_refs.cbegin(), iref.info_refs.cend(), [&ids](auto const& i) {
        ids.emplace_hint(ids.end(), i.id);
    });
    return ids;
}

ObjectIdSet GetObjectIds(nie::io::KeypointCollection const& kpnt) {
    ObjectIdSet ids{};
    ids.reserve(kpnt.keypoints.size());
    std::for_each(kpnt.keypoints.cbegin(), kpnt.keypoints.cend(), [&ids](auto const& k) {
        ids.emplace_hint(ids.end(), k.object_id);
    });
    return ids;
}

void KeepKeyPoses(
        nie::io::InfoRefCollection const& iref,
        nie::io::PoseCollection* pose,
        nie::io::ObjectCollection* objt,
        nie::io::KeypointCollection* kpnt) {
    // Create set of unique pose id's
    PoseIdSet const pose_ids = GetPoseIds(iref);

    // Filter keypoints
    if (objt != nullptr && kpnt != nullptr) {
        kpnt->keypoints.erase(
                std::remove_if(
                        kpnt->keypoints.begin(),
                        kpnt->keypoints.end(),
                        [&pose_ids](auto const& k) { return pose_ids.count(k.pose_id) == 0; }),
                kpnt->keypoints.end());

        ObjectIdSet const object_ids = GetObjectIds(*kpnt);

        // Filter objects
        objt->objects.erase(
                std::remove_if(
                        objt->objects.begin(),
                        objt->objects.end(),
                        [&object_ids](auto const& o) { return object_ids.count(o.id) == 0; }),
                objt->objects.end());
    }

    // Filter edges
    pose->edges.erase(
            std::remove_if(
                    pose->edges.begin(),
                    pose->edges.end(),
                    [&pose_ids](auto const& e) {
                        return pose_ids.count(e.id_begin) == 0 || pose_ids.count(e.id_end) == 0;
                    }),
            pose->edges.end());

    // Filter poses
    pose->poses.erase(
            std::remove_if(
                    pose->poses.begin(),
                    pose->poses.end(),
                    [&pose_ids](auto const& p) { return pose_ids.count(p.id) == 0; }),
            pose->poses.end());
}

//
// Create the image reference collection
//

std::string FindRightImage(boost::filesystem::path const& image_dir, boost::filesystem::path left_image_path) {
    left_image_path = left_image_path.lexically_relative(image_dir);

    std::string right_pattern = ".+";
    right_pattern.append(left_image_path.stem().string());
    right_pattern.back() = 'R';
    right_pattern.append("\\" + left_image_path.extension().string());  // escape the . in the extension

    boost::filesystem::path const right_path = nie::FindFileRecursive(image_dir, right_pattern);
    CHECK(!right_path.empty()) << "No right image found named '" << right_pattern << "'";

    return right_path.string();
}

void ReadCsvTue(
        std::string const& file_name, boost::filesystem::path const& image_dir, nie::io::InfoRefCollection* iref) {
    // Manually add first image reference
    boost::filesystem::path const left_path = nie::FindFileRecursive(image_dir, ".+0000000001_L\\..+");
    CHECK(!left_path.empty()) << "No first image found";
    iref->info_refs.push_back({1, 0, left_path.string()});
    iref->info_refs.push_back({1, 1, FindRightImage(image_dir, left_path)});

    nie::CsvRecorderFast<std::string, int, std::string> csv_recorder{','};
    nie::ReadAllCsvToHandler<nie::ReadLinesAfterHeaderPredicate>({file_name}, "tag,id,path", &csv_recorder);
    auto const csv_records = csv_recorder.ConvertToRecord();

    iref->info_refs.reserve(csv_records.NumRows() * 2);
    for (std::size_t i = 0; i < csv_records.NumRows(); ++i) {
        auto const& row = csv_records.GetRow(i);
        if (std::get<0>(row) != "keyframe") {
            continue;
        }

        auto& id = std::get<1>(row);
        auto& path = std::get<2>(row);
        iref->info_refs.push_back({id, 0, path});
        iref->info_refs.push_back({id, 1, FindRightImage(image_dir, path)});
    }
}

void ParseTimestamps(nie::io::InfoRefCollection const& iref, nie::io::PoseCollection* pose) {
    auto map_pose_id_to_pose = CreateRecordMap(pose->poses.begin(), pose->poses.end());
    for (auto const& i : iref.info_refs) {
        map_pose_id_to_pose.at(i.id).get().timestamp = nie::io::weiya::GpsTimeFromFilename(i.path);
    }
    pose->header.flags |= nie::io::PoseHeader::kHasTimestampPerRecord;
}

//
// Writing of the collections
//

template <typename Collection>
void WriteCollection(Collection const& coll, boost::filesystem::path const& dir, std::string const& name) {
    nie::io::Write(coll, dir.string() + "/" + name + nie::io::graph::Extension<Collection>());
}

void WriteCollections(
        nie::io::InfoRefCollection const& iref,
        nie::io::PoseCollection const& pose,
        nie::io::ObjectCollection const* const p_objt,
        nie::io::KeypointCollection const* const p_kpnt,
        std::string const& dir) {
    WriteCollection(iref, dir, "info_refs");
    WriteCollection(pose, dir, "estimates");
    if (p_objt != nullptr) {
        WriteCollection(*p_objt, dir, "estimates");
    }
    if (p_kpnt != nullptr) {
        WriteCollection(*p_kpnt, dir, "constraints");
    }
}

}  // namespace detail

//
// Main conversion function
//

void ConvertFromG2oTue() {
    CHECK(nie::ValidateIsDirectory("in_dir_images", FLAGS_in_dir_images));

    boost::filesystem::path path;
    GetAndCheckInPathsForExtensionOrFatal(".g2o", &path);
    std::string const in_g2o_file_name = path.string();
    GetAndCheckInPathsForExtensionOrFatal(".csv", &path);
    std::string const in_csv_file_name = path.string();

    boost::filesystem::path out_dir(FLAGS_out_paths);
    if (!boost::filesystem::is_directory(out_dir)) {
        boost::filesystem::create_directories(out_dir);
    }

    {
        nie::io::InfoRefCollection iref{};
        nie::io::PoseCollection pose{};
        nie::io::ObjectCollection objt{};
        nie::io::KeypointCollection kpnt{};

        nie::io::ObjectCollection* const p_objt = (FLAGS_convert_objects ? &objt : nullptr);
        nie::io::KeypointCollection* const p_kpnt = (FLAGS_convert_objects ? &kpnt : nullptr);

        nie::io::SetNieAuthority(&pose.header);
        VLOG(1) << "Reading g2o file.";
        g2o_tue::ReadG2oTue(in_g2o_file_name, &pose, p_objt, p_kpnt);
        VLOG(1) << "Reading csv file.";
        detail::ReadCsvTue(in_csv_file_name, FLAGS_in_dir_images, &iref);
        VLOG(1) << "Filtering the poses.";
        detail::KeepKeyPoses(iref, &pose, p_objt, p_kpnt);
        VLOG(1) << "Parsing and adding timestamps.";
        detail::ParseTimestamps(iref, &pose);

        VLOG(1) << "Writing output files.";
        detail::WriteCollections(iref, pose, p_objt, p_kpnt, out_dir.string());
    }
}
