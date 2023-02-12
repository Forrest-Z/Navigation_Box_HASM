/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "arguments.hpp"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <nie/core/gflags.hpp>
#include <nie/core/string.hpp>
#include <nie/formats/ba_graph/collection_reader.hpp>

// TODO(jbr) Stereo.

// clang-format off

DEFINE_string(in_file_intrinsics, "", "Input intrinsics (.json).");

DEFINE_string(in_files_estimates, "",
        "Input estimates (.pose, .objt). Pose estimates are used as constraints if not explicitly defined in "
        "-in_files_constraints.");

// If no input constraints are given they are considered to be the same as the estimates.
DEFINE_string(in_files_constraints, "", "Input constraints (.kpnt).");

// TODO(jbr) When we implement the double pass for bundle adjustment, also the kpnt file needs to be output.
DEFINE_string(out_directory_ba_graph, "", "Output directory for bundle adjustment graph files (.pose, .objt).");
DEFINE_validator(out_directory_ba_graph, nie::ValidateStringNotEmpty);

DEFINE_bool(output_information_matrices, false,
        "If true, outputs the information matrix of each pose. Fixed poses have their matrix set to nan.");

DEFINE_bool(output_mahalanobis_distances, false, "If true, outputs the mahalanobis distances at the different stages.");
DEFINE_bool(double_pass, false, "When set, problem is optimized twice, with removal of outliers after the first pass.");
DEFINE_double(mahalanobis_threshold, 3.0,
        "Threshold (only) used for double pass filtering (in sigmas). Default is 3.0.");
DEFINE_double(inlier_percentage, 90., "");
// Unofficial parameter documentation
// When the mahalanobis_threshold value is set to -1.0 then the inlier_percentage parameter value will be used to
// determine the filtering of the loop constraints.

// clang-format on

// Extension format example: ".pose"
std::vector<boost::filesystem::path> GetPathsForExtension(
        std::vector<std::string> const& names, std::string const& extension) {
    std::vector<boost::filesystem::path> v;

    for (auto&& name : names) {
        boost::filesystem::path p(name);

        if (p.extension().string() == extension) {
            v.push_back(std::move(p));
        }
    }

    return v;
}

std::string FileThatMustExist(
        std::vector<boost::filesystem::path> const& v, std::string const& extension, std::string const& flagname) {
    CHECK(v.size() == 1) << "File with extension " + extension + " must only be present once in flag -" + flagname;
    CHECK(boost::filesystem::is_regular_file(v[0])) << "No file found for path: " + v[0].string();
    return v[0].string();
}

bool FileThatMayExist(
        std::vector<boost::filesystem::path> const& v,
        std::string const& extension,
        std::string const& flagname,
        std::string* filename) {
    if (v.empty()) {
        return false;
    }
    *filename = FileThatMustExist(v, extension, flagname);
    return true;
}

bool FileThatMayExist(std::string const& filename) {
    if (!filename.empty()) {
        return boost::filesystem::is_regular_file(filename);
    }
    return false;
}

auto constexpr kInFilesEstimates = "in_files_estimates";
auto constexpr kInFilesConstraints = "in_files_constraints";

auto const kExtensionPose = nie::io::graph::Extension<nie::io::PoseCollection>();
auto const kExtensionObjt = nie::io::graph::Extension<nie::io::ObjectCollection>();
auto const kExtensionKpnt = nie::io::graph::Extension<nie::io::KeypointCollection>();

BaArguments::BaArguments() {
    std::vector<std::string> estimates = nie::Split<std::string>(FLAGS_in_files_estimates);
    std::vector<std::string> constraints = nie::Split<std::string>(FLAGS_in_files_constraints);

    has_camera_parameters_ = FileThatMayExist(FLAGS_in_file_intrinsics);

    in_filename_pose_estimates_ =
            FileThatMustExist(GetPathsForExtension(estimates, kExtensionPose), kExtensionPose, kInFilesEstimates);

    has_objt_estimates_ = FileThatMayExist(
            GetPathsForExtension(estimates, kExtensionObjt),
            kExtensionObjt,
            kInFilesEstimates,
            &in_filename_objt_estimates_);

    has_kpnt_constraints_ = FileThatMayExist(
            GetPathsForExtension(constraints, kExtensionKpnt),
            kExtensionKpnt,
            kInFilesConstraints,
            &in_filename_kpnt_constraints_);
    has_objt_constraints_ = FileThatMayExist(
            GetPathsForExtension(constraints, kExtensionObjt),
            kExtensionObjt,
            kInFilesConstraints,
            &in_filename_objt_constraints_);
    has_pose_constraints_ = FileThatMayExist(
            GetPathsForExtension(constraints, kExtensionPose),
            kExtensionPose,
            kInFilesConstraints,
            &in_filename_pose_constraints_);

    boost::filesystem::path const out_directory_ba_graph(FLAGS_out_directory_ba_graph);
    if (!boost::filesystem::is_directory(out_directory_ba_graph)) {
        boost::filesystem::create_directories(out_directory_ba_graph);
    }
    out_directory_ba_graph_ = out_directory_ba_graph.string();
}

nie::io::RectifiedCameraParameters BaArguments::ReadCameraParameters() const {
    return nie::io::RectifiedCameraParameters::Read(FLAGS_in_file_intrinsics);
}

nie::io::PoseCollection BaArguments::ReadPoseEstimates() const {
    return nie::io::ReadCollection<nie::io::PoseCollection>(in_filename_pose_estimates_);
}

nie::io::ObjectCollection BaArguments::ReadObjtEstimates() const {
    return nie::io::ReadCollection<nie::io::ObjectCollection>(in_filename_objt_estimates_);
}

nie::io::KeypointCollection BaArguments::ReadKpntConstraints() const {
    return nie::io::ReadCollection<nie::io::KeypointCollection>(in_filename_kpnt_constraints_);
}

nie::io::PoseCollection BaArguments::ReadPoseConstraints() const {
    return nie::io::ReadCollection<nie::io::PoseCollection>(in_filename_pose_constraints_);
}

nie::io::ObjectCollection BaArguments::ReadObjtConstraints() const {
    return nie::io::ReadCollection<nie::io::ObjectCollection>(in_filename_objt_constraints_);
}

void BaArguments::WriteObjtEstimates(nie::io::ObjectCollection const& objt_collection) const {
    nie::io::Write(objt_collection, out_directory_ba_graph_ + "/ba" + kExtensionObjt);
}

void BaArguments::WritePoseEstimates(nie::io::PoseCollection const& pose_collection) const {
    nie::io::Write(pose_collection, out_directory_ba_graph_ + "/ba" + kExtensionPose);
}

std::string BaArguments::GetMahalanobisDistancesOutputFilename(std::string const& post_fix) const {
    return out_directory_ba_graph_ + "/mahalanobis_distances" + (post_fix.empty() ? "" : "_" + post_fix) + ".csv";
}

bool BaArguments::has_camera_parameters() const { return has_camera_parameters_; }

bool BaArguments::has_objt_estimates() const { return has_objt_estimates_; }
bool BaArguments::has_knpt_constraints() const { return has_kpnt_constraints_; }
bool BaArguments::has_pose_constraints() const { return has_pose_constraints_; }
bool BaArguments::has_objt_constraints() const { return has_objt_constraints_; }

bool BaArguments::output_information_matrices() const { return FLAGS_output_information_matrices; }

bool BaArguments::output_mahalanobis_distances() const { return FLAGS_output_mahalanobis_distances; }
bool BaArguments::double_pass() const { return FLAGS_double_pass; }
double BaArguments::mahalanobis_threshold() const { return FLAGS_mahalanobis_threshold; }
double BaArguments::inlier_percentage() const { return FLAGS_inlier_percentage; }
