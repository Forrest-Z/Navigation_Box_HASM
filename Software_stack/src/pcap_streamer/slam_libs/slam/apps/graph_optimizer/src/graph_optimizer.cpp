/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gflags/gflags.h>
#include <boost/filesystem.hpp>

#include "arguments.hpp"
#include "solve.hpp"

bool HasValidConstraints(nie::io::ObjectCollection const& collection) { return collection.header.HasAnyInformation(); }

// This will just check what is directly checkable. It will not check if the fixes are relevant, if there are islands in
// the optimization problem, etc.
bool HasValidConstraints(nie::io::PoseCollection const& collection) {
    bool has_pose_information = collection.header.HasAnyPoseInformation();
    bool has_edge_information = collection.header.HasAnyEdgeInformation();
    // Needs a minimum of two constraint poses (fixes the scale and rotation optimization gauge freedom).
    bool valid_for_poses = collection.poses.size() > 1 && (has_pose_information || (collection.fixes.size() > 1));
    // Also needs two constraint poses. However, the 2nd one is constraint by an edge requiring only 1 fix.
    bool valid_for_edges = !collection.edges.empty() && (collection.poses.size() > 1) &&
                           (has_pose_information || !collection.fixes.empty()) && has_edge_information;
    // Basic bundle adjustment
    bool valid_for_bba = valid_for_poses;
    // Pose graph optimization
    bool valid_for_pgo = valid_for_edges;
    // Amazing bundle adjustment. This is just to show explicitly that the full thing means to use all constraints.
    bool valid_for_aba = valid_for_poses && valid_for_edges;
    // The valid_for_aba is not really needed but makes intention more clear.
    VLOG(3) << "Valid for basic bundle adjustment: " << std::boolalpha << valid_for_bba;
    VLOG(3) << "Valid for pose graph optimization: " << std::boolalpha << valid_for_pgo;
    VLOG(3) << "Valid for all in one optimization: " << std::boolalpha << valid_for_aba;
    return valid_for_bba || valid_for_pgo || valid_for_aba;
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    // Note that the estimates and constraints are used in the following way:
    //   Estimates
    //     - pose collection poses -> parameter
    //     - object collection objects -> parameter
    //   Constraints
    //     - pose collection poses -> residual
    //     - pose collection edges -> residual
    //     - pose collection fixes -> set pose estimate parameter fixed
    //     - object collection objects -> residual
    //     - keypoint collection keypoints -> residual

    BaArguments arguments;
    nie::io::PoseCollection pose_estimates = arguments.ReadPoseEstimates();
    nie::io::ObjectCollection objt_estimates{};
    nie::io::KeypointCollection kpnt_constraints{};
    nie::io::PoseCollection pose_constraints{};
    nie::io::ObjectCollection objt_constraints{};
    nie::io::RectifiedCameraParameters camera_parameters{};

    if (arguments.has_pose_constraints()) {
        pose_constraints = arguments.ReadPoseConstraints();
        LOG(INFO) << "Using pose constraints.";
    } else {
        LOG(INFO) << "Using pose estimates as constraints.";
        pose_constraints = pose_estimates;
    }
    // Pose constraints are mandatory.
    CHECK(HasValidConstraints(pose_constraints)) << "Invalid input constraints for poses.";

    if (arguments.has_knpt_constraints()) {
        LOG(INFO) << "Using keypoint constraints.";
        kpnt_constraints = arguments.ReadKpntConstraints();

        CHECK(arguments.has_camera_parameters()) << "Intrinsics not provided.";
        camera_parameters = arguments.ReadCameraParameters();

        // Objects can only be read when there are keypoints to constrain them
        CHECK(arguments.has_objt_estimates()) << "Object estimates not provided.";
        objt_estimates = arguments.ReadObjtEstimates();

        if (arguments.has_objt_constraints()) {
            objt_constraints = arguments.ReadObjtConstraints();
            LOG(INFO) << "Using object constraints.";
            // Only check when object constraints are given as input.
            CHECK(HasValidConstraints(objt_constraints)) << "Invalid input constraints for objects.";
        }
    }

    bool success = Solve(
            camera_parameters,
            kpnt_constraints,
            objt_constraints,
            arguments.output_information_matrices(),
            arguments.double_pass(),
            arguments.mahalanobis_threshold(),
            arguments.inlier_percentage(),
            arguments.output_mahalanobis_distances(),
            [arguments](std::string const& post_fix) -> std::string {
                return arguments.GetMahalanobisDistancesOutputFilename(post_fix);
            },
            &pose_constraints,
            &pose_estimates,
            &objt_estimates);

    if (success) {
        if (!arguments.has_pose_constraints()) {
            // Estimates were used as constraints, but edge constraints are filtered, so copy those back to the
            // estimates
            pose_estimates.edges = pose_constraints.edges;
        }

        LOG(INFO) << "Writing pose file...";
        arguments.WritePoseEstimates(pose_estimates);
        if (arguments.has_knpt_constraints()) {
            arguments.WriteObjtEstimates(objt_estimates);
        }
        return 0;
    } else {
        LOG(ERROR) << "Graph optimizer failed.";
        return 1;
    }
}
