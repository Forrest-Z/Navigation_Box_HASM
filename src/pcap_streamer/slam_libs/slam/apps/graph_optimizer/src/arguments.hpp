/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#pragma once

#include <string>

#include <nie/formats/ba_graph.hpp>
#include <nie/formats/calib3d/rectified_parameters.hpp>

class BaArguments {
public:
    BaArguments();
    [[nodiscard]] nie::io::RectifiedCameraParameters ReadCameraParameters() const;

    [[nodiscard]] nie::io::PoseCollection ReadPoseEstimates() const;
    [[nodiscard]] nie::io::ObjectCollection ReadObjtEstimates() const;

    [[nodiscard]] nie::io::KeypointCollection ReadKpntConstraints() const;
    [[nodiscard]] nie::io::PoseCollection ReadPoseConstraints() const;
    [[nodiscard]] nie::io::ObjectCollection ReadObjtConstraints() const;

    void WriteObjtEstimates(nie::io::ObjectCollection const& objt_collection) const;
    void WritePoseEstimates(nie::io::PoseCollection const& pose_collection) const;

    [[nodiscard]] std::string GetMahalanobisDistancesOutputFilename(std::string const& post_fix = "") const;

    [[nodiscard]] bool has_camera_parameters() const;

    [[nodiscard]] bool has_objt_estimates() const;
    [[nodiscard]] bool has_knpt_constraints() const;
    [[nodiscard]] bool has_objt_constraints() const;
    [[nodiscard]] bool has_pose_constraints() const;

    [[nodiscard]] bool output_information_matrices() const;

    [[nodiscard]] bool output_mahalanobis_distances() const;
    [[nodiscard]] bool double_pass() const;
    [[nodiscard]] double mahalanobis_threshold() const;
    [[nodiscard]] double inlier_percentage() const;

private:
    std::string in_filename_pose_estimates_;
    std::string in_filename_objt_estimates_;

    std::string in_filename_kpnt_constraints_;
    std::string in_filename_pose_constraints_;
    std::string in_filename_objt_constraints_;

    std::string out_directory_ba_graph_;

    bool has_camera_parameters_;

    bool has_objt_estimates_;
    bool has_kpnt_constraints_;
    bool has_pose_constraints_;
    bool has_objt_constraints_;
};
