/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <unordered_map>

#include <gtest/gtest.h>
#include <boost/filesystem/path.hpp>
#include <nie/cv/geometry/triangulation.hpp>
#include <nie/formats/calib3d/extended_mono_parameters.hpp>
#include <nie/formats/calib3d/rectified_parameters.hpp>
#include <nie/formats/calib3d/validated_mono_parameters.hpp>
#include <opencv2/calib3d.hpp>

TEST(TriangulationTest, Triangulation) {
    nie::io::ExtendedMonoParameters extended = nie::io::ExtendedMonoParameters::Read(
        "/data/aiim/unit_tests_data/cv/calib3d/calibration/mono/extended_mono.json");
    nie::io::RectifiedCameraParameters rectified = nie::io::RectifiedCameraParameters::Read(
        "/data/aiim/unit_tests_data/cv/calib3d/calibration/mono/rectified_intrinsics_mono_vector.json");
    nie::io::ValidatedMonoParameters validated = nie::io::ValidatedMonoParameters::Read(
        "/data/aiim/unit_tests_data/cv/calib3d/calibration/mono/validated_mono.json");

    std::unordered_map<std::string, std::size_t> id_to_index;

    for (std::size_t i = 0; i < extended.extended_data().image_ids.size(); ++i) {
        // The ids are actually not the same because the paths are different (which is correct because
        // they are different images, but we want to relate them now).
        std::string id = boost::filesystem::path(extended.extended_data().image_ids[i]).stem().string();
        id_to_index.insert(std::make_pair(id, i));
    }

    std::vector<Eigen::Vector3f> object_points = extended.object_points();
    std::vector<nie::Isometry3qd> extrinsics_matched;

    // TODO(jbr): Fix this when we get some proper test data.
    // TODO(jbr): Fix with future covariance matrices
    for (std::size_t i = 0; i < validated.validated_data().image_ids.size(); ++i) {
        if (validated.validated_data().residuals[i].size() == 0) continue;

        std::string id = boost::filesystem::path(validated.validated_data().image_ids[i]).stem().string();
        std::size_t index = id_to_index[id];

        extrinsics_matched.push_back(extended.extended_data().extrinsics[index]);
    }

    Eigen::Matrix3d K = rectified.K;

    for (std::size_t c = 0; c < object_points.size(); ++c) {
        std::vector<Eigen::Vector2f> image_points;
        for (std::size_t i = 0; i < validated.validated_data().image_ids.size(); ++i) {
            if (validated.validated_data().residuals[i].size() == 0) continue;
            // for each image_id we get the image_point for each corner
            image_points.push_back(validated.validated_data().image_points[i][c]);
        }

        Eigen::Vector3d x;
        nie::TriangulateNonLinear(extrinsics_matched, image_points, K, &x);

        ASSERT_NEAR((object_points[c].cast<double>() - x).norm(), 0.0, 0.0025);
    }
}
