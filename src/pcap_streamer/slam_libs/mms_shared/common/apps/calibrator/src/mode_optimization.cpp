/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "mode_optimization.hpp"

#include <chrono>

#include <glog/logging.h>

#include <boost/filesystem/path.hpp>
#include <nie/cv/calib3d/camera_calibration.hpp>
#include <nie/formats/calib3d/extended_mono_parameters.hpp>
#include <nie/formats/calib3d/extended_stereo_parameters.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "coverage_optimization.hpp"

// FIXME(jbr): This is nearly a duplicate of mode_calibration in calling the calibration.
// FIXME(jbr): It is a LOT more easy to refactor after convention changes (needing only minor changes) than fixing
// FIXME(jbr): it elaborately twice with adding and removing conversions while it should be an interface change only.

/*
void GenerateCoverageImages(
    cv::Size const& image_size,
    cv::Size const& pattern_size,
    std::vector<std::vector<cv::Point2f>> const& image_points,
    std::string out_coverage_image_path) {
    cv::Mat coverage_map;
    coverage_map = cv::Mat::zeros(image_size, CV_8UC1);

    for (std::vector<cv::Point2f> const& points : image_points) {
        std::vector<cv::Point> pattern_polygon{points[0],
                                               points[pattern_size.width * (pattern_size.height - 1)],
                                               points[(pattern_size.width * pattern_size.height) - 1],
                                               points[pattern_size.width - 1]};

        cv::Rect pattern_bbox = cv::boundingRect(pattern_polygon);
        cv::Mat coverage_roi = cv::Mat(coverage_map, pattern_bbox);

        // coverage on the mat is actually calculated per-pixel and not per bounding box
        coverage_roi.forEach<uint8_t>(
            [&pattern_polygon, &pattern_bbox](uint8_t& pixel_value, const int* position) -> void {
                // naturally, we must offset by the origin of the ROI
                cv::Point2f current_pixel = cv::Point2f(position[1] + pattern_bbox.x, position[0] + pattern_bbox.y);

                bool pixel_in_checkerboard = pointPolygonTest(pattern_polygon, current_pixel, false) > 0;

                if (pixel_in_checkerboard) {
                    ++pixel_value;
                }
            });
    }

    cv::Mat black_mask = coverage_map > 0;
    cv::normalize(coverage_map, coverage_map, 32, 255, cv::NORM_MINMAX, -1, black_mask);
    cv::resize(coverage_map, coverage_map, coverage_map.size() / 2);
    cv::imwrite(out_coverage_image_path, coverage_map);
}

void OptimizeMono(std::string const& in_path_extended) {
    // ceres output
    std::vector<double> intrinsics;
    std::vector<cv::Point3d> positions;
    std::vector<cv::Vec3d> orientations;
    std::vector<std::vector<cv::Point2d>> residuals;
    std::vector<double> stddevs_intrinsics;
    std::vector<std::vector<double>> stddevs_extrinsics;
    double stddev_residuals;

    auto extended_data = nie::io::ExtendedMonoParameters::Read(in_path_extended);

    if (extended_data.extended_data().image_ids.empty()) {
        // assume we have been fed a stereo file by mistake
        throw std::runtime_error("Invalid input for mono optimization. Maybe a stereo output file was passed?");
    }

    GenerateCoverageImages(
        extended_data.image_size(),
        extended_data.pattern_size(),
        extended_data.extended_data().image_points,
        boost::filesystem::path(in_path_extended).filename().stem().string() + ".png");

    auto before_optimization = std::chrono::system_clock::now();
    std::vector<std::string> optimized_ids = extended_data.extended_data().image_ids;
    std::vector<std::vector<cv::Point2f>> optimized_image_points = extended_data.extended_data().image_points;

    OptimizeSpatialCoverage(
        extended_data.image_size(), extended_data.pattern_size(), &optimized_ids, &optimized_image_points);

    VLOG(3) << " Optimized spatial coverage in "
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now() - before_optimization)
                   .count()
            << " ms";

    nie::EstimateCameraParametersCeres(
        extended_data.distortion_parameters(),
        extended_data.object_points(),
        optimized_image_points,
        &intrinsics,
        &positions,
        &orientations,
        &residuals,
        &stddevs_intrinsics,
        &stddevs_extrinsics,
        &stddev_residuals);

    nie::io::ExtendedMonoParameters::Write(
        boost::filesystem::path(in_path_extended).filename().stem().string() + "_optimized.json",
        extended_data.image_size(),
        extended_data.distortion_parameters(),
        extended_data.pattern_size(),
        extended_data.square_size(),
        extended_data.object_points(),
        stddev_residuals,
        extended_data.mono_parameters().optics_id(),
        optimized_ids,
        optimized_image_points,
        intrinsics,
        positions,
        orientations,
        residuals,
        stddevs_intrinsics,
        stddevs_extrinsics);

    GenerateCoverageImages(
        extended_data.image_size(),
        extended_data.pattern_size(),
        optimized_image_points,
        boost::filesystem::path(in_path_extended).filename().stem().string() + "_optimized.png");

    VLOG(3) << " Successfully wrote the optimized intrinsics and extended data.";
}

void OptimizeStereo(std::string const& in_path_extended) {
    // ceres output
    std::vector<double> intrinsics_left;
    std::vector<double> intrinsics_right;
    std::vector<cv::Point3d> positions_left;
    std::vector<cv::Vec3d> orientations_left;
    std::vector<cv::Point3d> positions_right;
    std::vector<cv::Vec3d> orientations_right;
    std::vector<double> baseline;
    std::vector<std::vector<cv::Point2d>> residuals_left;
    std::vector<std::vector<cv::Point2d>> residuals_right;
    std::vector<double> stddevs_intrinsics_left;
    std::vector<double> stddevs_intrinsics_right;
    std::vector<std::vector<double>> stddevs_extrinsics_left;
    std::vector<std::vector<double>> stddevs_extrinsics_right;
    std::vector<double> stddevs_baseline;
    double stddev_residuals;

    auto extended_data = nie::io::ExtendedStereoParameters::Read(in_path_extended);

    if (extended_data.extended_data_left().image_ids.empty()) {
        // assume we have been fed a mono file by mistake
        throw std::runtime_error("Invalid input for stereo optimization. Maybe a mono output file was passed?");
    }

    GenerateCoverageImages(
        extended_data.image_size(),
        extended_data.pattern_size(),
        extended_data.extended_data_left().image_points,
        boost::filesystem::path(in_path_extended).filename().stem().string() + "_left.png");

    GenerateCoverageImages(
        extended_data.image_size(),
        extended_data.pattern_size(),
        extended_data.extended_data_right().image_points,
        boost::filesystem::path(in_path_extended).filename().stem().string() + "_right.png");

    auto before_optimization = std::chrono::system_clock::now();
    std::size_t read_pair_count = extended_data.pair_count();
    auto optimized_ids_left = extended_data.extended_data_left().image_ids;
    auto optimized_ids_right = extended_data.extended_data_right().image_ids;
    auto optimized_image_points_left = extended_data.extended_data_left().image_points;
    auto optimized_image_points_right = extended_data.extended_data_right().image_points;
    std::size_t optimized_pair_count;

    OptimizeSpatialCoverage(
        extended_data.image_size(),
        extended_data.pattern_size(),
        read_pair_count,
        &optimized_ids_left,
        &optimized_ids_right,
        &optimized_image_points_left,
        &optimized_image_points_right,
        &optimized_pair_count);

    VLOG(3) << " Optimized spatial coverage in "
            << std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now() - before_optimization)
                   .count()
            << " ms";

    nie::EstimateStereoParametersCeres(
        extended_data.distortion_parameters(),
        extended_data.object_points(),
        optimized_image_points_left,
        optimized_image_points_right,
        optimized_pair_count,
        &intrinsics_left,
        &intrinsics_right,
        &positions_left,
        &orientations_left,
        &positions_right,
        &orientations_right,
        &baseline,
        &residuals_left,
        &residuals_right,
        &stddevs_intrinsics_left,
        &stddevs_intrinsics_right,
        &stddevs_extrinsics_left,
        &stddevs_extrinsics_right,
        &stddevs_baseline,
        &stddev_residuals);

    nie::io::ExtendedStereoParameters::Write(
        boost::filesystem::path(in_path_extended).filename().stem().string() + "_optimized.json",
        extended_data.image_size(),
        extended_data.distortion_parameters(),
        extended_data.pattern_size(),
        extended_data.square_size(),
        extended_data.object_points(),
        stddev_residuals,
        extended_data.stereo_parameters().optics_id_left(),
        extended_data.stereo_parameters().optics_id_right(),
        optimized_ids_left,
        optimized_ids_right,
        optimized_image_points_left,
        optimized_image_points_right,
        intrinsics_left,
        intrinsics_right,
        positions_left,
        positions_right,
        orientations_left,
        orientations_right,
        residuals_left,
        residuals_right,
        stddevs_intrinsics_left,
        stddevs_intrinsics_right,
        stddevs_extrinsics_left,
        stddevs_extrinsics_right,
        optimized_pair_count,
        baseline,
        stddevs_baseline);

    GenerateCoverageImages(
        extended_data.image_size(),
        extended_data.pattern_size(),
        optimized_image_points_left,
        boost::filesystem::path(in_path_extended).filename().stem().string() + "_optimized_left.png");

    GenerateCoverageImages(
        extended_data.image_size(),
        extended_data.pattern_size(),
        optimized_image_points_right,
        boost::filesystem::path(in_path_extended).filename().stem().string() + "_optimized_right.png");

    VLOG(3) << " Successfully wrote the optimized intrinsics and extended data.";
}
*/
void RunModeOptimization(std::string const&, bool) {
    //    if (stereo) {
    //        OptimizeStereo(in_path_extended);
    //    } else {
    //        OptimizeMono(in_path_extended);
    //    }
}
