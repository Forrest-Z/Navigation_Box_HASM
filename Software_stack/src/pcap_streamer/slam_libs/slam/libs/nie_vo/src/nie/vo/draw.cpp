/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "draw.hpp"
#include <glog/logging.h>

namespace nie {

namespace detail {

constexpr int kBits = 16;
constexpr int kType = CV_16U;

int GetBits(int CvType) {
    int result = kBits;
    switch (CvType) {
        case CV_8U:
        case CV_8S: {
            result = 8;
            break;
        }
        case CV_16U:
        case CV_16S: {
            result = 16;
            break;
        }
    }
    return result;
}
int MaxValue(int bits) { return int(std::pow(2, bits) - 1); }

}  // namespace detail

void Trajectory::Update(double x, double y) {
    points_.emplace_back(x, y);
    UpdateLimits(points_.back());
}

void Trajectory::UpdateLimits(cv::Point2d const& point) {
    limitX_.min = std::min(limitX_.min, point.x);
    limitX_.max = std::max(limitX_.max, point.x);
    limitY_.min = std::min(limitY_.min, point.y);
    limitY_.max = std::max(limitY_.max, point.y);
}

cv::Mat Trajectory::MakeImage(std::string const& fps) const {
    int plot_size = 800;  // Plotting window size [pixel]
    int max_range = static_cast<int>(std::ceil(std::max(limitX_.Range(), limitY_.Range())));
    double margin_rel = 0.05;                                          // Relative margin around plotting area
    int margin = static_cast<int>(std::ceil(plot_size * margin_rel));  // absolute [pixel]

    // Calculate the trajectory of points

    // Helper function
    auto ScreenPointFunc = [&](cv::Point2d const& p) {
        // Calculate relative position
        double x = (p.x - limitX_.min) / max_range;
        double y = (p.y - limitY_.min) / max_range;

        cv::Point2d q(margin + x * plot_size, margin + (1 - y) * plot_size);
        return q;
    };

    // Start drawing the trajectory
    cv::Point2d prevScreenLoc = ScreenPointFunc({0., 0.});
    cv::Mat trajectoryDisplay(plot_size + 2 * margin, plot_size + 2 * margin, CV_8UC1, cv::Scalar(255));
    for (cv::Point2d const& P : points_) {
        cv::Point2d screenLoc = ScreenPointFunc(P);

        cv::circle(trajectoryDisplay, screenLoc, 2, cv::Scalar(0), 1);
        cv::line(trajectoryDisplay, prevScreenLoc, screenLoc, cv::Scalar(0), 1, cv::LINE_AA);

        prevScreenLoc = screenLoc;
    }

    // Calculate heading
    cv::Point2d heading = *points_.rbegin();  // current point
    if (points_.size() > 1) {
        heading = heading - *(points_.rbegin() + 1);  // current point - previous point
    }

    cv::arrowedLine(
            trajectoryDisplay,
            prevScreenLoc,
            ScreenPointFunc(points_.back() + heading / cv::norm(heading) * max_range / 25),
            cv::Scalar(0, 255, 255),
            1,
            cv::LINE_AA,
            0,
            0.2);

    // Draw FPS
    if (!fps.empty()) {
        cv::putText(
                trajectoryDisplay,
                cv::String(fps),
                cv::Point(int(trajectoryDisplay.rows * 0.7), 30),
                cv::FONT_HERSHEY_DUPLEX,
                1.0,
                cv::Scalar(0, 255, 255));
    }

    return trajectoryDisplay;
}

cv::Mat MakeImageMatches(
        cv::Mat const& image,
        KeypointVector const& features_a,
        KeypointVector const& features_b,
        MatchVector const& matches,
        MatchVector const& matches_final,
        bool image_as_background) {
    int rangeX = image.cols;
    int rangeY = image.rows;

    constexpr double margin_rel = 0.01;  // Relative margin around plotting area
    int const margin = static_cast<int>(std::ceil(std::max(rangeX, rangeY) * margin_rel));  // absolute [pixel]
    cv::Point offset(margin, margin);

    cv::RNG rng(12345);
    constexpr float kDefaultColor = 175.;
    cv::Mat result;
    if (image_as_background) {
        result = ConvertImage(image);
        // To make the image brighter, scale all pixel values from the range
        // 0-255 to 100-255
        cv::convertScaleAbs(result, result, (255. - 100) / 255., 100);
        cv::copyMakeBorder(
                result,
                result,
                margin,
                margin,
                margin,
                margin,
                cv::BORDER_CONSTANT,
                cv::Scalar(kDefaultColor, kDefaultColor, kDefaultColor));
    } else {
        result =
                cv::Mat(rangeY + 2 * margin,
                        rangeX + 2 * margin,
                        CV_8UC3,
                        cv::Scalar(kDefaultColor, kDefaultColor, kDefaultColor));
    }

    // Make copy to be able to sort and efficiently search the filtered matches
    MatchVector matches_final_sorted = matches_final;
    std::sort(matches_final_sorted.begin(), matches_final_sorted.end());

    // First just print all matches that were filtered out
    for (const FeatureMatch& match : matches) {
        if (std::binary_search(matches_final_sorted.cbegin(), matches_final_sorted.cend(), match)) {
            continue;
        }

        cv::Point const& point_a = features_a[match.index_a];
        cv::Point const& point_b = features_b[match.index_b];

        cv::Scalar colour = cv::Scalar(0, 0, 0);
        cv::arrowedLine(result, offset + point_a, offset + point_b, colour, 1, cv::LINE_AA, 0, 0.2);
    }

    // Now only draw the actual matches (to have them on top of the filtered ones)
    for (FeatureMatch const& match : matches_final) {
        cv::Point const& point_a = features_a[match.index_a];
        cv::Point const& point_b = features_b[match.index_b];

        cv::Scalar colour(rng.uniform(127, 255), rng.uniform(127, 255), rng.uniform(127, 255));
        colour[rng.uniform(0, 2)] = 0;
        cv::arrowedLine(result, offset + point_a, offset + point_b, colour, 3, cv::LINE_AA, 0, 0.2);
    }

    return result;
}

cv::Mat MakeImageFeatures(
        cv::Mat const& image_a,
        KeypointVector const& features_a,
        cv::Mat const& image_b,
        KeypointVector const& features_b) {
    cv::Mat tmp_1 = nie::MakeImageFeatures(image_a, features_a);
    cv::Mat tmp_2 = nie::MakeImageFeatures(image_b, features_b);

    cv::Mat result;
    cv::hconcat(tmp_1, tmp_2, result);

    return result;
}

cv::Mat MakeImageFeatures(cv::Mat const& image, KeypointVector const& features) {
    cv::Mat result = ConvertImage(image);

    int max_value = detail::MaxValue(detail::GetBits(result.depth()));

    // Set up a random number generator (rng) with a fixed seed
    cv::RNG rng(12345);
    for (Keypoint const& feature : features) {
        // Generate a different colour for every different feature
        cv::Scalar colour(
                rng.uniform(max_value / 255 * 100, max_value),
                rng.uniform(max_value / 255 * 100, max_value),
                rng.uniform(max_value / 255 * 100, max_value));
        cv::circle(
                result,
                feature,
                /*radius*/ 8,
                colour,
                /*thickness*/ 2,
                /*lineType*/ 8);
    }

    return result;
}

cv::Mat MakeImageBackProjection(
        cv::Mat const& image, KeypointVector const& features, KeypointVector const& back_projected_features) {
    CHECK(features.size() == back_projected_features.size());

    cv::Mat result = MakeImageFeatures(image, features);

    cv::RNG rng(12345);
    for (std::size_t i = 0; i < features.size(); ++i) {
        Keypoint const& bpf = back_projected_features[i];

        cv::Scalar colour(rng.uniform(100, 255), rng.uniform(100, 255), rng.uniform(100, 255));
        cv::circle(result, bpf, /*radius*/ 3, colour, /*thickness*/ 2, /*lineType*/ 8);
        cv::line(result, features[i], bpf, colour, 1, cv::LINE_AA);
    }

    return result;
}

void DrawImage(std::string const& name, cv::Mat const& image, float scale, int sleep) {
    cv::Mat result;
    if (scale != 1.) {
        result = image.clone();
        cv::resize(image, result, cv::Size(), scale, scale);
    } else {
        result = image;
    }
    cv::imshow(name, result);
    cv::waitKey(sleep);
}

cv::Mat ConvertImage(cv::Mat const& image) {
    cv::Mat result;
    cv::cvtColor(image, result, cv::COLOR_GRAY2BGR);
    return result;
}

cv::Mat ConvertFilter(cv::Mat const& image) {
    double min_value, max_value;
    cv::minMaxLoc(image, &min_value, &max_value);

    cv::Mat result;
    double scale = ((double)detail::MaxValue(detail::kBits)) / (max_value - min_value);
    image.convertTo(result, detail::kType, scale, -min_value * scale);

    return result;
}

}  // namespace nie
