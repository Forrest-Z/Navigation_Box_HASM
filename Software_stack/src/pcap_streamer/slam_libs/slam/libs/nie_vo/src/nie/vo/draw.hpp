/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#ifndef NIE_VO_DRAW_HPP
#define NIE_VO_DRAW_HPP

#include <vector>

#include <opencv2/opencv.hpp>

#include <nie/core/geometry/isometry3.hpp>
#include <nie/cv/vo/feature/keypoint.hpp>
#include <nie/cv/vo/feature/match.hpp>

namespace nie {

class Trajectory {
public:
    Trajectory() : points_(), limitX_(0., 0.), limitY_(0., 0.) {}

    void Update(double x, double y);

    cv::Mat MakeImage(std::string const& fps = "") const;

private:
    void UpdateLimits(cv::Point2d const& point);

    std::vector<cv::Point2d> points_;

    struct limit {
    public:
        double min;
        double max;

        limit(double _min, double _max) : min(_min), max(_max) {}
        double Range() const { return max - min; }
    };
    limit limitX_;
    limit limitY_;
};

cv::Mat MakeImageMatches(
    cv::Mat const& image,
    KeypointVector const& features_a,
    KeypointVector const& features_b,
    MatchVector const& matches,
    MatchVector const& matches_final,
    bool image_as_background = true);

cv::Mat MakeImageFeatures(
    cv::Mat const& image_a, KeypointVector const& features_a, cv::Mat const& image_b, KeypointVector const& features_b);

cv::Mat MakeImageFeatures(cv::Mat const& image, KeypointVector const& features);

cv::Mat MakeImageBackProjection(
    cv::Mat const& image, KeypointVector const& features, KeypointVector const& back_projected_features);

/*
 * sleep in [millisecond]
 */
void DrawImage(std::string const& name, cv::Mat const& image, float scale = 1., int sleep = 1);

cv::Mat ConvertImage(cv::Mat const& image);

cv::Mat ConvertFilter(cv::Mat const& image);

}  // namespace nie

#endif  // NIE_VO_DRAW_HPP
