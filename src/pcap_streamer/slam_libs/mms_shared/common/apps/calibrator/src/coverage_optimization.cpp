/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "coverage_optimization.hpp"

#include <numeric>
#include <thread>
#include <unordered_map>

#include <glog/logging.h>

#include <boost/filesystem.hpp>
#include <opencv2/imgproc.hpp>

template <typename T>  // reuse this eventually for rotations
struct PixelCoverageMetadata {
    std::unordered_map<std::string, T> data;
};

void TryJoin(std::thread* thread) {
    if (thread->joinable()) thread->join();
}

float GetMeanVotes(std::unordered_map<std::string, std::size_t> const& removal_candidates, std::size_t max_candidates) {
    return std::accumulate(
               removal_candidates.begin(),
               removal_candidates.end(),
               std::size_t(0),
               [](size_t const& a, std::pair<std::string, std::size_t> const& b) -> std::size_t {
                   return a + b.second;
               }) /
           static_cast<float>(max_candidates);
}

PixelCoverageMetadata<cv::Rect> GetSpatialHeatmapForPixel(
    cv::Size const& pattern_size,
    cv::Point const& current_pixel,
    std::vector<std::string> const& image_ids,
    std::vector<std::vector<cv::Point2f>> const& image_points) {
    PixelCoverageMetadata<cv::Rect> coverage_heatmap = PixelCoverageMetadata<cv::Rect>();
    // because in our datasets the checkerboard covers roughly 10% of the image
    coverage_heatmap.data.reserve(image_ids.size() / 10);
    CHECK(image_points.size() == image_ids.size());
    for (size_t i = 0; i < image_points.size(); ++i) {
        // corners of the checkerboard parallelogram -- CCW
        std::vector<cv::Point> pattern_polygon{image_points[i][0],
                                               image_points[i][pattern_size.width * (pattern_size.height - 1)],
                                               image_points[i][(pattern_size.width * pattern_size.height) - 1],
                                               image_points[i][pattern_size.width - 1]};

        auto polygon_bbox = cv::boundingRect(pattern_polygon);

        if (polygon_bbox.contains(current_pixel)) {
            if (cv::pointPolygonTest(pattern_polygon, current_pixel, false) >= 0) {
                coverage_heatmap.data[image_ids[i]] = polygon_bbox;
            }
        }
    }

    return coverage_heatmap;
}

void UpdateCandidatesWithHeatmap(
    cv::Point const& pixel,
    PixelCoverageMetadata<cv::Rect> const& metadata,
    std::unordered_map<std::string, std::size_t>* removal_candidates) {
    // for each pixel, establish which of the checkerboards that see it should
    // be filtered out -- for now this is based on the pixel's proximity to the
    // center of the checkerboard. If we remove the checkerboard which is the
    // most 'centered' we are impacting the neighborhood of the pixel more
    // uniformly instead of, for example, affecting a lot neighbors on the left
    // and none on the right, which may introduce additional bias.
    if (metadata.data.size() > 1) {
        auto removal_candidate = min_element(
            metadata.data.begin(),
            metadata.data.end(),
            [&metadata, &pixel](
                std::pair<std::string, cv::Rect> const& lhs, std::pair<std::string, cv::Rect> const& rhs) -> bool {
                cv::Point center_lhs{lhs.second.x + lhs.second.width / 2, lhs.second.y + lhs.second.height / 2};
                cv::Point center_rhs{rhs.second.x + rhs.second.width / 2, rhs.second.y + rhs.second.height / 2};

                return norm(pixel - center_lhs) < norm(pixel - center_rhs);
            });

        removal_candidates->count((*removal_candidate).first) == 0
            ? ((*removal_candidates)[(*removal_candidate).first] = 1)
            : ((*removal_candidates)[(*removal_candidate).first] += 1);
    }
}

void FilterCandidates(float mean_votes, std::unordered_map<std::string, std::size_t>* removal_candidates) {
    // used to get the final list of images which will be removed
    // from the dataset (see UpdateCandidatesWithHeatmap and GetMeanVotes)
    float constexpr kClippingFactor = 1.5f;

    // pruning candidates -- doing this with a range loop blows up. this
    // version, however convoluted works fine because C++ iterators are the
    // greatest (not)
    auto ite_candidates = removal_candidates->begin();
    while (ite_candidates != removal_candidates->end()) {
        if (ite_candidates->second < static_cast<size_t>(kClippingFactor * mean_votes)) {
            // actually returns a new iterator after the
            // removed element, thus we must not increase
            // the iterator or else it all blows up
            ite_candidates = removal_candidates->erase(ite_candidates);

            continue;
        }
        ++ite_candidates;
    }
}

// calculate the coverage of each pixel for all images and output the
// initial candidates for removal
void CalculatePixelCoverage(
    cv::Size const& image_size,
    cv::Size const& pattern_size,
    std::vector<std::string> const& image_ids,
    std::vector<std::vector<cv::Point2f>> const& image_points,
    std::unordered_map<std::string, std::size_t>* removal_candidates) {
    // establish which checkerboards have seen each pixel
    for (int i = 0; i < image_size.height; ++i) {
        for (int j = 0; j < image_size.width; ++j) {
            cv::Point curr_pixel = cv::Point(j, i);
            PixelCoverageMetadata<cv::Rect> curr_metadata_right;
            curr_metadata_right = GetSpatialHeatmapForPixel(pattern_size, curr_pixel, image_ids, image_points);
            if (!curr_metadata_right.data.empty()) {
                UpdateCandidatesWithHeatmap(curr_pixel, curr_metadata_right, removal_candidates);
            }
        }
    }
}

void OptimizeSpatialCoverage(
    cv::Size const& image_size,
    cv::Size const& pattern_size,
    std::vector<std::string>* image_ids,
    std::vector<std::vector<cv::Point2f>>* image_points) {
    // we assume that the inputs are matched (in the same order) and thus avoid a lot
    // of funny business down the road
    CHECK(image_ids->size() == image_points->size());

    VLOG(3) << "Optimizing Spatial Coverage... (Mono)";
    VLOG(3) << "Spatial Coverage Heatmap Initialized (Mono)";

    std::unordered_map<std::string, std::size_t> removal_candidates;
    removal_candidates.reserve(image_ids->size());

    CalculatePixelCoverage(image_size, pattern_size, *image_ids, *image_points, &removal_candidates);

    float mean_votes = GetMeanVotes(removal_candidates, image_ids->size());

    VLOG(3) << "Mean Votes = " << mean_votes << " -- Candidates = " << removal_candidates.size()
            << " candidates (Mono)";

    FilterCandidates(mean_votes, &removal_candidates);

    // doing the actual filtering in a way that keeps image_ids and points balanced
    for (auto const& candidate : removal_candidates) {
        if (image_ids->size() <= 3) {
            break;
        }

        auto ite_candidate = std::find(image_ids->begin(), image_ids->end(), candidate.first);

        long index = std::distance(image_ids->begin(), ite_candidate);

        image_ids->erase(image_ids->begin() + index);
        image_points->erase(image_points->begin() + index);
    }

    VLOG(3) << image_ids->size() << " images left after spatial optimization (Mono)";
}

void OptimizeSpatialCoverage(
    cv::Size const& image_size,
    cv::Size const& pattern_size,
    std::size_t pair_count,
    std::vector<std::string>* image_ids_left,
    std::vector<std::string>* image_ids_right,
    std::vector<std::vector<cv::Point2f>>* image_points_left,
    std::vector<std::vector<cv::Point2f>>* image_points_right,
    std::size_t* optimized_pair_count) {
    // if you're maintaining this, its a good idea to check the comments on the
    // mono version and the called functions, which explain several tidbits
    // and things things done for the sake of space and time efficiency
    CHECK(image_ids_left->size() == image_points_left->size());
    CHECK(image_ids_right->size() == image_points_right->size());

    VLOG(3) << "Optimizing Spatial Coverage... (Pairs)";

    std::unordered_map<std::string, std::size_t> removal_candidates, removal_candidates_right;
    if (pair_count > 0) {
        removal_candidates.reserve(pair_count * 2);
        removal_candidates_right.reserve(pair_count);

        // want to make sure these live no longer than strictly necessary
        // doing this uses about 50% less memory (200mb vs 400mb) for a 200
        // image subset of our old stereo dataset
        std::vector<std::string>* image_ids_left_paired =
            new std::vector<std::string>(image_ids_left->begin(), image_ids_left->begin() + pair_count);
        std::vector<std::string>* image_ids_right_paired =
            new std::vector<std::string>(image_ids_right->begin(), image_ids_right->begin() + pair_count);
        std::vector<std::vector<cv::Point2f>>* image_points_left_paired = new std::vector<std::vector<cv::Point2f>>(
            image_points_left->begin(), image_points_left->begin() + pair_count);
        std::vector<std::vector<cv::Point2f>>* image_points_right_paired = new std::vector<std::vector<cv::Point2f>>(
            image_points_right->begin(), image_points_right->begin() + pair_count);

        // threaded context to ensure these destructors are called and this
        // memory is freed -- saves an extra 40-ish megabytes for the same
        // dataset as above
        {
            std::thread update_left(
                CalculatePixelCoverage,
                image_size,
                pattern_size,
                *image_ids_left_paired,
                *image_points_left_paired,
                &removal_candidates);
            std::thread update_right(
                CalculatePixelCoverage,
                image_size,
                pattern_size,
                *image_ids_right_paired,
                *image_points_right_paired,
                &removal_candidates_right);
            TryJoin(&update_left);
            TryJoin(&update_right);
        }

        removal_candidates.insert(
            std::make_move_iterator(removal_candidates_right.begin()),
            std::make_move_iterator(removal_candidates_right.end()));

        delete image_ids_left_paired;
        delete image_ids_right_paired;
        delete image_points_left_paired;
        delete image_points_right_paired;
    }

    float mean_votes = GetMeanVotes(removal_candidates, pair_count * 2);

    VLOG(3) << "Mean Votes = " << mean_votes << " -- Candidates = " << removal_candidates.size() / 2
            << " candidates (Pairs)";

    FilterCandidates(mean_votes, &removal_candidates);

    std::vector<std::pair<std::string, std::string>> paired_image_ids;
    std::vector<std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>>> paired_image_points;
    for (size_t i = 0; i < pair_count; ++i) {
        paired_image_ids.emplace_back(std::move((*image_ids_left)[i]), std::move((*image_ids_right)[i]));
        paired_image_points.emplace_back(std::move((*image_points_left)[i]), std::move((*image_points_right)[i]));
    }

    for (auto const& candidate : removal_candidates) {
        // better be on the safe side and not break Zhang estimation --  we may
        // end up with a little more than the optimum amount, but this should not
        // matter that much except for very small datasets
        if (paired_image_ids.size() <= 3) {
            break;
        }

        auto ite_candidate =
            std::find_if(paired_image_ids.begin(), paired_image_ids.end(), [&candidate](auto const& id) -> bool {
                return id.first == candidate.first || id.second == candidate.first;
            });

        long index = std::distance(paired_image_ids.begin(), ite_candidate);

        paired_image_ids.erase(paired_image_ids.begin() + index);
        paired_image_points.erase(paired_image_points.begin() + index);
    }

    *optimized_pair_count = paired_image_ids.size();

    // extract and optimize the left_only and right_only bits
    // then put it all back together

    std::vector<std::string> image_ids_left_only(
        std::make_move_iterator(image_ids_left->begin()) + pair_count, std::make_move_iterator(image_ids_left->end()));
    std::vector<std::string> image_ids_right_only(
        std::make_move_iterator(image_ids_right->begin()) + pair_count,
        std::make_move_iterator(image_ids_right->end()));

    std::vector<std::vector<cv::Point2f>> image_points_left_only(
        std::make_move_iterator(image_points_left->begin()) + pair_count,
        std::make_move_iterator(image_points_left->end()));
    std::vector<std::vector<cv::Point2f>> image_points_right_only(
        std::make_move_iterator(image_points_right->begin()) + pair_count,
        std::make_move_iterator(image_points_right->end()));

    {
        using OptimizationSignature = void (*)(
            cv::Size const&, cv::Size const&, std::vector<std::string>*, std::vector<std::vector<cv::Point2f>>*);
        std::thread optimize_left, optimize_right;
        if (!image_ids_left_only.empty()) {
            optimize_left = std::thread(
                static_cast<OptimizationSignature>(OptimizeSpatialCoverage),
                image_size,
                pattern_size,
                &image_ids_left_only,
                &image_points_left_only);
        }
        if (!image_ids_right_only.empty()) {
            optimize_right = std::thread(
                static_cast<OptimizationSignature>(OptimizeSpatialCoverage),
                image_size,
                pattern_size,
                &image_ids_right_only,
                &image_points_right_only);
        }
        if (image_ids_left_only.size() > image_ids_right_only.size()) {
            TryJoin(&optimize_right);
            TryJoin(&optimize_left);
        } else {
            TryJoin(&optimize_left);
            TryJoin(&optimize_right);
        }
    }

    image_ids_left->clear();
    image_ids_right->clear();
    image_points_left->clear();
    image_points_right->clear();

    for (size_t i = 0; i < paired_image_ids.size(); ++i) {
        std::pair<std::string, std::string> id_pair = std::move(paired_image_ids[i]);
        image_ids_left->push_back(std::move(id_pair.first));
        image_ids_right->push_back(std::move(id_pair.second));

        std::pair<std::vector<cv::Point2f>, std::vector<cv::Point2f>> point_pair = std::move(paired_image_points[i]);
        image_points_left->push_back(std::move(point_pair.first));
        image_points_right->push_back(std::move(point_pair.second));
    }

    image_ids_left->insert(
        image_ids_left->end(),
        std::make_move_iterator(image_ids_left_only.begin()),
        std::make_move_iterator(image_ids_left_only.end()));
    image_ids_right->insert(
        image_ids_right->end(),
        std::make_move_iterator(image_ids_right_only.begin()),
        std::make_move_iterator(image_ids_right_only.end()));

    image_points_left->insert(
        image_points_left->end(),
        std::make_move_iterator(image_points_left_only.begin()),
        std::make_move_iterator(image_points_left_only.end()));
    image_points_right->insert(
        image_points_right->end(),
        std::make_move_iterator(image_points_right_only.begin()),
        std::make_move_iterator(image_points_right_only.end()));

    VLOG(3) << paired_image_ids.size() << " images left after spatial optimization (Pairs)";
}