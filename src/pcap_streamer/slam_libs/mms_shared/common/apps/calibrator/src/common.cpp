/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "common.hpp"

#include <chrono>

#include <glog/logging.h>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <nie/core/work_pool.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

void WriteImage(std::unique_ptr<std::pair<std::string, cv::Mat>> const& work) {
    std::chrono::time_point<std::chrono::system_clock> before = std::chrono::system_clock::now();

    cv::imwrite(work->first, work->second);

    std::chrono::time_point<std::chrono::system_clock> after = std::chrono::system_clock::now();

    VLOG(3) << "Written image in ms: " << std::chrono::duration_cast<std::chrono::milliseconds>(after - before).count();
}

bool ReadImage(std::string const& in_path_image, std::pair<std::string, cv::Mat>* image) {
    image->first = in_path_image;
    image->second = cv::imread(in_path_image);

    return image->second.data;
}

bool DetectCorners(cv::Mat const& image, cv::Size const& pattern_size, std::vector<Eigen::Vector2f>* corners) {
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> c;

    if (cv::findChessboardCorners(gray, pattern_size, c, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_FAST_CHECK)) {
        cv::cornerSubPix(
                gray,
                c,
                cv::Size(11, 11),
                cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

        // It happens that the corners are ordered upside down depending on the
        // orientation of the pattern. This is extremely poorly documented by
        // opencv (not at all). Their reference is not described and with the
        // below code it doesn't matter anymore.
        if (c[0].x > c[c.size() - 1].x) {
            CHECK(c[0].y > c[c.size() - 1].y);
            std::reverse(c.begin(), c.end());
        }

        *corners = nie::ConvertPoints(c);

        return true;
    }

    return false;
}

void GetCornerData(
        std::string const& in_path_images,
        cv::Size const& pattern_size,
        bool write_corner_images,
        nie::mt::MtVector<ImageCorners>* p_ids_and_image_points) {
    nie::mt::MtVector<ImageCorners>& ids_and_image_points = *p_ids_and_image_points;

    {
        std::uint32_t processor_count_corners = CornerDetector::GetProcessorCountCorners();
        std::uint32_t processor_count_reader = CornerDetector::GetProcessorCountReader();

        nie::WorkPool<std::pair<std::string, cv::Mat>> pool_corner_detector(
                processor_count_corners,
                processor_count_corners * 2,
                [&write_corner_images, &pattern_size, &ids_and_image_points](
                        const std::unique_ptr<std::pair<std::string, cv::Mat>>& work) -> void {
                    std::chrono::time_point<std::chrono::system_clock> before = std::chrono::system_clock::now();

                    std::vector<Eigen::Vector2f> corners;

                    if (DetectCorners(work->second, pattern_size, &corners)) {
                        if (write_corner_images) {
                            cv::drawChessboardCorners(work->second, pattern_size, nie::ConvertPoints(corners), true);
                            cv::imwrite(work->first + "_c.png", work->second);
                        }

                        ids_and_image_points.PushBack(std::make_pair(work->first, std::move(corners)));

                        VLOG(3) << "Processed corners in ms: "
                                << std::chrono::duration_cast<std::chrono::milliseconds>(
                                           std::chrono::system_clock::now() - before)
                                           .count();
                    } else {
                        LOG(WARNING) << "Failed to detect corners: " << work->first;
                    }
                });

        nie::WorkPool<std::string> pool_image_reader(
                processor_count_reader,
                processor_count_reader * 2,
                [&pool_corner_detector](const std::unique_ptr<std::string>& work) -> void {
                    std::chrono::time_point<std::chrono::system_clock> before = std::chrono::system_clock::now();

                    std::unique_ptr<std::pair<std::string, cv::Mat>> out(new std::pair<std::string, cv::Mat>());

                    if (ReadImage(*work, &(*out))) {
                        VLOG(3) << "Read image in ms: "
                                << std::chrono::duration_cast<std::chrono::milliseconds>(
                                           std::chrono::system_clock::now() - before)
                                           .count();

                        pool_corner_detector.queue().BlockingPushBack(std::move(out));
                    } else {
                        LOG(WARNING) << "Unable to read: " << *work;
                    }
                });

        for (auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(in_path_images), {})) {
            if (boost::filesystem::is_regular_file(entry)) {
                pool_image_reader.queue().BlockingPushBack(std::make_unique<std::string>(entry.path().string()));
            }
        }
    }
}

void GetCornerData(
        std::string const& in_path_images,
        std::string const& path_relative_left,
        std::string const& path_relative_right,
        cv::Size const& pattern_size,
        bool write_corner_images,
        nie::mt::MtVector<std::pair<ImageCorners, ImageCorners>>* p_ids_and_image_points_pairs,
        nie::mt::MtVector<ImageCorners>* p_ids_and_image_points_left,
        nie::mt::MtVector<ImageCorners>* p_ids_and_image_points_right) {
    nie::mt::MtVector<std::pair<ImageCorners, ImageCorners>>& ids_and_image_points_pairs =
            *p_ids_and_image_points_pairs;
    nie::mt::MtVector<ImageCorners>& ids_and_image_points_left = *p_ids_and_image_points_left;
    nie::mt::MtVector<ImageCorners>& ids_and_image_points_right = *p_ids_and_image_points_right;

    {
        std::uint32_t processor_count_corners = CornerDetector::GetProcessorCountCorners();
        std::uint32_t processor_count_reader = CornerDetector::GetProcessorCountReader();

        using ImagePair = std::pair<std::pair<std::string, cv::Mat>, std::pair<std::string, cv::Mat>>;

        nie::WorkPool<ImagePair> pool_corner_detector(
                processor_count_corners,
                processor_count_corners * 2,
                [&write_corner_images,
                 &pattern_size,
                 &ids_and_image_points_pairs,
                 &ids_and_image_points_left,
                 &ids_and_image_points_right](const std::unique_ptr<ImagePair>& work) -> void {
                    std::chrono::time_point<std::chrono::system_clock> before = std::chrono::system_clock::now();

                    std::vector<Eigen::Vector2f> corners_left, corners_right;
                    bool found_corners_left = (work->first.second.data != nullptr) &&
                                              DetectCorners(work->first.second, pattern_size, &corners_left);
                    bool found_corners_right = (work->second.second.data != nullptr) &&
                                               DetectCorners(work->second.second, pattern_size, &corners_right);

                    if (write_corner_images) {
                        if (found_corners_left) {
                            cv::drawChessboardCorners(
                                    work->first.second, pattern_size, nie::ConvertPoints(corners_left), true);
                            cv::imwrite(work->first.first + "_c.png", work->first.second);
                        }

                        if (found_corners_right) {
                            cv::drawChessboardCorners(
                                    work->second.second, pattern_size, nie::ConvertPoints(corners_right), true);
                            cv::imwrite(work->second.first + "_c.png", work->second.second);
                        }
                    }

                    if (found_corners_left && found_corners_right) {
                        ids_and_image_points_pairs.PushBack(std::make_pair(
                                std::make_pair(work->first.first, std::move(corners_left)),
                                std::make_pair(work->second.first, std::move(corners_right))));
                        LOG(WARNING) << "Processed corners for pair: " << work->first.first << " "
                                     << work->second.first;
                    } else if (found_corners_left) {
                        ids_and_image_points_left.PushBack(std::make_pair(work->first.first, std::move(corners_left)));
                        VLOG(4) << "Processed left corners only: " << work->first.first;
                    } else if (found_corners_right) {
                        ids_and_image_points_right.PushBack(
                                std::make_pair(work->second.first, std::move(corners_right)));
                        VLOG(4) << "Processed right corners only: " << work->second.first;
                    } else {
                        LOG(WARNING) << "Failed to detect corners: " << work->first.first << " " << work->second.first;
                    }

                    if (found_corners_left || found_corners_right) {
                        VLOG(3) << "Processed corners in ms: "
                                << std::chrono::duration_cast<std::chrono::milliseconds>(
                                           std::chrono::system_clock::now() - before)
                                           .count();
                    }
                });

        nie::WorkPool<std::pair<std::string, std::string>> pool_image_reader(
                processor_count_reader,
                processor_count_reader * 2,
                [&pool_corner_detector](const std::unique_ptr<std::pair<std::string, std::string>>& work) -> void {
                    std::chrono::time_point<std::chrono::system_clock> before = std::chrono::system_clock::now();

                    std::unique_ptr<ImagePair> out(new ImagePair());

                    if (ReadImage(work->first, &out->first) && ReadImage(work->second, &out->second)) {
                        VLOG(3) << "Read image pair in ms: "
                                << std::chrono::duration_cast<std::chrono::milliseconds>(
                                           std::chrono::system_clock::now() - before)
                                           .count();

                        pool_corner_detector.queue().BlockingPushBack(std::move(out));
                    } else {
                        LOG(WARNING) << "Unable to read: " << work->first << " " << work->second;
                    }
                });

        for (auto& entry : boost::make_iterator_range(
                     boost::filesystem::directory_iterator(in_path_images + path_relative_left), {})) {
            boost::filesystem::path right(in_path_images + path_relative_right + entry.path().filename().string());

            if (boost::filesystem::is_regular_file(entry) && boost::filesystem::exists(right)) {
                pool_image_reader.queue().BlockingPushBack(std::make_unique<std::pair<std::string, std::string>>(
                        std::make_pair(entry.path().string(), right.string())));
            }
        }
    }
}

void MoveStereoInput(
        nie::mt::MtVector<std::pair<ImageCorners, ImageCorners>>* ids_and_image_points_pairs,
        nie::mt::MtVector<ImageCorners>* ids_and_image_points_left,
        nie::mt::MtVector<ImageCorners>* ids_and_image_points_right,
        std::vector<std::vector<Eigen::Vector2f>>* image_points_left,
        std::vector<std::vector<Eigen::Vector2f>>* image_points_right,
        std::vector<std::string>* ids_left,
        std::vector<std::string>* ids_right) {
    std::size_t pair_count = ids_and_image_points_pairs->size();
    std::size_t left_count = ids_and_image_points_left->size();
    std::size_t right_count = ids_and_image_points_right->size();

    image_points_left->resize(pair_count + left_count);
    image_points_right->resize(pair_count + right_count);
    ids_left->resize(pair_count + left_count);
    ids_right->resize(pair_count + right_count);

    for (std::size_t i = 0; i < pair_count; ++i) {
        (*image_points_left)[i] = std::move(ids_and_image_points_pairs->vector()[i].first.second);
        (*ids_left)[i] = std::move(ids_and_image_points_pairs->vector()[i].first.first);
        (*image_points_right)[i] = std::move(ids_and_image_points_pairs->vector()[i].second.second);
        (*ids_right)[i] = std::move(ids_and_image_points_pairs->vector()[i].second.first);
    }

    for (std::size_t i = 0; i < left_count; ++i) {
        std::size_t j = i + pair_count;
        (*image_points_left)[j] = std::move(ids_and_image_points_left->vector()[i].second);
        (*ids_left)[j] = std::move(ids_and_image_points_left->vector()[i].first);
    }

    for (std::size_t i = 0; i < right_count; ++i) {
        std::size_t j = i + pair_count;
        (*image_points_right)[j] = std::move(ids_and_image_points_right->vector()[i].second);
        (*ids_right)[j] = std::move(ids_and_image_points_right->vector()[i].first);
    }
}
