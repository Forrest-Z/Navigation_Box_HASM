/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <atomic>
#include <thread>
#include <unordered_set>

#include <nie/core/geometry/covariance.hpp>
#include <nie/formats/ba_graph/pose_collection.hpp>
#include <opencv2/core.hpp>

#include "color_conversion.hpp"
#include "drawing.hpp"

namespace nie {

class MapViewer {
private:
    struct DrawingParameters {
        cv::Size drawing_size{};
        float area_ratio{};
        bool draw_score{};
        double min_score{};
        double max_score{};
    };

public:
    MapViewer(
            std::string window_name,
            std::vector<PoseBbox> const& bounds,
            io::PoseCollection const& trace,
            std::unordered_set<std::size_t> const& selection = {})
        : running_{false},
          thread_{},
          bounds_{bounds},
          total_bounds_{GetExtent(bounds_.cbegin(), bounds_.cend(), bounds_.front().origin().translation())},
          window_name_{std::move(window_name)},
          map_{},
          selection_{selection.empty() ? CreateAllIndices(bounds_.size()) : selection},
          highlighted_{},
          drawing_parameters_{InitializeDrawingParameters(total_bounds_)},
          trace_{ProcessTrace(trace.header, trace.poses, &drawing_parameters_)},
          selecting_(false),
          selection_box_{},
          map_with_selection_box_{},
          callback_{} {}

    ~MapViewer() {
        running_ = false;

        if (thread_.joinable()) {
            thread_.join();
        }
    }

    using Callback = std::function<void(PoseBbox const&)>;
    void View(Callback const& callback = {}) {
        if (callback) {
            callback_ = callback;
        }

        running_ = true;
        thread_ = std::thread(&MapViewer::Run, this);
    }

    void Update(std::unordered_set<std::size_t> highlighted) {
        highlighted_ = std::move(highlighted);
        DrawMap();
    }

    bool Running() const { return running_; }

private:
    template <typename Iterator>
    void Permutate(
        std::size_t d, 
        std::size_t end_d, 
        std::array<Eigen::Vector3d, 2> const& in,
        Eigen::Vector3d* permutation,
        Iterator* a) {
        if (d == end_d) {
            **a = *permutation;
            (*a)++;
            return;
        }

        for (std::size_t p = 0; p < 2; ++p) {
            (*permutation)(d) = in[p](d);
            Permutate(d + 1, end_d, in, permutation, a);
        }
    }

    auto Corners(nie::PoseBbox const& b) {
        std::array<Eigen::Vector3d, 2> mm{ b.bbox().min.cast<double>(), b.bbox().max.cast<double>() };
        std::array<Eigen::Vector3d, 8> cs;
        Eigen::Vector3d permutation;
        auto it = cs.data();

        Permutate(0, 3, mm, &permutation, &it);

        for (auto& c : cs) {
            c = b.origin() * c;
        }

        return cs;
    }

    template <typename Iterator>
    nie::PoseBbox GetExtent(Iterator begin, Iterator end, Eigen::Vector3d const& reference) {
        nie::Bboxf bbox = nie::Bboxf::InverseMaxBoundingBox();

        for(Iterator it = begin; it != end; ++it) {
             auto cs = Corners(*it);

            for (auto const& c : cs) {
                Eigen::Vector3d v = c - reference;
                bbox.FitPoint(v.cast<float>());
            }
        }

        return nie::PoseBbox(reference, bbox);
    }

    static std::unordered_set<std::size_t> CreateAllIndices(std::size_t size) {
        std::vector<std::size_t> indices(size);
        std::iota(indices.begin(), indices.end(), 0);
        return {indices.cbegin(), indices.cend()};
    }

    static DrawingParameters InitializeDrawingParameters(nie::PoseBbox const& total_bounds) {
        DrawingParameters result;
        auto const range = total_bounds.bbox().Range();
        result.drawing_size.width = static_cast<int>(range.x());
        result.drawing_size.height = static_cast<int>(range.y());
        result.area_ratio = range.y() / range.x();
        result.draw_score = false;
        return result;
    }

    static std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3d>> ProcessTrace(
            io::PoseHeader const& header,
            std::vector<nie::io::PoseRecord> const& trace,
            DrawingParameters* parameters) {
        std::vector<double> scores;
        if (header.Has(nie::io::PoseHeader::kHasPoseInformationPerRecord)) {
            scores.reserve(trace.size());
            for (auto const& p : trace) {
                if (p.information != Eigen::Matrix<double, 6, 6>::Zero()) {
                    Eigen::Matrix<double, 6, 6> covariance;
                    InformationToCovariance(p.information, &covariance);
                    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> s{covariance.block<3, 3>(0, 0)};
                    scores.emplace_back(ScoreFromEigenValues(s.eigenvalues().asDiagonal().toDenseMatrix()));
                }
            }

            auto min_max = std::minmax_element(scores.cbegin(), scores.cend());
            parameters->min_score = *min_max.first;
            parameters->max_score = *min_max.second;
            parameters->draw_score = parameters->max_score != parameters->min_score;
        }

        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3d>> result(trace.size());
        Eigen::Vector3d colour(0., 0., 255.);
        for (std::size_t i = 0; i < trace.size(); ++i) {
            if (parameters->draw_score) {
                auto const colour_point = nie::MakePointXYZRGB(
                        HueFromStddev(scores[i], parameters->min_score, parameters->max_score), 1.0f);
                colour = {
                        static_cast<double>(colour_point.b),
                        static_cast<double>(colour_point.g),
                        static_cast<double>(colour_point.r)};
            }
            result[i] = std::make_pair(trace[i].isometry.translation().cast<float>(), colour);
        }
        return result;
    }

    template <typename Derived>
    static inline double ScoreFromEigenValues(Eigen::MatrixBase<Derived> const& m) {
        return m.diagonal().template head<3>().sum();
    }

    static inline float HueFromStddev(float stddev, float min_stddev, float max_stddev) {
        float d = max_stddev - min_stddev;

        if (d == 0.0) {
            CHECK(stddev - min_stddev == 0.0);
            return 0.0;
        }

        float r = (stddev - min_stddev) / d;
        CHECK(r >= 0.0f && r <= 1.0f);
        return (1.0f - r) * 120.0f;
    }

    void Run() {
        cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
        if (callback_) {
            cv::setMouseCallback(window_name_, MouseCallback, this);
        }

        // Initialize map
        DrawMap();
        while (running_) {
            cv::imshow(window_name_, selecting_ ? map_with_selection_box_ : map_);
            int const key = cv::waitKey(100);
            if (key == 113) {  // 113 == 'q' in ascii
                running_ = false;
                break;
            }

            cv::Rect current_rect = cv::getWindowImageRect(window_name_);
            if (selecting_) {
                AddSelectionBoxToMap();
            } else if (drawing_parameters_.drawing_size != current_rect.size()) {
                auto const height = static_cast<float>(current_rect.height);
                auto const width = static_cast<float>(current_rect.width);
                if (std::abs(height / width - drawing_parameters_.area_ratio) >
                    drawing_parameters_.area_ratio * 0.01f) {
                    current_rect.height = static_cast<int>(std::ceil(width * drawing_parameters_.area_ratio));
                }

                drawing_parameters_.drawing_size = current_rect.size();
                DrawMap();
            }
        }

        cv::setMouseCallback(window_name_, nullptr, nullptr);
        cv::destroyWindow(window_name_);
    }

    void DrawMap() {
        CoordinateConverter converter(total_bounds_, drawing_parameters_.drawing_size, 2);

        // Init map, same colour as pcl_viewer background
        auto const& size = converter.GetImageSize();
        map_ = cv::Mat(size.height, size.width, CV_8UC3, cv::Scalar(51, 51, 51));

        AddTraceToMap(trace_, converter, &map_);
        AddBboxesToMap(bounds_, selection_, highlighted_, converter, &map_);
        // Debugging to see if the map makes sense.
        //AddBboxesToMap({total_bounds_}, {}, {}, converter, &map_);
    }

    static void AddTraceToMap(
            std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3d>> const& trace,
            CoordinateConverter const& converter,
            cv::Mat* out) {
        std::for_each(trace.cbegin(), trace.cend(), [&converter, &out](auto const& pair) {
            Eigen::Vector3f const& p = pair.first;
            Eigen::Vector3d const& c = pair.second;
            cv::circle(*out, converter.GlobalToImage(p), 1, {c.x(), c.y(), c.z()}, 0);
        });
    }

    void AddSelectionBoxToMap() {
        map_with_selection_box_ = map_.clone();
        cv::rectangle(map_with_selection_box_, selection_box_.first, selection_box_.second, cv::Scalar{255, 0, 0});
    }

    static void MouseCallback(int event, int x, int y, int /*flags*/, void* data) {
        MapViewer& instance = *(reinterpret_cast<MapViewer*>(data));
        cv::Point2i& min = instance.selection_box_.first;
        cv::Point2i& max = instance.selection_box_.second;
        if (event == cv::EVENT_LBUTTONDOWN) {
            instance.selecting_ = true;
            min = {x, y};
            max = {x, y};
        } else if (instance.selecting_ && event == cv::EVENT_MOUSEMOVE) {
            max = {x, y};
        } else if (event == cv::EVENT_LBUTTONUP) {
            instance.selecting_ = false;
            max = {x, y};

            instance.DoCallback();
        }
    }

    void DoCallback() const {
        cv::Point2i const& a = selection_box_.first;
        cv::Point2i const& b = selection_box_.second;

        // The min and max points stored in the selection_box_ are the opencv image coordinate with the origin in the
        // top left, positive x/y direction begin in the right/down direction. The cloud bounds are given as the lower
        // left and upper right corner. Hence left = min(x), lower -> max(y), etc.
        cv::Point2i lower_left, upper_right;
        lower_left.x = std::min(a.x, b.x);
        lower_left.y = std::max(a.y, b.y);
        upper_right.x = std::max(a.x, b.x);
        upper_right.y = std::min(a.y, b.y);

        CoordinateConverter converter(total_bounds_, drawing_parameters_.drawing_size, 2);
        callback_(converter.ImageToBounds(lower_left, upper_right));
    }

    std::atomic_bool running_;
    std::thread thread_;

    std::vector<PoseBbox> const bounds_;
    PoseBbox const total_bounds_;

    std::string const window_name_;
    cv::Mat map_;
    std::unordered_set<std::size_t> const selection_;
    std::unordered_set<std::size_t> highlighted_;
    DrawingParameters drawing_parameters_;
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3d>> const trace_;

    // Variables required for handling the bounding box selection
    bool selecting_;
    std::pair<cv::Point2i, cv::Point2i> selection_box_;
    cv::Mat map_with_selection_box_;
    Callback callback_;
};

}  // namespace nie
