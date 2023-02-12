/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <unordered_set>

#include <nie/lidar/cloud.hpp>
#include <opencv2/opencv.hpp>

#include "drawing_color_handlers.hpp"
#include "pcl_viewer.hpp"
#include "point_cloud_color_handler_intensity.hpp"

namespace nie {

namespace detail {

auto kCloudPrefix = "cloud_";
auto kNormalsPrefix = "normals_";
auto kTracePrefix = "trace_";

}  // namespace detail

std::vector<std::string> GenerateNames(std::string const& prefix, int range);

template <typename PointT>
void AddClouds(
        std::vector<Cloud<PointT>> const& clouds,
        PointCloudColorHandlerPtr<PointT> const& handler,
        std::vector<std::string> const& names,
        nie::PclViewer* viewer);
// TODO: normals are not "switched", names will also have to be provided

template <typename PointT>
void AddClouds(
        Eigen::Vector3d const& origin,
        std::vector<Cloud<PointT>> const& clouds,
        std::vector<PointCloudColorHandlerPtr<PointT>> const& handlers,
        std::vector<std::string> const& names,
        nie::PclViewer* viewer);

template <typename PointT>
void AddClouds(
        std::vector<Cloud<PointT>> const& clouds,
        std::vector<PointCloudColorHandlerPtr<PointT>> const& handlers,
        std::vector<std::string> const& names,
        nie::PclViewer* viewer);

template <typename PointT, typename Iterator, typename CloudGetter>
void AddClouds(
        Iterator begin,
        Iterator end,
        PointCloudColorHandlerPtr<PointT> const& color_handler,
        nie::PclViewer* viewer,
        CloudGetter cloud_getter);

void AddCoordinateSystem(
        Eigen::Vector3d const& origin,
        std::vector<PoseBbox> const& bounds_list,
        pcl::visualization::PCLVisualizer* viewer);

template <typename Iterator, typename CloudGetter>
void AddCoordinateSystem(Iterator begin, Iterator end, nie::PclViewer* viewer, CloudGetter cloud_getter);

template <typename PointT>
void AddCoordinateSystem(std::vector<Cloud<PointT>> const& clouds, nie::PclViewer* viewer);

class CoordinateConverter {
public:
    using ImageCoord = cv::Point2i;

    CoordinateConverter(nie::PoseBbox bounds, cv::Size2i image_size, float margin)
        : bounds_(std::move(bounds)),
          image_size_(std::move(image_size)),
          margin_(margin),
          scale_(CalculateScale(bounds_.bbox().Range(), image_size_, margin_)) {}

    [[nodiscard]] cv::Size2i GetImageSize() const { return image_size_; };

    template <typename Derived>
    [[nodiscard]] ImageCoord GlobalToImage(Eigen::MatrixBase<Derived> const& p) const {
        // Sadly, posebox now only supports floats, so we cast every time.
        // The box we are drawing does not support an orientation.
        Eigen::Vector3d local = p.template cast<double>() - bounds_.origin().translation() - bounds_.bbox().min.cast<double>();
        return LocalToImage(local);
    }

    [[nodiscard]] std::array<ImageCoord, 4> GlobalToImage(nie::PoseBbox const& b) const {
        auto poly3d = b.GroundPlanePoly();
        std::array<ImageCoord, 4> poly2d;

        for (std::size_t i = 0; i < 4; ++i) {
            poly2d[i] = GlobalToImage(poly3d[i]);
        }

        return poly2d;
    }

    [[nodiscard]] nie::PoseBbox ImageToBounds(ImageCoord const& min, ImageCoord const& max) const {
        Bboxf box{};
        box.min = ImageToLocal(min);
        box.max = ImageToLocal(max);
        Isometry3qd origin = Isometry3qd::FromTranslation(bounds_.origin().translation());
        origin.translation() += bounds_.bbox().min.cast<double>();
        return {std::move(origin), std::move(box)};
    }

private:
    template <typename Derived>
    [[nodiscard]] ImageCoord LocalToImage(Eigen::MatrixBase<Derived> const& p) const {
        Eigen::Vector2f uf = p.template head<2>().template cast<float>() * scale_ - Eigen::Vector2f::Constant(0.5f);
        Eigen::Vector2i ui = uf.array().round().matrix().cast<int>() + Eigen::Vector2i::Constant(margin_);

        return { ui.x(), image_size_.height - ui.y() - 1};
    }

    [[nodiscard]] Eigen::Vector3f ImageToLocal(ImageCoord const& p) const {
        float x = (static_cast<float>(p.x) - margin_ + .5) / scale_;
        float y = (static_cast<float>(image_size_.height - p.y + 1) - margin_ + .5) / scale_;
        return {x, y, 0.};
    }

    static float CalculateScale(Eigen::Vector3f const& range, cv::Size2i const& image_size, float const margin) {
        float const width = static_cast<float>(image_size.width) - 2. * margin;
        float const height = static_cast<float>(image_size.height) - 2. * margin;
        float const range_x = static_cast<float>(range.x());
        float const range_y = static_cast<float>(range.y());
        return ((range_y / range_x) > (height / width)) ? height / range_y : width / range_x;
    }

    nie::PoseBbox const bounds_;
    cv::Size2i const image_size_;
    float const margin_;
    float const scale_;
};

void AddBboxesToMap(
        std::vector<PoseBbox> const& bounds_map,
        std::unordered_set<std::size_t> const& selection,
        std::unordered_set<std::size_t> const& highlighted,
        CoordinateConverter converter,
        cv::Mat* out);

void DrawOrientedBounds(
        Eigen::Vector3d const& origin,
        std::vector<PoseBbox> const& bounds,
        pcl::visualization::PCLVisualizer* viewer,
        std::string const& name_prefix = "OBB_");

}  // namespace nie

#include "drawing.inl"
