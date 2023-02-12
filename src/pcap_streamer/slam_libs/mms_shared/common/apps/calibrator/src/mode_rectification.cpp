/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "mode_rectification.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include <glog/logging.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>
#include <nie/core/geometry/conversion.hpp>
#include <nie/core/work_pool.hpp>
#include <nie/cv/calib3d/distortion_model.hpp>
#include <nie/cv/calib3d/distortion_model_factory.hpp>
#include <nie/cv/transform/gpu_image_warper.hpp>
#include <nie/cv/transform/transform.hpp>
#include <nie/formats/calib3d/calibrated_mono_parameters.hpp>
#include <nie/formats/calib3d/calibrated_stereo_parameters.hpp>
#include <nie/formats/calib3d/rectified_parameters.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include "common.hpp"

// TODO(JBr): Move cleaner version of rectification parameter code to nie_cv
bool RemapCoordinateAtoB(
        std::unique_ptr<nie::DistortionModel> const& model,
        cv::Size const& size,
        Eigen::Matrix3d const& K_out,
        Eigen::Vector2f const& p_u,
        Eigen::Vector2f* p_d) {
    model->Distort(K_out, p_u, p_d);

    // OpenCV uses the pixel indices for sub-pixel coordinates. The center of the top left pixel equals 0.0, 0.0 in
    // opencv.
    const Eigen::Vector2f opencv_coordinate_system_offset(0.5F, 0.5F);

    *p_d += opencv_coordinate_system_offset;

    return p_d->x() >= 0.0 && p_d->x() <= size.width && p_d->y() >= 0.0 && p_d->y() <= size.height;
}

class BboxAllPixels {
public:
    BboxAllPixels()
        : min_(std::numeric_limits<double>::max(), std::numeric_limits<double>::max()),
          max_(-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()) {}

    void UpdateMinX(Eigen::Vector2f const& u) { min_.x() = std::min(u.x(), min_.x()); }

    void UpdateMaxX(Eigen::Vector2f const& u) { max_.x() = std::max(u.x(), max_.x()); }

    void UpdateMinY(Eigen::Vector2f const& u) { min_.y() = std::min(u.y(), min_.y()); }

    void UpdateMaxY(Eigen::Vector2f const& u) { max_.y() = std::max(u.y(), max_.y()); }

    // huhuh
    cv::Rect GetRect() {
        return cv::Rect(
                std::floor(min_.x()),
                std::floor(min_.y()),
                std::ceil(max_.x()) - std::floor(min_.x()),
                std::ceil(max_.y()) - std::floor(min_.y()));
    }

    static cv::Rect GetMergedBbox(cv::Rect const& left, cv::Rect const& right) { return left | right; }

    static bool UseScaleX(double const scale_x, double const scale_y) { return scale_x < scale_y; }

private:
    Eigen::Vector2f min_;
    Eigen::Vector2f max_;
};

class BboxNoBlackPixels {
public:
    BboxNoBlackPixels()
        : min_(-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()),
          max_(std::numeric_limits<double>::max(), std::numeric_limits<double>::max()) {}

    void UpdateMinX(Eigen::Vector2f const& u) { min_.x() = std::max(u.x(), min_.x()); }

    void UpdateMaxX(Eigen::Vector2f const& u) { max_.x() = std::min(u.x(), max_.x()); }

    void UpdateMinY(Eigen::Vector2f const& u) { min_.y() = std::max(u.y(), min_.y()); }

    void UpdateMaxY(Eigen::Vector2f const& u) { max_.y() = std::min(u.y(), max_.y()); }

    cv::Rect GetRect() {
        return cv::Rect(
                std::ceil(min_.x()),
                std::ceil(min_.y()),
                std::floor(max_.x()) - std::ceil(min_.x()),
                std::floor(max_.y()) - std::ceil(min_.y()));
    }

    static cv::Rect GetMergedBbox(cv::Rect const& left, cv::Rect const& right) { return left & right; }

    static bool UseScaleX(double const scale_x, double const scale_y) { return scale_x > scale_y; }

private:
    Eigen::Vector2f min_;
    Eigen::Vector2f max_;
};

void UndistortWithHomography(
        std::unique_ptr<nie::DistortionModel> const& model,
        Eigen::Matrix3d const& H,
        Eigen::Matrix3d const& K_out,
        std::vector<Eigen::Vector2f> const& d,
        std::vector<Eigen::Vector2f>* u) {
    model->Undistort(K_out, d, u);
    // TODO(PR): perspective transformation still done per point and with
    //  opencv types/functions
    for (auto& undistorted : *u) {
        cv::Point2f u_cv = nie::ConvertPoint(undistorted);
        nie::TransformPerspective(nie::ConvertMat(H), u_cv, &u_cv);
        undistorted = nie::ConvertPoint(u_cv);
    }
}

template <typename BboxType>
cv::Rect GetBboxFromUndistortedBounds(
        std::unique_ptr<nie::DistortionModel> const& model,
        Eigen::Matrix3d const& H,
        cv::Size const& size,
        Eigen::Matrix3d const& K_out) {
    BboxType bbox;

    std::vector<Eigen::Vector2f> base_bbox_points{};
    std::vector<Eigen::Vector2f> undistorted_bbox_points{};

    // initial bounding box for image of size {size} should have this many points
    base_bbox_points.reserve(size.width * 2 + size.height * 2);
    undistorted_bbox_points.reserve(size.width * 2 + size.height * 2);

    // generate base points of the outline of the image, left, right, top, bottom
    for (int i = 0; i < size.height; ++i) {
        base_bbox_points.emplace_back(0.0, static_cast<double>(i));
    }
    for (int i = 0; i < size.height; ++i) {
        base_bbox_points.emplace_back(static_cast<double>(size.width), static_cast<double>(i));
    }
    for (int i = 0; i < size.width; ++i) {
        base_bbox_points.emplace_back(static_cast<double>(i), 0.0);
    }
    for (int i = 0; i < size.width; ++i) {
        base_bbox_points.emplace_back(static_cast<double>(i), static_cast<double>(size.height));
    }

    UndistortWithHomography(model, H, K_out, base_bbox_points, &undistorted_bbox_points);

    for (int i = 0; i < static_cast<int>(undistorted_bbox_points.size()); ++i) {
        if (i < size.height) {
            bbox.UpdateMinX(undistorted_bbox_points[i]);
        } else if (i < size.height + size.height) {
            bbox.UpdateMaxX(undistorted_bbox_points[i]);
        } else if (i < size.height + size.height + size.width) {
            bbox.UpdateMinY(undistorted_bbox_points[i]);
        } else if (size.height + size.height + size.width + size.width) {
            bbox.UpdateMaxY(undistorted_bbox_points[i]);
        }
    }

    return bbox.GetRect();
}

template <typename BboxType>
Eigen::Matrix3d GetKfromBbox(Eigen::Matrix3d const& K_in, cv::Rect const& bbox, cv::Size const& size) {
    double scale_x = static_cast<double>(size.width) / static_cast<double>(bbox.width);
    double scale_y = static_cast<double>(size.height) / static_cast<double>(bbox.height);
    double offset_x = 0.0;
    double offset_y = 0.0;
    double scalar;

    if (BboxType::UseScaleX(scale_x, scale_y)) {
        scalar = scale_x;
        offset_y -= (static_cast<double>(size.height) - scale_x * static_cast<double>(bbox.height)) / 2.0;
    } else {
        scalar = scale_y;
        offset_x -= (static_cast<double>(size.width) - scale_y * static_cast<double>(bbox.width)) / 2.0;
    }

    offset_x += static_cast<double>(bbox.x) * scalar;
    offset_y += static_cast<double>(bbox.y) * scalar;

    Eigen::Matrix3d K_out = K_in;
    K_out *= scalar;
    K_out(0, 2) -= offset_x;
    K_out(1, 2) -= offset_y;
    K_out(2, 2) = 1.0;

    return K_out;
}

// MONO ONLY

Eigen::Matrix3d GetKmaxFnoSkew(std::unique_ptr<nie::DistortionModel> const& model) {
    Eigen::Matrix3d K_out = model->K();
    double f = std::max(K_out(0, 0), K_out(1, 1));
    K_out(0, 0) = f;
    K_out(1, 1) = f;
    K_out(0, 1) = 0.0;

    return K_out;
}

// This method will give us a K matrix for an undistorted image. Properties depend on the given template argument.
template <typename BboxType>
Eigen::Matrix3d GetRectifiedParametersMono(std::unique_ptr<nie::DistortionModel> const& model, cv::Size const& size) {
    // Initially we start with setting the focal length equal. Either the minimum or maximum will
    // do as long as the aspect ratio is fixed. We also remove skew.
    Eigen::Matrix3d K_out = GetKmaxFnoSkew(model);
    cv::Rect max_bbox = GetBboxFromUndistortedBounds<BboxType>(model, Eigen::Matrix3d::Identity(), size, K_out);
    return GetKfromBbox<BboxType>(K_out, max_bbox, size);
}

// STEREO ONLY

Eigen::Matrix3d GetKmaxFnoSkewEqualPpy(
        std::unique_ptr<nie::DistortionModel> const& model_left,
        std::unique_ptr<nie::DistortionModel> const& model_right) {
    Eigen::Matrix3d K_l = model_left->K();
    Eigen::Matrix3d K_r = model_right->K();
    Eigen::Matrix3d K_out = K_l;  // Doesn't matter since we change it again anyway.

    double f = std::max({K_l(0, 0), K_l(1, 1), K_r(0, 0), K_r(1, 1)});
    K_out(0, 0) = f;
    K_out(1, 1) = f;
    K_out(0, 1) = 0.0;

    return K_out;
}

template <typename BboxType>
void GetRectifiedParametersStereoCw(
        std::unique_ptr<nie::DistortionModel> const& model_left,
        std::unique_ptr<nie::DistortionModel> const& model_right,
        nie::Isometry3qd const& baseline,
        cv::Size const& size,
        Eigen::Matrix3d* p_K,
        Eigen::Vector3d* p_baseline_t_c,
        Eigen::Matrix3d* p_R_c_left,
        Eigen::Matrix3d* p_R_c_right) {
    GetRectifiedParametersStereoCw(baseline, p_baseline_t_c, p_R_c_left, p_R_c_right);

    Eigen::Matrix3d K_temp = GetKmaxFnoSkewEqualPpy(model_left, model_right);
    Eigen::Matrix3d H_left = K_temp * *p_R_c_left * K_temp.inverse();
    Eigen::Matrix3d H_right = K_temp * *p_R_c_right * K_temp.inverse();
    cv::Rect max_bbox_left = GetBboxFromUndistortedBounds<BboxType>(model_left, H_left, size, K_temp);
    cv::Rect max_bbox_right = GetBboxFromUndistortedBounds<BboxType>(model_right, H_right, size, K_temp);
    cv::Rect intersection_bbox = BboxType::GetMergedBbox(max_bbox_left, max_bbox_right);

    *p_K = GetKfromBbox<BboxType>(K_temp, intersection_bbox, size);
}

void RunModeRectificationDirectory(
        std::string const& in_path_images, std::string const& out_path_rectified, cv::Mat const& lut) {
    std::uint32_t processor_count = std::thread::hardware_concurrency();
    std::uint32_t processor_count_stages =
            static_cast<std::uint32_t>(std::floor(static_cast<float>(processor_count) / 3.0f));

    VLOG(2) << "Using maximum amount of processors for rectification: " << processor_count;

    nie::WorkPool<std::pair<std::string, cv::Mat>> pool_image_writer(
            processor_count_stages * 2, processor_count * 2, &WriteImage);
    nie::gpu::GpuImageWarper warper(lut);

    nie::WorkPool<std::pair<std::string, cv::Mat>> pool_undistorter(
            1,
            processor_count * 2,
            [&out_path_rectified, &warper, &pool_image_writer](
                    const std::unique_ptr<std::pair<std::string, cv::Mat>>& work) -> void {
                std::chrono::time_point<std::chrono::system_clock> before = std::chrono::system_clock::now();
                cv::Mat dst;

                warper.Warp(work->second, &dst);

                std::unique_ptr<std::pair<std::string, cv::Mat>> out =
                        std::make_unique<std::pair<std::string, cv::Mat>>();
                out->first = out_path_rectified + boost::filesystem::path(work->first).filename().string();
                out->second = dst;

                std::chrono::time_point<std::chrono::system_clock> after = std::chrono::system_clock::now();

                auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
                auto ms = milliseconds.count();

                pool_image_writer.queue().BlockingPushBack(std::move(out));

                VLOG(3) << "Undistorted image in ms: " << ms;
            });

    nie::WorkPool<std::string> pool_image_reader(
            processor_count_stages,
            processor_count * 2,
            [&pool_undistorter](const std::unique_ptr<std::string>& work) -> void {
                std::chrono::time_point<std::chrono::system_clock> before = std::chrono::system_clock::now();
                std::unique_ptr<std::pair<std::string, cv::Mat>> out =
                        std::make_unique<std::pair<std::string, cv::Mat>>();
                out->first = *work;
                out->second = cv::imread(*work);

                if (out->second.data) {
                    std::chrono::time_point<std::chrono::system_clock> after = std::chrono::system_clock::now();

                    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(after - before);
                    auto ms = milliseconds.count();

                    VLOG(3) << "Read image in ms: " << ms;

                    pool_undistorter.queue().BlockingPushBack(std::move(out));
                } else {
                    LOG(WARNING) << "Unable to read: " << *work;
                }
            });

    // std::cout << FLAGS_path_images << " is a directory containing: " std::endl;
    for (auto& entry : boost::make_iterator_range(boost::filesystem::directory_iterator(in_path_images), {})) {
        if (boost::filesystem::is_regular_file(entry)) {
            // std::cout << entry.path().string() << std::endl;
            pool_image_reader.queue().BlockingPushBack(std::make_unique<std::string>(entry.path().string()));
        }
    }
}

static std::string ReturnSecondStringId() {
    using namespace boost::posix_time;
    ptime t = microsec_clock::universal_time();
    return std::string{"second"}.append(to_iso_extended_string(t)).append("Z\n");
}

void RunModeRectificationMono(
        std::string const& in_path_images,
        std::string const& in_path_intrinsics,
        std::string const& out_path_rectified,
        std::string const& out_path_intrinsics) {
    auto calibrated = nie::io::CalibratedMonoParameters::Read(in_path_intrinsics);
    auto model = nie::DistortionModelFactory::Create(
            DistortionParametersFromCalibratedParameters(calibrated.distortion_parameters()),
            calibrated.lens().intrinsics);
    auto image_size = calibrated.image_size();
    auto K_c = GetRectifiedParametersMono<BboxNoBlackPixels>(model, image_size);

    // NOTE: For remapped images the standard deviations are a LUT for the entire image.
    std::vector<nie::io::FrameData> frames{{calibrated.lens().id, nie::Isometry3qd::Identity()}};
    // TODO missing camera id from CalibratedMonoParameters
    nie::io::RectifiedCameraParameters rectified{
            ReturnSecondStringId(), {image_size.width, image_size.height}, K_c, frames};
    rectified.Write(out_path_intrinsics);

    cv::Mat lut = nie::GetLookUpTable(
            [&model, &image_size, &K_c](cv::Point2f const& p_u, cv::Point2f* p_d) {
                Eigen::Vector2f p_temp;
                bool ret = RemapCoordinateAtoB(model, image_size, K_c, nie::ConvertPoint(p_u), &p_temp);
                *p_d = nie::ConvertPoint(p_temp);
                return ret;
            },
            image_size);

    RunModeRectificationDirectory(in_path_images, out_path_rectified, lut);
}

void RunModeRectificationStereo(
        std::string const& in_path_images,
        std::string const& path_relative_left,
        std::string const& path_relative_right,
        std::string const& in_path_intrinsics,
        std::string const& out_path_images,
        std::string const& out_path_intrinsics) {
    auto camera_parameters = nie::io::CalibratedStereoParameters::Read(in_path_intrinsics);
    auto model_left = nie::DistortionModelFactory::Create(
            DistortionParametersFromCalibratedParameters(camera_parameters.distortion_parameters()),
            camera_parameters.lens_left().intrinsics);
    auto model_right = nie::DistortionModelFactory::Create(
            DistortionParametersFromCalibratedParameters(camera_parameters.distortion_parameters()),
            camera_parameters.lens_right().intrinsics);
    auto image_size = camera_parameters.image_size();

    Eigen::Matrix3d K_c;
    Eigen::Vector3d cw_baseline_t_c;
    Eigen::Matrix3d cw_R_c_left;
    Eigen::Matrix3d cw_R_c_right;

    GetRectifiedParametersStereoCw<BboxNoBlackPixels>(
            model_left,
            model_right,
            camera_parameters.baseline().estimate.Inversed(),
            camera_parameters.image_size(),
            &K_c,
            &cw_baseline_t_c,
            &cw_R_c_left,
            &cw_R_c_right);

    // Note that we convert the baseline from cw to wc.
    std::vector<nie::io::FrameData> frames{
            {camera_parameters.lens_left().id, nie::Isometry3qd::Identity()},
            {camera_parameters.lens_right().id, nie::Isometry3qd::FromTranslation(-cw_baseline_t_c)}};
    // TODO missing camera id from CalibratedStereoParameters
    nie::io::RectifiedCameraParameters rectifiedCameraParameters{
            ReturnSecondStringId(), {image_size.width, image_size.height}, K_c, frames};
    rectifiedCameraParameters.Write(out_path_intrinsics);

    Eigen::Matrix3d H_left = K_c * cw_R_c_left.transpose() * K_c.inverse();
    Eigen::Matrix3d H_right = K_c * cw_R_c_right.transpose() * K_c.inverse();

    cv::Mat lut_left = nie::GetLookUpTable(
            [&model_left, &image_size, &K_c, &H_left](cv::Point2f const& p_u, cv::Point2f* p_d) -> bool {
                cv::Point2f p_t0;
                nie::TransformPerspective(nie::ConvertMat(H_left), p_u, &p_t0);
                Eigen::Vector2f p_t1;
                bool ret = RemapCoordinateAtoB(model_left, image_size, K_c, nie::ConvertPoint(p_t0), &p_t1);
                *p_d = nie::ConvertPoint(p_t1);
                return ret;
            },
            image_size);

    cv::Mat lut_right = nie::GetLookUpTable(
            [&model_right, &image_size, &K_c, &H_right](cv::Point2f const& p_u, cv::Point2f* p_d) -> bool {
                cv::Point2f p_t0;
                nie::TransformPerspective(nie::ConvertMat(H_right), p_u, &p_t0);
                Eigen::Vector2f p_t1;
                bool ret = RemapCoordinateAtoB(model_right, image_size, K_c, nie::ConvertPoint(p_t0), &p_t1);
                *p_d = nie::ConvertPoint(p_t1);
                return ret;
            },
            image_size);

    RunModeRectificationDirectory(in_path_images + path_relative_left, out_path_images + path_relative_left, lut_left);
    RunModeRectificationDirectory(
            in_path_images + path_relative_right, out_path_images + path_relative_right, lut_right);
}

void RunModeRectification(
        std::string const& in_path_images,
        std::string const& path_relative_left,
        std::string const& path_relative_right,
        std::string const& in_path_intrinsics,
        std::string const& out_path_images,
        std::string const& out_path_intrinsics,
        bool stereo) {
    if (!stereo) {
        RunModeRectificationMono(in_path_images, in_path_intrinsics, out_path_images, out_path_intrinsics);
    } else {
        RunModeRectificationStereo(
                in_path_images,
                path_relative_left,
                path_relative_right,
                in_path_intrinsics,
                out_path_images,
                out_path_intrinsics);
    }
}
