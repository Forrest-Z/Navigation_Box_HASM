/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "filter_ground_plane.hpp"

#include <pcl/common/transforms.h>
#include <nie/core/time.hpp>
#include <opencv2/opencv.hpp>

#define PLOT_IMAGES false

#if PLOT_IMAGES
namespace plot {

using ImageType = std::uint8_t;

template <typename Scalar>
static cv::Mat_<ImageType> CreateImage(cv::Mat const& map, std::function<Scalar(Scalar)> const& normalize) {
    cv::Mat_<ImageType> image(map.size());
    std::transform(map.begin<Scalar>(), map.end<Scalar>(), image.begin(), [&normalize](Scalar x) -> ImageType {
        return static_cast<ImageType>(normalize(x) * std::numeric_limits<ImageType>::max());
    });
    return image;
}

static cv::Mat CreateColorImage(cv::Mat_<ImageType> const& image, int map = -1) {
    cv::Mat result;
    if (map == -1) {
        cv::cvtColor(image, result, CV_GRAY2BGR);  // Explicit 1 to 3 channels conversion
    } else {
        cv::applyColorMap(image, result, map);  // Automatic 1 to 3 channels conversion
    }
    return result;
}

cv::Mat ShiftImage(cv::Mat image, int offset_x, int offset_y) {
    cv::Mat trans_mat = (cv::Mat_<double>(2, 3) << 1, 0, offset_x, 0, 1, offset_y);
    cv::warpAffine(image, image, trans_mat, image.size(), cv::INTER_LINEAR, cv::BORDER_WRAP);
    return image;
}

// Blocking function that waits until a key is pressed, or the image window is closed
void Plot(std::vector<cv::Mat> const& images, int shift_hor = 0) {
    // Create full result image
    cv::Mat result = images.front();
    for (std::size_t i = 1; i < images.size(); ++i) {
        auto const& image = images[i];
        cv::vconcat(result, image, result);
    }
    result = ShiftImage(result, shift_hor, 0);

    std::string const name = "images";
    cv::namedWindow(name, cv::WINDOW_NORMAL);
    cv::resizeWindow(name, result.size().width * 6, result.size().height * 7);
    cv::imshow(name, result);

    cv::waitKey(0);
    cv::destroyAllWindows();
}

}  // namespace plot
#endif

namespace nie {

namespace detail {

using LabelType = uchar;
enum class Label : LabelType { kNone = 9, kNotGround = 0, kGround = 1 };
static LabelType constexpr Value(Label const& l) { return static_cast<LabelType>(l); }

struct Pixel {
    int r;
    int c;

    // Returns whether the pixel was updated
    bool Move(std::int8_t dr, std::int8_t dc, cv::Size const& size) {
        bool result = false;
        if (dr != 0) result = Clamp(dr, size.height, &r);
        if (dc != 0) result = Clamp(dc, size.width, &c) || result;
        return result;
    }

    // Apply delta to pixel coordinate, without passing the borders. Returns whether the pixel coordinate was
    // actually updated.
    static bool Clamp(int const delta, int const size, int* x) {
        int const x_orig = *x;
        *x += delta;
        *x = *x < 0 ? 0 : *x >= size ? size - 1 : *x;
        return *x != x_orig;
    }
};

static double CalculateLocalNormal(
        cv::Mat_<double> const& x_map,
        cv::Mat_<double> const& y_map,
        cv::Mat_<double> const& z_map,
        Pixel const& p,
        int const width) {
    // If the pixel is at the last column of the maps
    bool const flip_c = p.c == width - 1;
    // If the pixel is as the last column, then this value is -1, to
    //  - loop a column back, instead of forward
    //  - flip the between-column difference unit vector
    int const dc = flip_c ? -1 : 1;

    // Look up the central pixel averaged 3D point location and do the same for the pixel below and to the right.
    // At the last column, then the point is taken for the column of the left.
    Eigen::Vector3d const p_0 = {x_map(p.r, p.c), y_map(p.r, p.c), z_map(p.r, p.c)};
    Eigen::Vector3d const p_dr = {x_map(p.r + 1, p.c), y_map(p.r + 1, p.c), z_map(p.r + 1, p.c)};
    Eigen::Vector3d const p_dc = {x_map(p.r, p.c + dc), y_map(p.r, p.c + dc), z_map(p.r, p.c + dc)};

    // Calculate the difference unit vectors
    Eigen::Vector3d const dx = (p_dr - p_0).normalized();
    Eigen::Vector3d const dy = dc * (p_dc - p_0).normalized();

    // The cross product calculates the normal to the surface locally (spanned by the between-column and between-row
    // vectors) and is pointing towards the lidar. The angle between this normal and the positive z-axis
    // (pointing up) is the property of interest.
    return std::acos(dx.cross(dy).normalized().z());
}

}  // namespace detail

[[nodiscard]] std::vector<bool> GroundPlaneFilter::filter(
        io::lidar::Returns const& returns, io::lidar::Angles const& angles) const {
    CHECK(returns.points.size() == angles.hor_angles.size());
    CHECK(returns.points.size() == angles.ver_angles.size());

    // Note that the lidar points received are transformed from lidar to gps system
    pcl::PointCloud<pcl::PointXYZI> const points = ApplyCalibration(returns.points);
    std::vector<double> const& hor_angles = angles.hor_angles;
    std::vector<double> const& ver_angles = angles.ver_angles;

    // Dimensions of the images
    int const rows = num_lasers_;
    int const cols = options_.columns;

    cv::Size const image_size{cols, rows};
    cv::Size const angle_image_size{cols, rows - 1};

    // Determine the minimum and maximum vertical angles spanning the beam scan lines
    auto const [min_ver_angle, max_ver_angle] = std::minmax_element(ver_angles.cbegin(), ver_angles.cend());
    double const del_ver_angle = (*max_ver_angle - *min_ver_angle) / (rows - 1);
    auto const angle_index = [min = *min_ver_angle, del = del_ver_angle](double angle) -> int {
        return static_cast<int>(std::trunc((angle - min) / del + .5));
    };

    // Calculate the required average quantities
    cv::Mat_<double> x_map(image_size, 0), y_map(image_size, 0), z_map(image_size, 0);
    cv::Mat_<uchar> count_map(image_size, 0);
    for (std::size_t i = 0; i < points.size(); ++i) {
        // Translate the horizontal and vertical angles to the image row and column indices
        std::size_t const r = image_size.height - 1 - angle_index(ver_angles[i]);
        std::size_t const c = hor_angles[i] / 360. * image_size.width;

        x_map(r, c) += points[i].x;
        y_map(r, c) += points[i].y;
        z_map(r, c) += points[i].z;
        ++count_map(r, c);
    }
    x_map /= count_map;
    y_map /= count_map;
    z_map /= count_map;

    // Calculate angle maps
    //  - a: inclination angle calculated along a columns, angle between ground and x-y plane
    //  - n: angle between the normal of the local surface and the z/up direction
    cv::Mat_<double> a_map(angle_image_size, -10);
    cv::Mat_<double> n_map(angle_image_size, 0);
    for (int x = 0; x < angle_image_size.height; ++x) {
        auto const r_curr = image_size.height - 1 - x;
        auto const r_next = image_size.height - 1 - (x + 1);
        auto const ra = angle_image_size.height - 1 - x;
        for (int c = 0; c < angle_image_size.width; ++c) {
            // Note that the iteration over a column (using r) is actually top-to-bottom
            Eigen::Vector3d const v_curr_l{x_map(r_curr, c), y_map(r_curr, c), z_map(r_curr, c)};
            Eigen::Vector3d const v_next_l{x_map(r_next, c), y_map(r_next, c), z_map(r_next, c)};
            a_map(ra, c) = std::asin((v_next_l - v_curr_l).normalized().z());
            n_map(ra, c) = detail::CalculateLocalNormal(x_map, y_map, z_map, {ra, c}, image_size.width);
        }
    }

    // Smooth the angle map
    // Apply Savitzky-Folay filter for smoothing the columns for better angle classification
    // https://en.wikipedia.org/wiki/Savitzky%E2%80%93Golay_filter#Derivation_of_convolution_coefficients
    cv::Mat_<double> af_map;  // filtered angle map
    cv::Vec<double, 1> const kernel_x{1.};
    cv::Vec<double, 5> const kernel_y{-3., 12., 17., 12., -3.};
    cv::sepFilter2D(a_map, af_map, -1, kernel_x, kernel_y);
    af_map /= cv::sum(kernel_y);

    // Calculate the actual selection of the pixels (binned points) that are within the thresholds and are so marked
    // as being part of the ground plane.
    cv::Mat_<detail::LabelType> s_map(angle_image_size, Value(detail::Label::kNone));

    // Loop over the bottom row
    for (int c = 0; c < angle_image_size.width; ++c) {
        detail::Pixel const p0{angle_image_size.height - 1, c};
        detail::Pixel const p1{angle_image_size.height - 2, c};
        if (s_map(p0.r, p0.c) == Value(detail::Label::kNone)) {
            if (z_map(p0.r, p0.c) < 0. && count_map(p0.r, p0.c) > 0 && count_map(p1.r, p1.c) > 0 &&
                std::abs(af_map(p0.r, p0.c)) < options_.init_angle &&
                CalculateLocalNormal(x_map, y_map, z_map, p0, image_size.width) < options_.max_normal_angle) {
                std::queue<detail::Pixel> candidates;
                candidates.push(p0);
                s_map(p0.r, p0.c) = Value(detail::Label::kGround);
                while (!candidates.empty()) {
                    detail::Pixel const& p = candidates.front();

                    // Function to check a neighbour
                    auto const check = [&](std::int8_t dr, std::int8_t dc) {
                        auto px = p;
                        if (px.Move(dr, dc, angle_image_size) && s_map(px.r, px.c) == Value(detail::Label::kNone) &&
                            z_map(px.r, px.c) < 0. && count_map(px.r, px.c) > 0 &&
                            std::abs(af_map(p.r, p.c) - af_map(px.r, px.c)) < options_.diff_angle &&
                            CalculateLocalNormal(x_map, y_map, z_map, px, image_size.width) <
                                    options_.max_normal_angle) {
                            candidates.push(px);
                            s_map(px.r, px.c) = Value(detail::Label::kGround);
                        }
                    };
                    // Actually check all neighbours
                    check(0, -1);  // left
                    check(0, 1);   // right
                    check(-1, 0);  // up
                    check(1, 0);   // down

                    candidates.pop();
                }
            }
        }
    }
    // replace all unlabelled pixels as not-ground
    s_map.setTo(Value(detail::Label::kNotGround), s_map == Value(detail::Label::kNone));
    auto sf_map = s_map.clone();
    cv::dilate(sf_map, sf_map, cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3}));
    cv::erode(sf_map, sf_map, cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3}));
    cv::erode(sf_map, sf_map, cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3}));
    cv::dilate(sf_map, sf_map, cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3}));

#if PLOT_IMAGES
    // Determine the range of the 3d point coordinates
    Eigen::Vector3f min = points.points.front().getVector3fMap();
    Eigen::Vector3f diff = min;
    std::for_each(points.points.cbegin(), points.points.cend(), [&min, &diff](pcl::PointXYZI const& p) {
        min = p.getVector3fMap().cwiseMin(min);
        diff = p.getVector3fMap().cwiseMax(diff);
    });
    diff -= min;

    // Normalize values from range [min, max] to [0, 1]
    auto x_image = plot::CreateImage<double>(x_map, [&min, &diff](double x) { return (x - min.x()) / diff.x(); });
    auto y_image = plot::CreateImage<double>(y_map, [&min, &diff](double y) { return (y - min.y()) / diff.y(); });
    auto z_image = plot::CreateImage<double>(z_map, [&min, &diff](double z) { return (z - min.z()) / diff.z(); });

    // Normalize values from range [-pi, pi] to [0, 1]
    auto a_image = plot::CreateImage<double>(a_map, [](double a) { return (a / kPi<> + 1.) / 2.; });
    auto af_image = plot::CreateImage<double>(af_map, [](double a) { return (a / kPi<> + 1.) / 2.; });

    // Normalize values from range [pi, 0] to [0, 1]
    auto n_image = plot::CreateImage<double>(n_map, [](double a) { return 1. - a / kPi<>; });

    auto s_image = plot::CreateImage<detail::LabelType>(s_map, [](detail::LabelType s) { return s; });
    auto sf_image = plot::CreateImage<detail::LabelType>(sf_map, [](detail::LabelType s) { return s; });

    std::vector<cv::Mat> const images = {// plot::CreateColorImage(x_image, cv::COLORMAP_HOT),
                                         // plot::CreateColorImage(y_image, cv::COLORMAP_HOT),
                                         // plot::CreateColorImage(z_image, cv::COLORMAP_HOT),
                                         plot::CreateColorImage(a_image),
                                         plot::CreateColorImage(af_image),
                                         // plot::CreateColorImage(af_image, cv::COLORMAP_TWILIGHT),
                                         plot::CreateColorImage(n_image),
                                         plot::CreateColorImage(s_image),
                                         plot::CreateColorImage(sf_image)};

    int shift_hor = 0;
    for (std::size_t i = 0; i < points.size() - 1; ++i) {
        if (points[i].z < 0 && std::signbit(points[i].x) != std::signbit(points[i + 1].x)) {
            shift_hor = static_cast<int>(hor_angles[i] / 360. * image_size.width + 1);
            break;
        }
    }

    plot::Plot(images, shift_hor);
#endif

    // Construct filter to return
    std::vector<bool> filter(points.size(), false);
    for (std::size_t i = 0; i < points.size() - 1; ++i) {
        // Translate the horizontal and vertical angles to the image row and column indices
        int const r = image_size.height - 1 - angle_index(ver_angles[i]);
        int const c = static_cast<int>(hor_angles[i] / 360. * image_size.width);

        if ((r != image_size.height - 1 && s_map(r, c) == Value(detail::Label::kGround)) ||
            (r != 0 && s_map(r - 1, c) == Value(detail::Label::kGround))) {
            filter[i] = true;
        }
    }

    return filter;
}

// Apply the rotation of the lidar extrinsics to transform the points from the lidar system to GPS
[[nodiscard]] pcl::PointCloud<GroundPlaneFilter::PointT> GroundPlaneFilter::ApplyCalibration(
        pcl::PointCloud<PointT> const& points) const {
    pcl::PointCloud<PointT> result;
    pcl::transformPointCloud(points, result, T_calib_extr_rot_);
    return result;
}

}  // namespace nie
