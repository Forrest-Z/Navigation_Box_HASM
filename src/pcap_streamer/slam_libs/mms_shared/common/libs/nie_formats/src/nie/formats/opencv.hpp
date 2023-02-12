/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_FORMATS_OPENCV_HPP
#define NIE_FORMATS_OPENCV_HPP

#include <glog/logging.h>
#include <Eigen/Geometry>
#include <nie/core/geometry/conversion.hpp>
#include <nie/core/geometry/isometry3.hpp>
#include <opencv2/core.hpp>

namespace cv {

namespace detail {

template <typename T>
bool CheckNodeNotEmpty(cv::FileNode const& node, T& x, T const& default_value) {
    if (node.empty()) {
        x = default_value;
        return false;
    }

    return true;
}

}  // namespace detail

template <typename Rotation>
void write(cv::FileStorage& fs, std::string const&, nie::Isometry3<Rotation> const& p) {
    fs << "{";
    fs << "translation" << p.translation();
    fs << "rotation" << p.rotation();
    fs << "}";
}

template <typename Rotation>
void read(
        cv::FileNode const& node,
        nie::Isometry3<Rotation>& x,
        nie::Isometry3<Rotation> const& default_value = nie::Isometry3<Rotation>()) {
    if (detail::CheckNodeNotEmpty(node, x, default_value)) {
        node["translation"] >> x.translation();
        node["rotation"] >> x.rotation();
    }
}

template <typename T, int Rows, int Cols>
void write(cv::FileStorage& fs, std::string const&, Eigen::Matrix<T, Rows, Cols> const& p) {
    fs << "[";
    for (std::size_t row = 0; row < Rows; ++row) {
        for (std::size_t col = 0; col < Cols; ++col) {
            fs << p(row, col);
        }
    }
    fs << "]";
}

template <typename T, int Rows, int Cols>
void read(
        cv::FileNode const& node,
        Eigen::Matrix<T, Rows, Cols>& x,
        Eigen::Matrix<T, Rows, Cols> const& default_value = {}) {
    if (detail::CheckNodeNotEmpty(node, x, default_value)) {
        constexpr size_t expected_size = Rows * Cols;
        CHECK(node.size() == expected_size);
        for (std::size_t row = 0; row < Rows; ++row) {
            for (std::size_t col = 0; col < Cols; ++col) {
                node[col + row * Cols] >> x(row, col);
            }
        }
    }
}

template <typename T>
void write(cv::FileStorage& fs, std::string const&, Eigen::Quaternion<T> const& p) {
    fs << "[";
    fs << p.x();
    fs << p.y();
    fs << p.z();
    fs << p.w();
    fs << "]";
}

template <typename T>
void read(
        cv::FileNode const& node,
        Eigen::Quaternion<T>& x,
        Eigen::Quaternion<T> const& default_value = Eigen::Quaternion<T>::Identity()) {
    if (detail::CheckNodeNotEmpty(node, x, default_value)) {
        node[0] >> x.x();
        node[1] >> x.y();
        node[2] >> x.z();
        node[3] >> x.w();
        x.normalize();
    }
}

template <typename T>
void write(cv::FileStorage& fs, std::string const&, Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> const& p) {
    auto converted = nie::ConvertMat(p);
    fs << converted;
}

template <typename T>
void read(
        cv::FileNode const& node,
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& x,
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> const& default_value = {}) {
    if (detail::CheckNodeNotEmpty(node, x, default_value)) {
        cv::Mat cast;
        node >> cast;
        x = nie::ConvertMat<T>(cast);
    }
}

}  // namespace cv

namespace nie {

namespace io {

// Very generic single OpenCV serializable type IO. Each type should have a read and write function. Many examples above
// this comment.
template <typename T>
class OpenCvSerializer {
public:
    static T Read(std::string const& filename) {
        cv::FileStorage storage(filename, cv::FileStorage::READ);
        CHECK(storage.isOpened()) << "Unable to read file: " + filename;
        T value;
        storage["value"] >> value;
        return value;
    }

    static void Write(std::string const& filename, T const& value) {
        cv::FileStorage storage(filename, cv::FileStorage::WRITE);
        CHECK(storage.isOpened()) << "Unable to write file: " + filename;
        storage << "value" << value;
    }
};

}  // namespace io

}  // namespace nie

#endif  // NIE_FORMATS_OPENCV_HPP
