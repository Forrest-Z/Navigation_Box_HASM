/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

// Note that include <opencv/core/eigen.hpp> contains various converions. But not vectors or point types.
// https://docs.opencv.org/3.4/d3/ddd/eigen_8hpp.html

namespace nie {

namespace detail {

template <typename T>
struct ConvertPointTraits;

template <typename T>
struct ConvertPointTraits<cv::Point_<T>> {
    using Type = cv::Point_<T>;
    using Cast = Eigen::Matrix<T, 2, 1>;
};

template <typename T>
struct ConvertPointTraits<cv::Point3_<T>> {
    using Type = cv::Point3_<T>;
    using Cast = Eigen::Matrix<T, 3, 1>;
};

template <typename T>
struct ConvertPointTraits<Eigen::Matrix<T, 2, 1>> {
    using Type = Eigen::Matrix<T, 2, 1>;
    using Cast = cv::Point_<T>;
};

template <typename T>
struct ConvertPointTraits<Eigen::Matrix<T, 3, 1>> {
    using Type = Eigen::Matrix<T, 3, 1>;
    using Cast = cv::Point3_<T>;
};

template <typename T>
struct ConvertMatTraits;

template <typename T, int N>
struct ConvertMatTraits<cv::Vec<T, N>> {
    using Type = cv::Vec<T, N>;
    using Cast = Eigen::Matrix<T, N, 1>;
};

template <typename T, int Rows, int Cols>
struct ConvertMatTraits<cv::Matx<T, Rows, Cols>> {
    using Type = cv::Matx<T, Rows, Cols>;
    using Cast = Eigen::Matrix<T, Rows, Cols>;
};

template <typename T, int Rows, int Cols>
struct ConvertMatTraits<Eigen::Matrix<T, Rows, Cols>> {
    using Type = Eigen::Matrix<T, Rows, Cols>;
    using Cast = cv::Matx<T, Rows, Cols>;
};

}  // namespace detail

// ***************
// OpenCv to Eigen
// ***************

template <typename T>
inline Eigen::Matrix<T, 2, 1> ConvertPoint(cv::Point_<T> const& src) {
    return {src.x, src.y};
}

template <typename T>
inline Eigen::Matrix<T, 3, 1> ConvertPoint(cv::Point3_<T> const& src) {
    return {src.x, src.y, src.z};
}

template <typename T, int Rows>
inline Eigen::Matrix<T, Rows, 1> ConvertMat(cv::Vec<T, Rows> const& src) {
    return Eigen::Matrix<T, Rows, 1>{src.val};
}

template <typename T, int Rows, int Cols>
inline Eigen::Matrix<T, Rows, Cols> ConvertMat(cv::Matx<T, Rows, Cols> const& src) {
    return Eigen::Matrix<T, Rows, Cols>{Eigen::Map<const Eigen::Matrix<T, Rows, Cols, Eigen::RowMajor>>{src.val}};
}

template <typename T, int Rows>
inline Eigen::Matrix<T, Rows, 1> ConvertMat(cv::Matx<T, Rows, 1> const& src) {
    return Eigen::Matrix<T, Rows, 1>{Eigen::Map<const Eigen::Matrix<T, Rows, 1>>{src.val}};
}

// Non deducible template parameter T
template <typename T>
inline Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> ConvertMat(cv::Mat const& src) {
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> dst;
    cv::cv2eigen(src, dst);
    return dst;
}

// ***************
// Eigen to OpenCv
// ***************

template <typename T>
inline cv::Point_<T> ConvertPoint(Eigen::Matrix<T, 2, 1> const& src) {
    return {src.x(), src.y()};
}

template <typename T>
inline cv::Point3_<T> ConvertPoint(Eigen::Matrix<T, 3, 1> const& src) {
    return {src.x(), src.y(), src.z()};
}

template <typename T, int Rows, int Cols>
inline cv::Matx<T, Rows, Cols> ConvertMat(Eigen::Matrix<T, Rows, Cols> const& src) {
    Eigen::Matrix<T, Cols, Rows> t = src.transpose();
    return cv::Matx<T, Rows, Cols>(t.data());
}

template <typename T, int Rows>
inline cv::Vec<T, Rows> ConvertVec(Eigen::Matrix<T, Rows, 1> const& src) {
    return cv::Vec<T, Rows>(src.data());
}

template <typename T>
inline cv::Mat ConvertMat(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> const& src) {
    cv::Mat dst;
    cv::eigen2cv(src, dst);
    return dst;
}

// ***************
// All
// ***************

template <typename T>
inline std::vector<typename detail::ConvertPointTraits<T>::Cast> ConvertPoints(std::vector<T> const& srcs) {
    std::vector<typename detail::ConvertPointTraits<T>::Cast> converted(srcs.size());
    for (std::size_t i = 0; i < srcs.size(); ++i) {
        converted[i] = ConvertPoint(srcs[i]);
    }
    return converted;
}

template <typename T>
inline std::vector<typename detail::ConvertMatTraits<T>::Cast> ConvertMats(std::vector<T> const& srcs) {
    std::vector<typename detail::ConvertMatTraits<T>::Cast> converted(srcs.size());
    for (std::size_t i = 0; i < srcs.size(); ++i) {
        converted[i] = ConvertMat(srcs[i]);
    }
    return converted;
}

}  // namespace nie
