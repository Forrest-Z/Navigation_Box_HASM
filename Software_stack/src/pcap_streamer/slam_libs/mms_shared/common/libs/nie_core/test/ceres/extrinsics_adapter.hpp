/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef CERES_EXTRINSICS_ADAPTER_HPP
#define CERES_EXTRINSICS_ADAPTER_HPP

#include <vector>

#include <ceres/rotation.h>
#include <opencv2/core.hpp>

namespace nie {

// Keep everything related to the extrinsics vector in 1 place.
// Extrinsics: 6 elements => translation + angle axis rotation
template <typename T>
class ExtrinsicsAdapter {
public:
    static constexpr int kParametersExtrinsics = 6;

    explicit ExtrinsicsAdapter(T* const p_extrinsics) : p_extrinsics_(p_extrinsics) {}

    static std::vector<T> CreateVector(cv::Point3d const& t, cv::Vec3d const& R);
    template <typename U>
    static std::vector<T> CreateVector(std::vector<U> const& extrinsics);

    // The functions below are kept as double type on purpose.
    // Jets are also initialized from doubles.
    void Set(cv::Point3d const& t, cv::Vec3d const& R);
    void Set(cv::Point3d const& t, cv::Matx33d const& R);
    // Changes the interpretation of the extrinsics to be its inverse.
    // I.e., camera -> world to world -> camera.
    // p_other = { -R.t() * t, R.t() }
    template <typename U>
    void Invert(U* const p_other) const;
    void Invert();

    // out = R * in + t
    template <typename U>
    void Transform(const U* const in, U* out) const;

    cv::Point3_<T> translation() const;
    cv::Vec<T, 3> rotation() const;

private:
    T* const p_extrinsics_;
};

template <typename T>
std::vector<T> ExtrinsicsAdapter<T>::CreateVector(cv::Point3d const& t, cv::Vec3d const& R) {
    std::vector<T> e(6);
    ExtrinsicsAdapter adapter(e.data());
    adapter.Set(t, R);
    return e;
}

template <typename T>
template <typename U>
std::vector<T> ExtrinsicsAdapter<T>::CreateVector(std::vector<U> const& extrinsics) {
    std::vector<T> e(6);
    e[0] = T(extrinsics[0]);
    e[1] = T(extrinsics[1]);
    e[2] = T(extrinsics[2]);
    e[3] = T(extrinsics[3]);
    e[4] = T(extrinsics[4]);
    e[5] = T(extrinsics[5]);

    return e;
}

template <typename T>
void ExtrinsicsAdapter<T>::Set(cv::Point3d const& t, cv::Vec3d const& R) {
    p_extrinsics_[0] = T(t.x);
    p_extrinsics_[1] = T(t.y);
    p_extrinsics_[2] = T(t.z);
    p_extrinsics_[3] = T(R[0]);
    p_extrinsics_[4] = T(R[1]);
    p_extrinsics_[5] = T(R[2]);
}

template <typename T>
void ExtrinsicsAdapter<T>::Set(cv::Point3d const& t, cv::Matx33d const& R) {
    p_extrinsics_[0] = T(t.x);
    p_extrinsics_[1] = T(t.y);
    p_extrinsics_[2] = T(t.z);

    double axis_angle[3];

    ceres::MatrixAdapter<const double, 3, 1> adapter(R.val);
    ceres::RotationMatrixToAngleAxis<double, 3, 1>(adapter, axis_angle);

    p_extrinsics_[3] = T(axis_angle[0]);
    p_extrinsics_[4] = T(axis_angle[1]);
    p_extrinsics_[5] = T(axis_angle[2]);
}

template <typename T>
template <typename U>
void ExtrinsicsAdapter<T>::Invert(U* const p_other) const {
    p_other[3] = -p_extrinsics_[3];
    p_other[4] = -p_extrinsics_[4];
    p_other[5] = -p_extrinsics_[5];

    T t[3] = {-p_extrinsics_[0], -p_extrinsics_[1], -p_extrinsics_[2]};

    ceres::AngleAxisRotatePoint(p_other + 3, t, p_other);
}

template <typename T>
void ExtrinsicsAdapter<T>::Invert() {
    Invert(p_extrinsics_);
}

template <typename T>
template <typename U>
void ExtrinsicsAdapter<T>::Transform(U const* const in, U* out) const {
    ceres::AngleAxisRotatePoint(p_extrinsics_ + 3, in, out);

    out[0] = out[0] + p_extrinsics_[0];
    out[1] = out[1] + p_extrinsics_[1];
    out[2] = out[2] + p_extrinsics_[2];
}

template <typename T>
cv::Point3_<T> ExtrinsicsAdapter<T>::translation() const {
    return cv::Point3_<T>(p_extrinsics_[0], p_extrinsics_[1], p_extrinsics_[2]);
}

template <typename T>
cv::Vec<T, 3> ExtrinsicsAdapter<T>::rotation() const {
    return cv::Vec<T, 3>(p_extrinsics_[3], p_extrinsics_[4], p_extrinsics_[5]);
}

// Helper class for having an invert function that can be used with
// ceres auto diff for the purpose of covariance transformations.
// See => error_propagation.hpp
class ExtrinsicsInvertFunction {
public:
    template <typename T>
    bool operator()(const T* const x, T* y) const {
        ExtrinsicsAdapter<T const>(x).Invert(y);

        return true;
    }
};

}  // namespace nie

#endif  // CERES_EXTRINSICS_ADAPTER_HPP
