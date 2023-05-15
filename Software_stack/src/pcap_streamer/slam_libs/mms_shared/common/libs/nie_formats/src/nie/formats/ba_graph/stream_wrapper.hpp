/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <array>
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <type_traits>

#include <glog/logging.h>
#include <Eigen/Geometry>
#include <nie/core/geometry/isometry3.hpp>
#include <nie/core/time.hpp>
#include <opencv2/core.hpp>

#include "ba_graph_types.hpp"

namespace nie {

namespace io {

inline std::size_t SizeOfString(std::string const& s) { return sizeof(std::int32_t) + s.size(); }

// TODO(jbr): Perhaps elevate class.
// This class is simply a promise of intent.
// Breaks google convention. Reason: class is a reference.
template <typename Derived>
struct SymmetricMatrixRef : public std::reference_wrapper<Derived> {
    // Forces Eigen matrix expression type
    static_assert(
        std::is_base_of<
            Eigen::MatrixBase<typename std::remove_const<Derived>::type>,
            typename std::remove_const<Derived>::type>::value,
        "ONLY EIGEN MATRICES ARE SUPPORTED");
    static_assert(Derived::RowsAtCompileTime == Derived::ColsAtCompileTime, "ONLY SQUARE MATRICES SUPPORTED.");
    explicit SymmetricMatrixRef(Derived& matrix) : std::reference_wrapper<Derived>(matrix) {}
};

template <typename Derived>
SymmetricMatrixRef<Derived> MakeSymmetricMatrixRef(Eigen::MatrixBase<Derived>& matrix) {
    return SymmetricMatrixRef<Derived>(matrix.derived());
}

template <typename Derived>
SymmetricMatrixRef<const Derived> MakeSymmetricMatrixRef(Eigen::MatrixBase<Derived> const& matrix) {
    return SymmetricMatrixRef<const Derived>(matrix.derived());
}

class IStreamWrapper {
public:
    explicit IStreamWrapper(std::istream* stream);

    template <typename T>
    void ReadValue(T* value) {
        if constexpr (std::is_enum_v<T>) {
            std::underlying_type_t<T> tmp_value;
            ReadValue(&tmp_value);
            *value = static_cast<T>(tmp_value);
        } else {
            static_assert(std::is_arithmetic<T>::value, "NOT AN ARITHMETIC TYPE. WRITE THE SPECIFIC OVERLOAD.");
            stream_->read(reinterpret_cast<char*>(value), sizeof(T));
            CHECK(sizeof(T) == stream_->gcount()) << "Invalid read in stream";
        }
    }

    void ReadValue(std::string* value) {
        std::int32_t size;
        ReadValue(&size);
        value->resize(static_cast<std::size_t>(size));
        CHECK(stream_->read(&(*value)[0], size)) << "Invalid read in stream";
    }

    template <typename T, std::size_t size>
    void ReadValue(std::array<T, size>* value) {
        CHECK(stream_->read(reinterpret_cast<char*>(value->data()), sizeof(T) * size)) << "Invalid read in stream";
    }

    template <typename T>
    void ReadValue(cv::Point_<T>* value) {
        ReadValue(&value->x);
        ReadValue(&value->y);
    }

    template <typename T>
    void ReadValue(cv::Point3_<T>* value) {
        ReadValue(&value->x);
        ReadValue(&value->y);
        ReadValue(&value->z);
    }

    template <typename T, int Rows, int Cols>
    void ReadValue(Eigen::Matrix<T, Rows, Cols>* value) {
        static_assert(Rows != Eigen::Dynamic && Cols != Eigen::Dynamic, "DYNAMIC MATRICES NOT SUPPORTED.");
        CHECK(stream_->read(reinterpret_cast<char*>(value->data()), sizeof(T) * value->size()))
            << "Invalid read in stream";
    }

    template <typename T>
    void ReadValue(Eigen::Quaternion<T>* value) {
        CHECK(stream_->read(reinterpret_cast<char*>(value->coeffs().data()), sizeof(T) * value->coeffs().size()))
            << "Invalid read in stream";
    }

    template <typename Derived>
    void ReadValue(SymmetricMatrixRef<Derived>* p_value) {
        static_assert(
            Derived::RowsAtCompileTime != Eigen::Dynamic && Derived::ColsAtCompileTime != Eigen::Dynamic,
            "DYNAMIC MATRICES NOT SUPPORTED.");

        Derived& matrix = *p_value;
        // Firstly, read the upper triangle
        for (int row = 0; row < matrix.template rows(); ++row) {
            for (int col = row; col < matrix.template cols(); ++col) {
                ReadValue(&matrix(row, col));
            }
        }

        // Secondly, copy the upper to the lower triangle
        matrix.template triangularView<Eigen::StrictlyLower>() = matrix.template transpose();
    }

    template <typename Rotation>
    void ReadValue(Isometry3<Rotation>* value) {
        ReadValue(&value->translation());
        ReadValue(&value->rotation());
    }

    template <typename Rotation>
    void ReadValue(Isometry3Map<Rotation>* value) {
        ReadValue(&value->translation());
        ReadValue(&value->rotation());
    }

    void ReadValue(std::chrono::weeks* value) {
        std::uint32_t week = 0;
        ReadValue(&week);
        *value = std::chrono::weeks(week);
    }

    void ReadValue(std::chrono::nanoseconds* value) {
        std::uint64_t nanoseconds = 0;
        ReadValue(&nanoseconds);
        *value = std::chrono::nanoseconds(nanoseconds);
    }

    template <typename Duration>
    void ReadValue(GPSWeekTime<Duration>* value) {
        ReadValue(&value->week);
        ReadValue(&value->time_in_week);
    }

    void ReadValue(Timestamp_ns* value) {
        GPSWeekTime<std::chrono::nanoseconds> gps_week_time;
        ReadValue(&gps_week_time);
        *value = ToGPSTime(gps_week_time);
    }

private:
    std::istream* stream_;
};

class OStreamWrapper {
public:
    explicit OStreamWrapper(std::ostream* stream);

    template <typename T>
    void WriteValue(T const& value) {
        if constexpr (std::is_enum_v<T>) {
            WriteValue(static_cast<std::underlying_type_t<T>>(value));
        } else {
            static_assert(std::is_arithmetic<T>::value, "NOT AN ARITHMETIC TYPE. WRITE THE SPECIFIC OVERLOAD.");
            stream_->write(reinterpret_cast<const char*>(&value), sizeof(T));
        }
    }

    void WriteValue(std::string const& value) {
        auto size = static_cast<std::int32_t>(value.size());
        WriteValue(size);
        stream_->write(value.c_str(), size);
    }

    template <typename T, std::size_t size>
    void WriteValue(std::array<T, size> const& value) {
        stream_->write(reinterpret_cast<const char*>(value.data()), sizeof(T) * size);
    }

    template <typename T>
    void WriteValue(cv::Point_<T> const& value) {
        WriteValue(value.x);
        WriteValue(value.y);
    }

    template <typename T>
    void WriteValue(cv::Point3_<T> const& value) {
        WriteValue(value.x);
        WriteValue(value.y);
        WriteValue(value.z);
    }

    template <typename T, int Rows, int Cols>
    void WriteValue(Eigen::Matrix<T, Rows, Cols> const& value) {
        static_assert(Rows != Eigen::Dynamic && Cols != Eigen::Dynamic, "DYNAMIC MATRICES NOT SUPPORTED.");

        stream_->write(reinterpret_cast<const char*>(value.data()), sizeof(T) * value.size());
    }

    template <typename T>
    void WriteValue(Eigen::Quaternion<T> const& value) {
        stream_->write(reinterpret_cast<const char*>(value.coeffs().data()), sizeof(T) * value.coeffs().size());
    }

    template <typename Derived>
    void WriteValue(SymmetricMatrixRef<const Derived> const& value) {
        static_assert(
            Derived::RowsAtCompileTime != Eigen::Dynamic && Derived::ColsAtCompileTime != Eigen::Dynamic,
            "DYNAMIC MATRICES NOT SUPPORTED.");

        Derived const& matrix = value;

        for (int row = 0; row < matrix.template rows(); ++row) {
            for (int col = row; col < matrix.template cols(); ++col) {
                WriteValue(matrix(row, col));
            }
        }
    }

    template <typename Rotation>
    void WriteValue(Isometry3<Rotation> const& value) {
        WriteValue(value.translation());
        WriteValue(value.rotation());
    }

    template <typename Rotation>
    void WriteValue(Isometry3Map<Rotation> const& value) {
        WriteValue(value.translation());
        WriteValue(value.rotation());
    }

    void WriteValue(std::chrono::weeks const& value) {
        static_assert(sizeof(std::uint32_t) >= sizeof(std::chrono::weeks::rep));
        WriteValue(static_cast<std::uint32_t>(value.count()));
    }

    void WriteValue(std::chrono::nanoseconds const& value) {
        static_assert(sizeof(std::uint64_t) >= sizeof(std::chrono::nanoseconds::rep));
        WriteValue(static_cast<std::uint64_t>(value.count()));
    }

    template <typename Duration>
    void WriteValue(GPSWeekTime<Duration> const& value) {
        WriteValue(value.week);
        WriteValue(value.time_in_week);
    }

    void WriteValue(Timestamp_ns const& value) { WriteValue(ToGPSWeekTime(value)); }

private:
    std::ostream* stream_;
};

}  // namespace io

}  // namespace nie
