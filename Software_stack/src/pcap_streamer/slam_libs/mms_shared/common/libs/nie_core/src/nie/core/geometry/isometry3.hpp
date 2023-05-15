/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <iomanip>
#include <ostream>

#include <Eigen/Geometry>

#include <ratio>
#include <type_traits>

// An isometry is a distance preserving transformation between metric spaces. It contains a translation and rotation.
// The class assumes that rotations are normalized (for performance reasons).
//
// https://en.wikipedia.org/wiki/Isometry

namespace nie {

namespace detail {

// We effectively support matrices, quaternions and maps to either.

template <typename Derived>
inline auto InverseRotation(Eigen::MatrixBase<Derived> const& r) {
    return r.transpose();
}

template <typename Derived>
inline auto InverseRotation(Eigen::QuaternionBase<Derived> const& r) {
    return r.conjugate();
}

template <typename T, class Enabled = void>
struct EigenTraits {};

template <typename Derived>
struct EigenTraits<
        Derived,
        std::enable_if_t<
                std::is_base_of_v<Eigen::MatrixBase<std::remove_const_t<Derived>>, std::remove_const_t<Derived>>>> {
    template <typename NewType>
    using CastType = Eigen::Matrix<NewType, Derived::RowsAtCompileTime, Derived::ColsAtCompileTime>;

    template <typename DerivedLeft, typename DerivedRight>
    static bool Equals(Eigen::MatrixBase<DerivedLeft> const& lhs, Eigen::QuaternionBase<DerivedRight> const& rhs) {
        return lhs == rhs.toRotationMatrix();
    }

    template <typename DerivedLeft, typename DerivedRight>
    static bool Equals(Eigen::MatrixBase<DerivedLeft> const& lhs, Eigen::MatrixBase<DerivedRight> const& rhs) {
        return lhs == rhs;
    }

    template <typename DerivedLeft, typename DerivedRight>
    static bool IsApprox(
            Eigen::MatrixBase<DerivedLeft> const& lhs,
            Eigen::MatrixBase<DerivedRight> const& rhs,
            typename DerivedLeft::Scalar epsilon) {
        return lhs.isApprox(rhs, epsilon);
    }

    template <typename DerivedLeft, typename DerivedRight>
    static bool IsApprox(
            Eigen::MatrixBase<DerivedLeft> const& lhs,
            Eigen::QuaternionBase<DerivedRight> const& rhs,
            typename DerivedLeft::Scalar epsilon) {
        return lhs.isApprox(rhs.toRotationMatrix(), epsilon);
    }
};

template <typename Derived>
struct EigenTraits<
        Derived,
        std::enable_if_t<
                std::is_base_of_v<Eigen::QuaternionBase<std::remove_const_t<Derived>>, std::remove_const_t<Derived>>>> {
    template <typename NewType>
    using CastType = Eigen::Quaternion<NewType>;

    template <typename DerivedLeft, typename DerivedRight>
    static bool Equals(Eigen::QuaternionBase<DerivedLeft> const& lhs, Eigen::QuaternionBase<DerivedRight> const& rhs) {
        return lhs.coeffs() == rhs.coeffs();
    }

    template <typename DerivedLeft, typename DerivedRight>
    static bool Equals(Eigen::QuaternionBase<DerivedLeft> const& lhs, Eigen::MatrixBase<DerivedRight> const& rhs) {
        return lhs.toRotationMatrix() == rhs;
    }

    template <typename DerivedLeft, typename DerivedRight>
    static bool IsApprox(
            Eigen::QuaternionBase<DerivedLeft> const& lhs,
            Eigen::QuaternionBase<DerivedRight> const& rhs,
            typename DerivedLeft::Scalar epsilon) {
        return lhs.isApprox(rhs, epsilon);
    }

    template <typename DerivedLeft, typename DerivedRight>
    static bool IsApprox(
            Eigen::QuaternionBase<DerivedLeft> const& lhs,
            Eigen::MatrixBase<DerivedRight> const& rhs,
            typename DerivedLeft::Scalar epsilon) {
        return lhs.toRotationMatrix().isApprox(rhs, epsilon);
    }
};

template <typename T, typename U>
struct SameCvQualifier {
private:
    using R = std::remove_reference_t<T>;
    using U0 = std::remove_cv_t<U>;
    using U1 = std::conditional_t<std::is_const_v<R>, std::add_const_t<U0>, U0>;
    using U2 = std::conditional_t<std::is_volatile_v<R>, std::add_volatile_t<U1>, U1>;
    using U3 = std::conditional_t<std::is_lvalue_reference_v<T>, std::add_lvalue_reference_t<U2>, U2>;
    using U4 = std::conditional_t<std::is_rvalue_reference_v<T>, std::add_rvalue_reference_t<U3>, U3>;

public:
    using type = U4;
};

}  // namespace detail

template <typename Rotation_>
class Isometry3;

// Note: Can be used for expressions as well but they would be inefficient due to the nature of having two properties
// (translation and rotation). Expressions would be expanded multiple times.
/// @brief The base Isometry3 logic for both the Isometry3 and Isometry3Map. Used for interop as well in so far Eigen
/// supports this interop.
/// @details An isometry is a distance preserving transformation between metric spaces. It contains a translation and
/// rotation. The class assumes that rotations are normalized (for performance reasons).
template <typename Derived, typename Rotation_>
class Isometry3Base {
public:
    using Rotation = Rotation_;
    using Scalar = typename Rotation_::Scalar;
    using Translation = typename detail::SameCvQualifier<Rotation, Eigen::Matrix<Scalar, 3, 1>>::type;
    using TranslationMap = Eigen::Map<Translation>;
    using RotationMap = Eigen::Map<Rotation>;
    using ScalarPointer = typename TranslationMap::PointerType;
    using NonConstScalar = std::remove_const_t<Scalar>;
    using NonConstTranslation = std::remove_const_t<Translation>;
    using NonConstRotation = std::remove_const_t<Rotation>;
    using NonConstScalarPointer = NonConstScalar*;
    using NonConstVector = NonConstTranslation;

    /// @brief T_left * v_right
    template <typename DerivedMatrix>
    inline NonConstVector Transform(Eigen::MatrixBase<DerivedMatrix> const& right) const {
        return *this * right;
    }

    /// @brief T_left^-1 * v_right
    template <typename DerivedMatrix>
    inline NonConstVector TransformInverseLeft(Eigen::MatrixBase<DerivedMatrix> const& right) const {
        return detail::InverseRotation(rotation()) * (right - translation());
    }

    /// @brief T_left * T_right
    template <typename OtherDerived, typename OtherRotation>
    inline Isometry3<NonConstRotation> Transform(Isometry3Base<OtherDerived, OtherRotation> const& right) const {
        return *this * right;
    }

    /// @brief T_left^-1 * T_right
    template <typename OtherDerived, typename OtherRotation>
    inline Isometry3<NonConstRotation> TransformInverseLeft(
            Isometry3Base<OtherDerived, OtherRotation> const& right) const {
        NonConstRotation temp = detail::InverseRotation(rotation());
        return {temp * (right.translation() - translation()), temp * right.rotation()};
    }

    /// @brief T_left * T_right^-1
    template <typename OtherDerived, typename OtherRotation>
    inline Isometry3<NonConstRotation> TransformInverseRight(
            Isometry3Base<OtherDerived, OtherRotation> const& right) const {
        NonConstRotation temp = rotation() * detail::InverseRotation(right.rotation());
        return {translation() - temp * right.translation(), temp};
    }

    /// @brief Naive difference between left and right. Calculated as (t_left - t_right, R_left * R_right^-1)
    template <typename OtherDerived, typename OtherRotation>
    inline Isometry3<NonConstRotation> Delta(Isometry3Base<OtherDerived, OtherRotation> const& right) const {
        return {translation() - right.translation(), rotation() * detail::InverseRotation(right.rotation())};
    }

    /// @brief T^-1
    inline Isometry3<NonConstRotation> Inversed() const {
        auto temp = detail::InverseRotation(derived().rotation());
        return {temp * -derived().translation(), temp};
    }

    /// @brief In place T^-1
    inline Derived& Inverse() {
        derived() = Inversed();
        return derived();
    }

    template <typename OtherDerived, typename OtherRotation>
    inline bool isApprox(
            Isometry3Base<OtherDerived, OtherRotation> const& other,
            Scalar precision = Eigen::NumTraits<Scalar>::dummy_precision()) const {
        return translation().isApprox(other.translation(), precision) &&
               detail::EigenTraits<Rotation>::IsApprox(rotation(), other.rotation(), precision);
    }

    /// @brief Creates a 4x4 transformation matrix representing this isometry.
    inline Eigen::Transform<Scalar, 3, Eigen::Isometry> ToTransform() const {
        Eigen::Transform<Scalar, 3, Eigen::Isometry> t;
        t = Eigen::Translation<Scalar, 3>(translation());
        t.rotate(rotation());
        return t;
    }

    /// @brief T_left * v_right
    template <typename DerivedMatrix>
    inline NonConstVector operator*(Eigen::MatrixBase<DerivedMatrix> const& right) const {
        return (rotation() * right) + translation();
    }

    /// @brief T_left *= T_right
    template <typename OtherDerived, typename OtherRotation>
    inline Derived& operator*=(Isometry3Base<OtherDerived, OtherRotation> const& right) {
        translation() = (rotation() * right.translation()) + translation();
        rotation() = rotation() * right.rotation();
        return derived();
    }

    /// @brief T_left * T_right
    template <typename OtherDerived, typename OtherRotation>
    inline Isometry3<NonConstRotation> operator*(Isometry3Base<OtherDerived, OtherRotation> const& right) const {
        Isometry3<NonConstRotation> left(*this);
        left *= right;
        return left;
    }

    // Can't just return auto because of the Eigen template expressions. That would create an object where the
    // translation template parameter would be a cast expression.
    /// @brief Allows casting between scalar types (double, float, etc.).
    template <typename NewType>
    Isometry3<typename detail::EigenTraits<NonConstRotation>::template CastType<NewType>> cast() const {
        return {translation().template cast<NewType>(), rotation().template cast<NewType>()};
    }

    inline Derived const& derived() const { return *static_cast<Derived const*>(this); }
    inline Derived& derived() { return *static_cast<Derived*>(this); }
    inline auto const& translation() const { return derived().translation(); }
    inline auto& translation() { return derived().translation(); }
    inline auto const& rotation() const { return derived().rotation(); }
    inline auto& rotation() { return derived().rotation(); }
};

/// @brief An isometry that takes value types.
/// @details An isometry is a distance preserving transformation between metric spaces. It contains a translation and
/// rotation. The class assumes that rotations are normalized (for performance reasons).
template <typename Rotation_>
class Isometry3 : public Isometry3Base<Isometry3<Rotation_>, Rotation_> {
public:
    using Base = Isometry3Base<Isometry3<Rotation_>, Rotation_>;
    using typename Base::Rotation;
    using typename Base::Scalar;
    using typename Base::Translation;

    Isometry3() = default;
    template <typename OtherDerived, typename OtherRotation>
    explicit Isometry3(Isometry3Base<OtherDerived, OtherRotation> const& other)
        : translation_(other.translation()), rotation_(other.rotation()) {}
    Isometry3(Translation translation, Rotation rotation)
        : translation_(std::move(translation)), rotation_(std::move(rotation)) {}
    /// @brief Construct an Isometry3 from any 3x4 or 4x4 Eigen matrix representations.
    template <typename Derived>
    explicit Isometry3(Eigen::MatrixBase<Derived> const& t)
        : translation_(t.template block<3, 1>(0, 3)), rotation_(t.template block<3, 3>(0, 0)) {
        // If the matrix is too small, Eigen will complain through the initialize list. Otherwise if dimensions don't
        // match the static_assert will complain.
        static_assert(
                (Eigen::MatrixBase<Derived>::RowsAtCompileTime == 3 ||
                 Eigen::MatrixBase<Derived>::RowsAtCompileTime == 4) &&
                        Eigen::MatrixBase<Derived>::ColsAtCompileTime == 4,
                "Isometry3(): Input matrix should be 3x4 or 4x4");
    }

    // The compiler automatically generates the assignment operator. The generated one will overrule the operator of the
    // base class. This results in a copy only being available for identical classes and not for any Isometry3Base type
    // class. We have to define it in each derived class.
    template <typename OtherDerived, typename OtherRotation>
    inline Isometry3& operator=(Isometry3Base<OtherDerived, OtherRotation> const& right) {
        translation() = right.translation();
        rotation() = right.rotation();
        return *this;
    }

    template <typename OtherDerived, typename OtherRotation>
    inline bool operator==(Isometry3Base<OtherDerived, OtherRotation> const& right) const {
        return translation() == right.translation() &&
               detail::EigenTraits<Rotation>::Equals(rotation(), right.rotation());
    }
    template <typename OtherDerived, typename OtherRotation>
    inline bool operator!=(Isometry3Base<OtherDerived, OtherRotation> const& right) const {
        return !(*this == right);
    }

    inline Translation const& translation() const { return translation_; }
    inline Translation& translation() { return translation_; }
    inline Rotation const& rotation() const { return rotation_; }
    inline Rotation& rotation() { return rotation_; }

    static Isometry3 Identity() { return Isometry3{Translation::Zero(), Rotation::Identity()}; }
    static Isometry3 FromTranslation(Translation translation) { return {std::move(translation), Rotation::Identity()}; }
    static Isometry3 FromRotation(Rotation rotation) { return {Translation::Zero(), rotation}; }

private:
    Translation translation_;
    Rotation rotation_;
};

/// @brief An isometry that takes pointers or pointer maps.
/// @details An isometry is a distance preserving transformation between metric spaces. It contains a translation and
/// rotation. The class assumes that rotations are normalized (for performance reasons).
template <typename Rotation_>
class Isometry3Map : public Isometry3Base<Isometry3Map<Rotation_>, Rotation_> {
public:
    using Base = Isometry3Base<Isometry3Map<Rotation_>, Rotation_>;
    using typename Base::Rotation;
    using typename Base::RotationMap;
    using typename Base::Scalar;
    using typename Base::ScalarPointer;
    using typename Base::Translation;
    using typename Base::TranslationMap;

    Isometry3Map() = delete;
    Isometry3Map(Isometry3Map const&) = delete;
    Isometry3Map(ScalarPointer translation, ScalarPointer rotation) : translation_(translation), rotation_(rotation) {}
    Isometry3Map(TranslationMap translation, RotationMap rotation)
        : translation_(std::move(translation)), rotation_(std::move(rotation)) {}

    // The compiler automatically generates the assignment operator. The generated one will overrule the operator of the
    // base class. This results in a copy only being available for identical classes and not for any Isometry3Base type
    // class. We have to define it in each derived class.
    template <typename OtherDerived, typename OtherRotation>
    inline Isometry3Map& operator=(Isometry3Base<OtherDerived, OtherRotation> const& right) {
        translation() = right.translation();
        rotation() = right.rotation();
        return *this;
    }

    template <typename OtherDerived, typename OtherRotation>
    inline bool operator==(Isometry3Base<OtherDerived, OtherRotation> const& right) const {
        return translation() == right.translation() &&
               detail::EigenTraits<Rotation>::Equals(rotation(), right.rotation());
    }
    template <typename OtherDerived, typename OtherRotation>
    inline bool operator!=(Isometry3Base<OtherDerived, OtherRotation> const& right) const {
        return !(*this == right);
    }

    inline TranslationMap const& translation() const { return translation_; }
    inline TranslationMap& translation() { return translation_; }
    inline RotationMap const& rotation() const { return rotation_; }
    inline RotationMap& rotation() { return rotation_; }

private:
    TranslationMap translation_;
    RotationMap rotation_;
};

// The convention used below is closest to Eigen and OpenCV without running into too many upper/lower case issues.
// The idea is to keep the names short as well (kind of the point of usings/typedefs).
// So Isometry3MapQuaterniond => Isometry3Mapqd.

using Isometry3qd = Isometry3<Eigen::Quaterniond>;
using Isometry3qf = Isometry3<Eigen::Quaternionf>;
template <typename Scalar>
using Isometry3q = Isometry3<Eigen::Quaternion<Scalar>>;

using Isometry3Mapqd = Isometry3Map<Eigen::Quaterniond>;
using Isometry3Mapqf = Isometry3Map<Eigen::Quaternionf>;
template <typename Scalar>
using Isometry3Mapq = Isometry3Map<Eigen::Quaternion<Scalar>>;

using Isometry3md = Isometry3<Eigen::Matrix3d>;
using Isometry3mf = Isometry3<Eigen::Matrix3f>;
template <typename Scalar>
using Isometry3m = Isometry3<Eigen::Matrix<Scalar, 3, 3>>;

}  // namespace nie

template <typename DerivedIsometry, typename Scalar>
std::ostream& operator<<(
        std::ostream& os, nie::Isometry3Base<DerivedIsometry, Eigen::Quaternion<Scalar>> const& isometry) {
    os << std::setfill(' ') << "[" << isometry.translation().transpose() << "] ["
       << isometry.rotation().coeffs().transpose() << "]";
    return os;
}

template <typename DerivedIsometry, typename Scalar>
std::ostream& operator<<(
        std::ostream& os, nie::Isometry3Base<DerivedIsometry, Eigen::Matrix<Scalar, 3, 3>> const& isometry) {
    //
    os << std::setfill(' ') << isometry.ToTransform().matrix();
    return os;
}
