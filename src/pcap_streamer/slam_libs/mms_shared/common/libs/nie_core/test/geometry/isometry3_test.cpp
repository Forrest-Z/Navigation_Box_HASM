/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <tuple>

#include <gtest/gtest.h>
#include <nie/core/algorithm.hpp>
#include <nie/core/geometry/isometry3.hpp>

// The constructions in the Isometry3Test constructor are compile time tests. These check proper creation of various
// isometry types. We check construction based on pointers, Eigen types, and copy constructor from the base isometry
// (map into value, quat into mat). Note that mat into quat may not work but we don't care (not all conversions are
// supported by Eigen).
class Isometry3Test : public testing::Test {
public:
    Isometry3Test()
        : tr1(Eigen::Vector3d{0.0, 0.0, 1.0}),
          rt1(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitX())),
          tr2(Eigen::Vector3d{1.0, -2.0, 1.0}),
          rt2(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitY())),
          ptr1(tr1.data()),
          prt1(rt1.coeffs().data()),
          ptr2(tr2.data()),
          prt2(rt2.coeffs().data()),
          cptr1(ptr1),
          cprt1(prt1),
          cptr2(ptr2),
          cprt2(prt2),
          ivqi(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity()),
          imq1(ptr1, prt1),
          imq2(Eigen::Map<Eigen::Vector3d>(ptr2), Eigen::Map<Eigen::Quaterniond>(prt2)),
          ivq1(tr1, rt1),
          ivq2(imq2),
          ivm1(tr1, rt1.toRotationMatrix()),
          ivm2(ivq2),
          cimq1(cptr1, cprt1),
          cimq2(cptr2, cprt2) {}

    // Translations and rotations 1 and 2 on all forms using quaternions.
    Eigen::Vector3d tr1;
    Eigen::Quaterniond rt1;
    Eigen::Vector3d tr2;
    Eigen::Quaterniond rt2;
    double *ptr1, *prt1;
    double *ptr2, *prt2;
    double const* const cptr1;
    double const* const cprt1;
    double const* const cptr2;
    double const* const cprt2;

    // Value quaternion identity
    nie::Isometry3<Eigen::Quaterniond> ivqi;
    // Map quaternion 1
    nie::Isometry3Map<Eigen::Quaterniond> imq1;
    // Map quaternion 2
    nie::Isometry3Map<Eigen::Quaterniond> imq2;
    // Value quaternion 1
    nie::Isometry3<Eigen::Quaterniond> ivq1;
    // Value quaternion 2
    nie::Isometry3<Eigen::Quaterniond> ivq2;
    // Value matrix 1
    nie::Isometry3<Eigen::Matrix3d> ivm1;
    // Value matrix 2
    nie::Isometry3<Eigen::Matrix3d> ivm2;
    // Map const quaternion 1
    nie::Isometry3Map<const Eigen::Quaterniond> cimq1;
    // Map const quaternion 2
    nie::Isometry3Map<const Eigen::Quaterniond> cimq2;
};

constexpr double kEpsilon = 1.0e-15;

template <typename DerivedIsometry1, typename Rotation1, typename DerivedIsometry2, typename Rotation2>
void AssertIdentical(
    nie::Isometry3Base<DerivedIsometry1, Rotation1> const& isometry1,
    nie::Isometry3Base<DerivedIsometry2, Rotation2> const& isometry2,
    double precision = kEpsilon) {
    ASSERT_NEAR((isometry1.translation() - isometry2.translation()).norm(), 0.0, precision);
    // Whatever rotation type we get should be castable to a 3x3 matrix and equal the identity.
    ASSERT_NEAR((Eigen::Matrix3d(isometry1.rotation()) - Eigen::Matrix3d(isometry2.rotation())).norm(), 0.0, precision);

    ASSERT_TRUE(isometry1.isApprox(isometry2, precision));
}

template <typename Derived1, typename Derived2>
void AssertIdentical(Eigen::MatrixBase<Derived1> const& one, Eigen::MatrixBase<Derived2> const& two) {
    ASSERT_NEAR((one - two).norm(), 0.0, kEpsilon);
}

// **********************
// Value types quaternion
// **********************

TEST_F(Isometry3Test, QatTransformAndInverseIsometry) {
    // We check if transform and inverse work by canceling each other out. This should compare to identity.
    AssertIdentical(ivq1.Inversed().Transform(ivq1), ivqi);
}

TEST_F(Isometry3Test, QatTransformInverseLeftIsometry) {
    // Should be identical. Left side is more efficient.
    AssertIdentical(ivq1.TransformInverseLeft(ivq2), ivq1.Inversed().Transform(ivq2));
}

TEST_F(Isometry3Test, QatTransformInverseRightIsometry) {
    // Should be identical. Left side is more efficient.
    AssertIdentical(ivq1.TransformInverseRight(ivq2), ivq1.Transform(ivq2.Inversed()));
}

TEST_F(Isometry3Test, QatTransformAndInverseVector) {
    Eigen::Vector3d p{1.0, 1.0, 1.0};
    // We check if transform and inverse work by canceling each other out. This should compare to the original.
    AssertIdentical(ivq1.TransformInverseLeft(ivq1.Transform(p)), p);
}

TEST_F(Isometry3Test, QatTransformInverseLeftVector) {
    Eigen::Vector3d p{1.0, 1.0, 1.0};
    // Should be identical. Left side is more efficient.
    AssertIdentical(ivq1.TransformInverseLeft(p), ivq1.Inversed().Transform(p));
}

TEST_F(Isometry3Test, QatDelta) {
    // Should be identical to identity if comparing directly with itself.
    AssertIdentical(ivq1.Delta(ivq1), ivqi);
}

TEST_F(Isometry3Test, QatMatrix) {
    AssertIdentical((ivq1.ToTransform() * ivq1.Inversed().ToTransform()).matrix(), Eigen::Matrix4d::Identity());
}

TEST_F(Isometry3Test, QatCopy) {
    nie::Isometry3<Eigen::Matrix3d> ivm;
    ivm = ivq1;

    // Should be identical to identity if comparing directly with itself.
    AssertIdentical(ivm, ivq1);
}

TEST_F(Isometry3Test, QatIdentity) { AssertIdentical(nie::Isometry3<Eigen::Quaterniond>::Identity(), ivqi); }

TEST_F(Isometry3Test, QatFromTranslation) {
    AssertIdentical(nie::Isometry3<Eigen::Quaterniond>::FromTranslation(Eigen::Vector3d::Zero()), ivqi);
}

TEST_F(Isometry3Test, QatFromRotation) {
    AssertIdentical(nie::Isometry3<Eigen::Quaterniond>::FromRotation(Eigen::Quaterniond::Identity()), ivqi);
}

// **********************
// Map types quaternion
// **********************

TEST_F(Isometry3Test, QMapTransformAndInverseIsometry) {
    // We check if transform and inverse work by canceling each other out. This should compare to identity.
    AssertIdentical(imq1.Inversed().Transform(imq1), ivqi);
}

TEST_F(Isometry3Test, QMapTransformInverseLeftIsometry) {
    // Should be identical. Left side is more efficient.
    AssertIdentical(imq1.TransformInverseLeft(imq2), imq1.Inversed().Transform(imq2));
}

TEST_F(Isometry3Test, QMapTransformInverseRightIsometry) {
    // Should be identical. Left side is more efficient.
    AssertIdentical(imq1.TransformInverseRight(imq2), imq1.Transform(imq2.Inversed()));
}

TEST_F(Isometry3Test, QMapTransformAndInverseVector) {
    Eigen::Vector3d p{1.0, 1.0, 1.0};
    // We check if transform and inverse work by canceling each other out. This should compare to the original.
    AssertIdentical(imq1.TransformInverseLeft(imq1.Transform(p)), p);
}

TEST_F(Isometry3Test, QMapTransformInverseLeftVector) {
    Eigen::Vector3d p{1.0, 1.0, 1.0};
    // Should be identical. Left side is more efficient.
    AssertIdentical(imq1.TransformInverseLeft(p), imq1.Inversed().Transform(p));
}

TEST_F(Isometry3Test, QMapDelta) {
    // Should be identical to identity if comparing directly with itself.
    AssertIdentical(imq1.Delta(imq1), ivqi);
}

TEST_F(Isometry3Test, QMapMatrix) {
    AssertIdentical((imq1.ToTransform() * imq1.Inversed().ToTransform()).matrix(), Eigen::Matrix4d::Identity());
}

TEST_F(Isometry3Test, QMapCopy) {
    double t[3], r[4];
    nie::Isometry3Map<Eigen::Quaterniond> imq(t, r);
    imq = ivq1;

    // Should be identical to identity if comparing directly with itself.
    AssertIdentical(imq, ivq1);
}

// **********************
// Value types matrix
// **********************

TEST_F(Isometry3Test, MatTransformAndInverseIsometry) {
    // We check if transform and inverse work by canceling each other out. This should compare to identity.
    AssertIdentical(ivm1.Inversed().Transform(ivm1), ivqi);
}

TEST_F(Isometry3Test, MatTransformInverseLeftIsometry) {
    // Should be identical. Left side is more efficient.
    AssertIdentical(ivm1.TransformInverseLeft(ivm2), ivm1.Inversed().Transform(ivm2));
}

TEST_F(Isometry3Test, MatTransformInverseRightIsometry) {
    // Should be identical. Left side is more efficient.
    AssertIdentical(ivm1.TransformInverseRight(ivm2), ivm1.Transform(ivm2.Inversed()));
}

TEST_F(Isometry3Test, MatTransformAndInverseVector) {
    Eigen::Vector3d p{1.0, 1.0, 1.0};
    // We check if transform and inverse work by canceling each other out. This should compare to the original.
    AssertIdentical(ivm1.TransformInverseLeft(ivm1.Transform(p)), p);
}

TEST_F(Isometry3Test, MatTransformInverseLeftVector) {
    Eigen::Vector3d p{1.0, 1.0, 1.0};
    // Should be identical. Left side is more efficient.
    AssertIdentical(ivm1.TransformInverseLeft(p), ivm1.Inversed().Transform(p));
}

TEST_F(Isometry3Test, MatDelta) {
    // Should be identical to identity if comparing directly with itself.
    AssertIdentical(ivm1.Delta(ivm1), ivqi);
}

TEST_F(Isometry3Test, MatMatrix) {
    AssertIdentical((ivm1.ToTransform() * ivm1.Inversed().ToTransform()).matrix(), Eigen::Matrix4d::Identity());
}

TEST_F(Isometry3Test, MatIdentity) { AssertIdentical(nie::Isometry3<Eigen::Matrix3d>::Identity(), ivqi); }

TEST_F(Isometry3Test, MatFromMatrix) {
    AssertIdentical(nie::Isometry3<Eigen::Matrix3d>(ivm2.ToTransform().matrix()), ivm2);
}

TEST_F(Isometry3Test, MatFromTranslation) {
    AssertIdentical(nie::Isometry3<Eigen::Matrix3d>::FromTranslation(Eigen::Vector3d::Zero()), ivqi);
}

TEST_F(Isometry3Test, MatFromRotation) {
    AssertIdentical(nie::Isometry3<Eigen::Matrix3d>::FromRotation(Eigen::Matrix3d::Identity()), ivqi);
}

// **********************
// Map type matrix
// **********************

TEST_F(Isometry3Test, MMapCopy) {
    double t[3], r[9];
    nie::Isometry3Map<Eigen::Matrix3d> imm(t, r);
    imm = ivm1;

    // Should be identical to identity if comparing directly with itself.
    AssertIdentical(imm, ivm1);
}

// **********************
// Mixed types
// **********************

TEST_F(Isometry3Test, MixTransformInverseLeftIsometry) {
    // Should be identical. Left side is more efficient.
    AssertIdentical(ivq1.TransformInverseLeft(imq2), imq1.Inversed().Transform(ivq2));
}

TEST_F(Isometry3Test, MixTransformInverseRightIsometry) {
    // Should be identical. Left side is more efficient.
    AssertIdentical(ivq1.TransformInverseRight(imq2), imq1.Transform(ivq2.Inversed()));
}

TEST_F(Isometry3Test, ConstMixTransformInverseLeftIsometry) {
    // Should be identical. Left side is more efficient.
    AssertIdentical(ivq1.TransformInverseLeft(cimq2), cimq1.Inversed().Transform(ivq2));
}

TEST_F(Isometry3Test, ConstMixTransformInverseRightIsometry) {
    // Should be identical. Left side is more efficient.
    AssertIdentical(ivq1.TransformInverseRight(cimq2), cimq1.Transform(ivq2.Inversed()));
}

// With the matrices on the left side this interaction should compile. The other way around not.
TEST_F(Isometry3Test, MatMixTransformInverseLeftIsometry) {
    // Should be identical. Left side is more efficient.
    AssertIdentical(ivm1.TransformInverseLeft(imq2), ivm1.Inversed().Transform(imq2));
}

TEST_F(Isometry3Test, MixCast) {
    Eigen::Matrix4f t = ivq1.ToTransform().matrix().cast<float>();

    // Should be identical.
    AssertIdentical(nie::Isometry3qd(t.cast<double>()), ivq1.cast<float>().cast<double>(), 1.0e-07);
}

// ******************************
// Identical floating point types
// ******************************

TEST_F(Isometry3Test, ComparisonOperator) {
    // Value identity
    auto const& vi = ivqi;
    // Collection of values 1
    std::tuple<nie::Isometry3Mapqd&, nie::Isometry3qd&, nie::Isometry3md&, nie::Isometry3Map<const Eigen::Quaterniond>&>
        v1c{imq1, ivq1, ivm1, cimq1};
    // Collection of values 2
    std::tuple<nie::Isometry3Mapqd&, nie::Isometry3qd&, nie::Isometry3md&, nie::Isometry3Map<const Eigen::Quaterniond>&>
        v2c{imq2, ivq2, ivm2, cimq2};
    auto all = std::tuple_cat(v1c, v2c);

    // Test equalities
    ASSERT_TRUE(vi == vi);

    ForEach(v1c, [&v1c](auto const& a) {
        ForEach(v1c, [&a](auto const& b) {
            ASSERT_TRUE(a == b) << typeid(a).name() << " should equal " << typeid(b).name();
            ASSERT_TRUE(b == a) << typeid(b).name() << " should equal " << typeid(a).name();
        });
    });

    ForEach(v2c, [&v2c](auto const& a) {
        ForEach(v2c, [&a](auto const& b) {
            ASSERT_TRUE(a == b) << typeid(a).name() << " should equal " << typeid(b).name();
            ASSERT_TRUE(b == a) << typeid(b).name() << " should equal " << typeid(a).name();
        });
    });

    // Test inequalities
    ForEach(all, [&vi](auto const& item) {
        ASSERT_TRUE(item != vi) << typeid(item).name() << " should not equal " << typeid(vi).name();
        ASSERT_TRUE(vi != item) << typeid(vi).name() << " should not equal " << typeid(item).name();
    });

    ForEach(v1c, [&v2c](auto const& a) {
        ForEach(v2c, [&a](auto const& b) {
            ASSERT_TRUE(a != b) << typeid(a).name() << " should not equal " << typeid(b).name();
            ASSERT_TRUE(b != a) << typeid(b).name() << " should not equal " << typeid(a).name();
        });
    });
}

// ******************************
// Test type trait SameCvQualifier
// ******************************

TEST(SameCvQualifierTest, KeepNoCV) {
    using FromType = int;
    using ToType = double;
    using ResultType = nie::detail::SameCvQualifier<FromType, ToType>::type;
    EXPECT_FALSE(std::is_const<ResultType>::value);
    EXPECT_FALSE(std::is_volatile<ResultType>::value);
}

TEST(SameCvQualifierTest, KeepConst) {
    using FromType = const int;
    using ToType = const double;
    using ResultType = nie::detail::SameCvQualifier<FromType, ToType>::type;
    EXPECT_TRUE(std::is_const<ResultType>::value);
    EXPECT_FALSE(std::is_volatile<ResultType>::value);
}

TEST(SameCvQualifierTest, KeepVolatile) {
    using FromType = volatile int;
    using ToType = volatile double;
    using ResultType = nie::detail::SameCvQualifier<FromType, ToType>::type;
    EXPECT_FALSE(std::is_const<ResultType>::value);
    EXPECT_TRUE(std::is_volatile<ResultType>::value);
}

TEST(SameCvQualifierTest, KeepConstKeepVolatile) {
    using FromType = const volatile int;
    using ToType = const volatile double;
    using ResultType = nie::detail::SameCvQualifier<FromType, ToType>::type;
    EXPECT_TRUE(std::is_const<ResultType>::value);
    EXPECT_TRUE(std::is_volatile<ResultType>::value);
}

TEST(SameCvQualifierTest, RemoveConstRemoveVolatile) {
    using FromType = int;
    using ToType = const volatile double;
    using ResultType = nie::detail::SameCvQualifier<FromType, ToType>::type;
    EXPECT_FALSE(std::is_const<ResultType>::value);
    EXPECT_FALSE(std::is_volatile<ResultType>::value);
}

TEST(SameCvQualifierTest, AddConstAddVolatile) {
    using FromType = const volatile int;
    using ToType = double;
    using ResultType = nie::detail::SameCvQualifier<FromType, ToType>::type;
    EXPECT_TRUE(std::is_const<ResultType>::value);
    EXPECT_TRUE(std::is_volatile<ResultType>::value);
}

TEST(SameCvQualifierTest, RemoveConst) {
    using FromType = int;
    using ToType = const double;
    using ResultType = nie::detail::SameCvQualifier<FromType, ToType>::type;
    EXPECT_FALSE(std::is_const<ResultType>::value);
    EXPECT_FALSE(std::is_volatile<ResultType>::value);
}

TEST(SameCvQualifierTest, AddConst) {
    using FromType = const int;
    using ToType = double;
    using ResultType = nie::detail::SameCvQualifier<FromType, ToType>::type;
    EXPECT_TRUE(std::is_const<ResultType>::value);
    EXPECT_FALSE(std::is_volatile<ResultType>::value);
}

TEST(SameCvQualifierTest, RemoveVolatile) {
    using FromType = int;
    using ToType = volatile double;
    using ResultType = nie::detail::SameCvQualifier<FromType, ToType>::type;
    EXPECT_FALSE(std::is_const<ResultType>::value);
    EXPECT_FALSE(std::is_volatile<ResultType>::value);
}

TEST(SameCvQualifierTest, AddVolatile) {
    using FromType = volatile int;
    using ToType = double;
    using ResultType = nie::detail::SameCvQualifier<FromType, ToType>::type;
    EXPECT_FALSE(std::is_const<ResultType>::value);
    EXPECT_TRUE(std::is_volatile<ResultType>::value);
}