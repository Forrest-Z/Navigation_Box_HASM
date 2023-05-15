/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <glog/logging.h>
#include <nie/core/geometry/interpolation.hpp>

#include "isometry3_gtest_helper.hpp"

class InterpolationFixture : public ::testing::Test {
protected:
    double const kPrecision = 1e-12;

    struct InterpolationReference {
        nie::Isometry3qd isom1;
        nie::Isometry3qd isom2;
        nie::Isometry3qd expected;
        double ratio;
    };

    // Reference values generated with numpy-quaternion
    std::vector<InterpolationReference> const test_vector{
        {{{0, 0, 0}, {1, 0, 0, 0}},
         {{1, 2, 3}, {0, 1, 0, 0}},
         {{0.1, 0.2, 0.3}, {0.987688340595138, 0.156434465040231, 0, 0}},
         0.1},
        {{{0, 0, 0}, {1, 0, 0, 0}},
         {{1, 2, 3}, {0, 1, 0, 0}},
         {{0.25, 0.5, 0.75}, {0.923879532511287, 0.38268343236509, 0, 0}},
         0.25},
        {{{0, 0, 0}, {1, 0, 0, 0}},
         {{1, 2, 3}, {0, 1, 0, 0}},
         {{0.5, 1, 1.5}, {0.707106781186548, 0.707106781186547, 0, 0}},
         0.5},
        {{{0, 0, 0}, {1, 0, 0, 0}},
         {{1, 2, 3}, {0, 1, 0, 0}},
         {{0.75, 1.5, 2.25}, {0.38268343236509, 0.923879532511287, 0, 0}},
         0.75},
        {{{0, 0, 0}, {1, 0, 0, 0}},
         {{0, 0, 0}, {0.182574185835055, 0.365148371670111, 0.547722557505166, 0.730296743340221}},
         {{0, 0, 0}, {0.985384525365588, 0.0632644555322975, 0.0948966832984463, 0.126528911064595}},
         0.1234},
    };
};

TEST_F(InterpolationFixture, Reflexivity) {
    nie::Isometry3qd const isom1{{0, 0, 0}, {1, 0, 0, 0}};
    nie::Isometry3qd const& expected = isom1;

    for (double ratio = 0.0; ratio <= 1.0; ratio += 1e-2) {
        nie::Isometry3qd const result{nie::Interpolate(isom1, isom1, ratio)};
        ASSERT_ISOMETRY_NEAR(result, expected, kPrecision);
    }
}

TEST_F(InterpolationFixture, TranslationRatioZero) {
    nie::Isometry3qd const isom1{{0, 0, 0}, {1, 0, 0, 0}};
    nie::Isometry3qd const isom2{{1, 1, 1}, {1, 0, 0, 0}};
    nie::Isometry3qd const expected{{0, 0, 0}, {1, 0, 0, 0}};

    nie::Isometry3qd const result{nie::Interpolate(isom1, isom2, 0.0)};

    ASSERT_ISOMETRY_NEAR(result, expected, kPrecision);
}

TEST_F(InterpolationFixture, TranslationRatioHalf) {
    nie::Isometry3qd const isom1{{0, 0, 0}, {1, 0, 0, 0}};
    nie::Isometry3qd const isom2{{1, 1, 1}, {1, 0, 0, 0}};
    nie::Isometry3qd const expected{{0.5, 0.5, 0.5}, {1, 0, 0, 0}};

    nie::Isometry3qd const result{nie::Interpolate(isom1, isom2, 0.5)};

    ASSERT_ISOMETRY_NEAR(result, expected, kPrecision);
}

TEST_F(InterpolationFixture, TranslationRatioOne) {
    nie::Isometry3qd const isom1{{0, 0, 0}, {1, 0, 0, 0}};
    nie::Isometry3qd const isom2{{1, 1, 1}, {1, 0, 0, 0}};
    nie::Isometry3qd const expected{{1, 1, 1}, {1, 0, 0, 0}};

    nie::Isometry3qd const result{nie::Interpolate(isom1, isom2, 1.0)};

    ASSERT_ISOMETRY_NEAR(result, expected, kPrecision);
}

TEST_F(InterpolationFixture, RotationRatioZero) {
    nie::Isometry3qd const isom1{{0, 0, 0}, {1, 0, 0, 0}};
    nie::Isometry3qd const isom2{{0, 0, 0}, {0, 1, 0, 0}};
    nie::Isometry3qd const expected{{0, 0, 0}, {1, 0, 0, 0}};

    nie::Isometry3qd const result{nie::Interpolate(isom1, isom2, 0.0)};

    ASSERT_ISOMETRY_NEAR(result, expected, kPrecision);
}

TEST_F(InterpolationFixture, RotationRatioHalf) {
    nie::Isometry3qd const isom1{{0, 0, 0}, {1, 0, 0, 0}};
    nie::Isometry3qd const isom2{{0, 0, 0}, {0, 1, 0, 0}};
    nie::Isometry3qd const expected{{0, 0, 0}, {0.707106781186548, 0.707106781186547, 0, 0}};

    nie::Isometry3qd const result{nie::Interpolate(isom1, isom2, 0.5)};

    ASSERT_ISOMETRY_NEAR(result, expected, kPrecision);
}

TEST_F(InterpolationFixture, GenericTest) {
    for (auto const& ref : test_vector) {
        nie::Isometry3qd const result{nie::Interpolate(ref.isom1, ref.isom2, ref.ratio)};
        ASSERT_ISOMETRY_NEAR(result, ref.expected, kPrecision);
    }
}

TEST_F(InterpolationFixture, NormalizedResult) {
    // Test to check if the quaternion remains normalized for many repeated interpolations.
    nie::Isometry3qd isom1{{0, 0, 0}, {0.987688340595138, 0.156434465040231, 0, 0}};
    nie::Isometry3qd const isom2{{0, 0, 0}, {0, 1, 0, 0}};

    for (size_t i = 0; i < 1000; ++i) {
        isom1 = nie::Interpolate(isom1, isom2, 0.5);
        nie::Isometry3qd const isom1_normalized{isom1.translation(), isom1.rotation().normalized()};
        ASSERT_ISOMETRY_NEAR(isom1, isom1_normalized, kPrecision);
    }
}

TEST_F(InterpolationFixture, FailureRatioOutsideInterval) {
    nie::Isometry3qd const isom1{{0, 0, 0}, {1, 0, 0, 0}};
    nie::Isometry3qd const isom2{{0, 0, 0}, {0, 1, 0, 0}};

#ifndef NDEBUG

    LOG(INFO) << "Testing DCHECKS...";

    ASSERT_DEATH(nie::Interpolate(isom1, isom2, -1e-12), "Interpolation ratio must be greater than or equal to 0.0.");
    ASSERT_DEATH(nie::Interpolate(isom1, isom2, 1 + 1e-12), "Interpolation ratio must be lesser than or equal to 1.0.");

    ASSERT_DEATH(
        nie::Interpolate(isom1.rotation(), isom2.rotation(), -1e-12),
        "Interpolation ratio must be greater than or equal to 0.0.");
    ASSERT_DEATH(
        nie::Interpolate(isom1.rotation(), isom2.rotation(), 1 + 1e-12),
        "Interpolation ratio must be lesser than or equal to 1.0.");

    ASSERT_DEATH(
        nie::Interpolate(isom1.translation(), isom2.translation(), -1e-12),
        "Interpolation ratio must be greater than or equal to 0.0.");
    ASSERT_DEATH(
        nie::Interpolate(isom1.translation(), isom2.translation(), 1 + 1e-12),
        "Interpolation ratio must be lesser than or equal to 1.0.");
#endif
}

template <typename Derived>
bool IsSymmetricalMatrix(Eigen::MatrixBase<Derived> const& m) {
    Derived const cov_middle_u = m.template triangularView<Eigen::StrictlyUpper>();
    Derived const cov_middle_l = m.template triangularView<Eigen::StrictlyLower>();
    return cov_middle_u.isApprox(cov_middle_l.transpose());
}

TEST(CovInterpolationTest, BasicTest) {
    using EigenMatrix6d = Eigen::Matrix<double, 6, 6>;

    // clang-format off
    EigenMatrix6d const cov_before = (Eigen::MatrixXd(6,6) <<
        62479.336691, -4055.055614,  6353.020698,         0.000000,         0.000000,         0.000000,
        -4055.055614, 58730.751422,  1747.206013,         0.000000,         0.000000,         0.000000,
         6353.020698,  1747.206013, 16245.868983,         0.000000,         0.000000,         0.000000,
            0.000000,     0.000000,     0.000000, 820318511.779985,   1175826.071448, -41905005.632733,
            0.000000,     0.000000,     0.000000,   1175826.071448, 817265425.922828,  34221074.776537,
            0.000000,     0.000000,     0.000000, -41905005.632733,  34221074.776537,  67368919.974028).finished();

    EigenMatrix6d const cov_after = (Eigen::MatrixXd(6,6) <<
        62201.061443, -3468.466889,  6596.357781,         0.000000,         0.000000,         0.000000,
        -3468.466889, 58176.739248,  1660.961425,         0.000000,         0.000000,         0.000000,
         6596.357781,  1660.961425, 16294.013433,         0.000000,         0.000000,         0.000000,
            0.000000,     0.000000,     0.000000, 821577317.432107,   -376333.519395, -36982475.789606,
            0.000000,     0.000000,     0.000000,   -376333.519395, 818108144.176093,  35875291.929954,
            0.000000,     0.000000,     0.000000, -36982475.789606,  35875291.929954,  67883336.265275).finished();

    EigenMatrix6d const cov_middle_expected = (Eigen::MatrixXd(6,6) <<
        62369.334872, -3883.397113,  6417.380325,         0.000000,         0.000000,         0.000000,
        -3883.397113, 58523.614898,  1716.281974,         0.000000,         0.000000,         0.000000,
         6417.380325,  1716.281974, 16257.736515,         0.000000,         0.000000,         0.000000,
            0.000000,     0.000000,     0.000000, 809898339.554511,    767985.218401, -40019636.840028,
            0.000000,     0.000000,     0.000000,    767985.218401, 806567151.785002,  34465560.778577,
            0.000000,     0.000000,     0.000000, -40019636.840028,  34465560.778577,  67490600.555615).finished();
    // clang-format on
    double const ratio = 0.286476;

    EigenMatrix6d const cov_middle = nie::CovInterpolate(cov_before, cov_after, ratio);

    ASSERT_TRUE(IsSymmetricalMatrix(cov_middle));
    ASSERT_TRUE(cov_middle.isApprox(cov_middle_expected, 1e-6));
}

TEST(InfInterpolationTest, BasicTest) {
    using EigenMatrix6d = Eigen::Matrix<double, 6, 6>;

    // clang-format off
    EigenMatrix6d const inf_before = (Eigen::MatrixXd(6,6) <<
         40948.54590006627404363826, -13764.76179215907541220076, -1928.41561934786500387418,         0.00000000000000000000,         0.00000000000000000000,         0.00000000000000000000,
        -13764.76179215907541220076,  57709.44380074432410765439,  6079.30529216394734248752,         0.00000000000000000000,         0.00000000000000000000,         0.00000000000000000000,
         -1928.41561934786500387418,   6079.30529216394734248752, 13517.96098543929292645771,         0.00000000000000000000,         0.00000000000000000000,         0.00000000000000000000,
             0.00000000000000000000,      0.00000000000000000000,     0.00000000000000000000, 824526339.43336331844329833984, -22577364.53011267632246017456,   3262134.33463941002264618874,
             0.00000000000000000000,      0.00000000000000000000,     0.00000000000000000000, -22577364.53011267632246017456, 945116381.09321045875549316406,  51021590.03543114662170410156,
             0.00000000000000000000,      0.00000000000000000000,     0.00000000000000000000,   3262134.33463941002264618874,  51021590.03543114662170410156,  30992089.92592424526810646057).finished();

    EigenMatrix6d const inf_after = (Eigen::MatrixXd(6,6) <<
         37942.43897774524521082640, -15199.88454414509396883659, -2358.08195149110588317853,         0.00000000000000000000,         0.00000000000000000000,         0.00000000000000000000,
        -15199.88454414509396883659,  56785.82110474810906453058,  5963.64360487465910409810,         0.00000000000000000000,         0.00000000000000000000,         0.00000000000000000000,
         -2358.08195149110588317853,   5963.64360487465910409810, 13246.92385865938376809936,         0.00000000000000000000,         0.00000000000000000000,         0.00000000000000000000,
             0.00000000000000000000,      0.00000000000000000000,     0.00000000000000000000, 832403723.73632037639617919922, -30635394.41831148043274879456,  -2127055.66304523125290870667,
             0.00000000000000000000,      0.00000000000000000000,     0.00000000000000000000, -30635394.41831148043274879456, 943193986.99748694896697998047,  51886571.17434442788362503052,
             0.00000000000000000000,      0.00000000000000000000,     0.00000000000000000000,  -2127055.66304523125290870667,  51886571.17434442788362503052,  32399775.72517402470111846924).finished();

    EigenMatrix6d const inf_middle_expected = (Eigen::MatrixXd(6,6) <<
         39368.14312605556187918410, -14526.02470907721362891607, -2158.27527209213485548389,         0.00000000000000000000,         0.00000000000000000000,        0.00000000000000000000,
        -14526.02470907721362891607,  57223.47863873354799579829,  6013.53720115452961181290,         0.00000000000000000000,         0.00000000000000000000,        0.00000000000000000000,
         -2158.27527209213485548389,   6013.53720115452961181290, 13379.82948264773767732549,         0.00000000000000000000,         0.00000000000000000000,        0.00000000000000000000,
             0.00000000000000000000,      0.00000000000000000000,     0.00000000000000000000, 829439505.51616811752319335938, -26736412.82809397578239440918,   584422.49534356303047388792,
             0.00000000000000000000,      0.00000000000000000000,     0.00000000000000000000, -26736412.82809397950768470764, 945589500.91301333904266357422, 51518407.58915154635906219482,
             0.00000000000000000000,      0.00000000000000000000,     0.00000000000000000000,    584422.49534356279764324427,  51518407.58915154635906219482, 31673145.06620706617832183838).finished();
    // clang-format on
    double const ratio = 0.49797999999999997822;

    EigenMatrix6d inf_middle = nie::InfInterpolate(inf_before, inf_after, ratio);

    ASSERT_TRUE(IsSymmetricalMatrix(inf_middle));
    ASSERT_TRUE(inf_middle.isApprox(inf_middle_expected, std::numeric_limits<double>::epsilon() * 10.));
}
