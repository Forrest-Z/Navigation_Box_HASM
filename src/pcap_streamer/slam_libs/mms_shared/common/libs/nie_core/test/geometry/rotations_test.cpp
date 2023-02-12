/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>

#include <nie/core/constants.hpp>
#include <nie/core/geometry/rotation.hpp>

#include "isometry3_gtest_helper.hpp"

class RotationsFixture : public ::testing::Test {
protected:
    double const kPrecision = 1e-12;

    struct RotationReference {
        double const roll;
        double const pitch;
        double const yaw;
        Eigen::Quaterniond q;
    };

    //clang-format off
    std::vector<RotationReference> test_vector{
        {1.1,
         2.1,
         3.1,
         {-0.444472013106645413849094,
          0.744747859249729859953959,
          -0.24464002866374512379366,
          0.43352798304523870731586}},
        {0,
         0,
         0,
         {
             1,
             0,
             0,
             0,
         }},
        {0,
         0,
         1,
         {
             0.877582561890372758739431,
             0,
             0,
             0.479425538604203005377258,
         }},
        {0,
         0,
         2,
         {
             0.540302305868139765010483,
             0,
             0,
             0.841470984807896504875657,
         }},
        {0,
         1,
         0,
         {
             0.877582561890372758739431,
             0,
             0.479425538604203005377258,
             0,
         }},
        {0,
         1,
         1,
         {
             0.770151152934069882505241,
             0.229848847065930145250334,
             0.420735492403948252437829,
             0.420735492403948252437829,
         }},
        {0,
         1,
         2,
         {
             0.4741598817790378950221,
             0.403422680111334919228483,
             0.259034723999925720061555,
             0.738460262604128780949964,
         }},
        {0,
         2,
         0,
         {
             0.540302305868139765010483,
             0,
             0.841470984807896504875657,
             0,
         }},
        {0,
         2,
         1,
         {
             0.4741598817790378950221,
             0.403422680111334919228483,
             0.738460262604128780949964,
             0.259034723999925720061555,
         }},
        {0,
         2,
         2,
         {
             0.291926581726428879814961,
             0.70807341827357117569619,
             0.454648713412840910219614,
             0.454648713412840910219614,
         }},
        {1,
         0,
         0,
         {
             0.877582561890372758739431,
             0.479425538604203005377258,
             0,
             0,
         }},
        {1,
         0,
         1,
         {
             0.770151152934069882505241,
             0.420735492403948252437829,
             -0.229848847065930145250334,
             0.420735492403948252437829,
         }},
        {1,
         0,
         2,
         {
             0.4741598817790378950221,
             0.259034723999925720061555,
             -0.403422680111334919228483,
             0.738460262604128780949964,
         }},
        {1,
         1,
         0,
         {
             0.770151152934069882505241,
             0.420735492403948252437829,
             0.420735492403948252437829,
             0.229848847065930145250334,
         }},
        {1,
         1,
         1,
         {
             0.565675814532566656467338,
             0.570941471357731877844799,
             0.16751879124639693086074,
             0.570941471357731877844799,
         }},
        {1,
         1,
         2,
         {
             0.222703308099756469573549,
             0.581361065843206015202327,
             -0.126712352430365132738288,
             0.772247711181224261167699,
         }},
        {1,
         2,
         0,
         {
             0.4741598817790378950221,
             0.259034723999925720061555,
             0.738460262604128780949964,
             0.403422680111334919228483,
         }},
        {1,
         2,
         1,
         {
             0.222703308099756497329125,
             0.581361065843206015202327,
             0.523871987039513320461026,
             0.581361065843206126224629,
         }},
        {1,
         2,
         2,
         {
             -0.0832788024517472358354553,
             0.76134994309207115747995,
             0.181021578373343650092409,
             0.616961986980661958313021,
         }},
        {2,
         0,
         0,
         {
             0.540302305868139765010483,
             0.841470984807896504875657,
             0,
             0,
         }},
        {2,
         0,
         1,
         {
             0.4741598817790378950221,
             0.738460262604128780949964,
             -0.403422680111334919228483,
             0.259034723999925720061555,
         }},
        {2,
         0,
         2,
         {
             0.291926581726428879814961,
             0.454648713412840910219614,
             -0.70807341827357117569619,
             0.454648713412840910219614,
         }},
        {2,
         1,
         0,
         {
             0.4741598817790378950221,
             0.738460262604128780949964,
             0.259034723999925720061555,
             0.403422680111334919228483,
         }},
        {2,
         1,
         1,
         {
             0.222703308099756497329125,
             0.772247711181224150145397,
             -0.126712352430365216005015,
             0.581361065843206126224629,
         }},
        {2,
         1,
         2,
         {
             -0.0832788024517472358354553,
             0.616961986980661958313021,
             -0.48143582573791704470878,
             0.616961986980661958313021,
         }},
        {2,
         2,
         0,
         {
             0.291926581726428879814961,
             0.454648713412840910219614,
             0.454648713412840910219614,
             0.70807341827357117569619,
         }},
        {2,
         2,
         1,
         {
             -0.0832788024517472358354553,
             0.616961986980661958313021,
             0.181021578373343622336833,
             0.76134994309207115747995,
         }},
        {2,
         2,
         2,
         {
             -0.438094631339962126137522,
             0.628221448834087370372004,
             -0.136925952400205358516772,
             0.628221448834087259349701,
         }},
    };
    //clang-format on
};

TEST_F(RotationsFixture, InverseConstruction) {
    for (auto const& ref : test_vector) {
        Eigen::Quaterniond result = nie::EulerToQuaternion(ref.roll, ref.pitch, ref.yaw);
        EXPECT_QUATERNION_NEAR(result, result.normalized(), kPrecision);
        EXPECT_QUATERNION_NEAR(result, ref.q, kPrecision);

        // Alternative way of creating a quaternion from rpy. Starting with identity quaternion and incrementally
        //  adding axis rotations in the correct order.
        Eigen::Quaterniond result2{1, 0, 0, 0};  // identity rotation

        result2 = result2 * Eigen::AngleAxisd{ref.roll, Eigen::Vector3d::UnitX()};
        result2 = result2 * Eigen::AngleAxisd{ref.pitch, Eigen::Vector3d::UnitY()};
        result2 = result2 * Eigen::AngleAxisd{ref.yaw, Eigen::Vector3d::UnitZ()};

        EXPECT_QUATERNION_NEAR(result2, result2.normalized(), kPrecision);
        EXPECT_QUATERNION_NEAR(result2, ref.q, kPrecision);
    }
}

TEST_F(RotationsFixture, RpyToQuaternion) {
    for (auto const& ref : test_vector) {
        Eigen::Quaterniond result = nie::EulerToQuaternion(ref.roll, ref.pitch, ref.yaw);
        EXPECT_QUATERNION_NEAR(result, result.normalized(), kPrecision);
        EXPECT_QUATERNION_NEAR(result, ref.q, kPrecision);
    }
}
