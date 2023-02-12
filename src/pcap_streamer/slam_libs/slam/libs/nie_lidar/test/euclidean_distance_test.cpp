/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>
#include <nie/lidar/euclidean_distance.hpp>

struct HasX {
    int x;
};
struct HasY {
    int y;
};
struct HasZ {
    int z;
};
struct HasXyz {
    int x, y, z;
};

TEST(HasAttr, X) {
    bool result;
    result = nie::HasAttr<HasX, nie::GetAttrTypeX>::value;
    ASSERT_EQ(result, true);

    result = nie::HasAttr<HasY, nie::GetAttrTypeX>::value;
    ASSERT_EQ(result, false);

    result = nie::HasAttr<HasZ, nie::GetAttrTypeX>::value;
    ASSERT_EQ(result, false);
}

TEST(HasAttr, Y) {
    bool result;
    result = nie::HasAttr<HasX, nie::GetAttrTypeY>::value;
    ASSERT_EQ(result, false);

    result = nie::HasAttr<HasY, nie::GetAttrTypeY>::value;
    ASSERT_EQ(result, true);

    result = nie::HasAttr<HasZ, nie::GetAttrTypeY>::value;
    ASSERT_EQ(result, false);
}

TEST(HasAttr, Z) {
    bool result;
    result = nie::HasAttr<HasX, nie::GetAttrTypeZ>::value;
    ASSERT_EQ(result, false);

    result = nie::HasAttr<HasY, nie::GetAttrTypeZ>::value;
    ASSERT_EQ(result, false);

    result = nie::HasAttr<HasZ, nie::GetAttrTypeZ>::value;
    ASSERT_EQ(result, true);
}

TEST(HasAttr, XYZ) {
    bool result;
    result = nie::HasAttrXyz<HasX>::value;
    ASSERT_EQ(result, false);

    result = nie::HasAttrXyz<HasY>::value;
    ASSERT_EQ(result, false);

    result = nie::HasAttrXyz<HasZ>::value;
    ASSERT_EQ(result, false);

    result = nie::HasAttrXyz<HasXyz>::value;
    ASSERT_EQ(result, true);
}

class EuclideanDistanceF : public ::testing::Test {
protected:
    constexpr static double kPrecision = 1e-12;
    nie::EuclideanDistance euclidean_distance_metric{};
    struct XYZ {
        double x, y, z;
    };

    struct ReferenceData {
        XYZ const test1;
        XYZ const test2;
        double const distance;
    };

    //clang-format off
    std::vector<ReferenceData> const kTestVector{
        {{0, 0, 0}, {1, 0, 0}, 1},
        {{0, 0, 0}, {0, 1, 0}, 1},
        {{0, 0, 0}, {0, 0, 1}, 1},
        {{0, 0, 0}, {3, 4, 0}, 5},
    };
    //clang-format on
};

TEST_F(EuclideanDistanceF, OperatorXyz) {
    for (auto const& ref_data : kTestVector) {
        double const dist = euclidean_distance_metric(ref_data.test1, ref_data.test2);
        ASSERT_NEAR(dist, ref_data.distance, kPrecision);
    }
}
