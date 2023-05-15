/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <gtest/gtest.h>
#include <random>
#include <string>

#include <nie/geo_convert/geo_convert.hpp>

using namespace nie;

class TransformationTest : public testing::Test {
public:
    static double random_latitude() { return random_uniform_real(-90.0, +90.0); }

    static double random_longitude() { return random_uniform_real(-180.0, +180.0); }

    static double random_uniform_real(const double lower_bound, const double upper_bound) {
        std::uniform_real_distribution<double> uniform(lower_bound, upper_bound);
        std::random_device rd;
        std::mt19937 gen(rd());
        return uniform(gen);
    }
};

TEST(TransformationTest, ToAndFromUTM) {
    std::size_t const number_of_samples = 5000;
    double const precision = 1e-8;

    for (std::size_t i = 0; i < number_of_samples; ++i) {
        double const latitude = TransformationTest::random_latitude();
        double const longitude = TransformationTest::random_longitude();

        double utm_x{};
        double utm_y{};
        double gamma{};
        double k{};
        double recovered_latitude{};
        double recovered_longitude{};
        double recovered_gamma{};
        double recovered_k{};
        std::string utm_zone{};

        geo_convert::Wgs84ToUtm(latitude, longitude, &utm_x, &utm_y, &utm_zone, &gamma, &k);

        geo_convert::UtmToWgs84(
                utm_x, utm_y, utm_zone, &recovered_latitude, &recovered_longitude, &recovered_gamma, &recovered_k);

        ASSERT_NEAR(latitude, recovered_latitude, precision);
        ASSERT_NEAR(longitude, recovered_longitude, precision);
        ASSERT_NEAR(gamma, recovered_gamma, precision);
        ASSERT_NEAR(k, recovered_k, precision);
    }
}

TEST(TransformationTest, FromWgs84ToUTM) {
    double const latitude = 51.454;
    double const longitude = 5.398199;
    std::string utm_zone;

    double easting = 0.0;
    double northing = 0.0;
    double gamma = 0.0;
    double k = 0.0;

    geo_convert::Wgs84ToUtm(latitude, longitude, &easting, &northing, &utm_zone, &gamma, &k);

    const double precision = 1e-3;  // 1mm of error is tolerated

    EXPECT_NEAR(666622.257, easting, precision);
    EXPECT_NEAR(5703041.38, northing, precision);
}

TEST(TransformationTest, FromUTMToWgs84) {
    double const easting = 533210.66;
    double const northing = 6462896.50;
    std::string const utm_zone = "8n";

    double latitude{};
    double longitude{};
    double gamma{};
    double k{};

    geo_convert::UtmToWgs84(easting, northing, utm_zone, &latitude, &longitude, &gamma, &k);

    double const precision = 1e-6;

    EXPECT_NEAR(58.305801, latitude, precision);
    EXPECT_NEAR(-134.433304, longitude, precision);
}
