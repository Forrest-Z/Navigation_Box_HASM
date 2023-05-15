/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <algorithm>

#include <gtest/gtest.h>
#include <nie/cv/non_maximum_suppression.hpp>
#include <opencv2/opencv.hpp>

namespace {

// Helper types
using ImageType = int8_t;
constexpr ImageType kMaxValue = 8;
constexpr ImageType kMinValue = -8;

bool IsPointSmallerThan(cv::Point const& a, cv::Point const& b) {
    bool result;
    if (a.x != b.x) {
        result = a.x < b.x;
    } else {
        result = a.y < b.y;
    }
    return result;
}

/*
 * This function is created to filter the extreme values found, because the test
 * images contain zeroes by default. For the extreme locations having image zero
 * these locations will be filtered out. The sorting is required in order to
 * compare the points, regardless of the order in which the algorithm finds them.
 */
std::vector<cv::Point> FilterAndSortActualPoints(cv::Mat_<ImageType> const& image, std::vector<cv::Point2f>& points) {
    std::vector<cv::Point> result;

    // Filter all points having image value 0
    for (auto const& p : points) {
        if (image(p) != 0) {
            result.emplace_back(p.x, p.y);
        }
    }

    std::sort(result.begin(), result.end(), IsPointSmallerThan);

    return result;
}

struct Extreme {
    explicit Extreme(int x, int y, int v = kMaxValue, bool e = true) : point(x, y), value(v), is_extreme(e) {}

    bool operator<(Extreme const& other) { return IsPointSmallerThan(point, other.point); }

    cv::Point2i point;
    ImageType value;
    bool is_extreme;
};
using Extremes = std::vector<Extreme>;

struct Test {
    explicit Test(std::string new_name, int size, int new_distance)
        : name(std::move(new_name)), image(cv::Mat::zeros(size, size, CV_8SC1)), distance(new_distance) {}

    bool MaxOnly() const {
        return std::all_of(extremes.begin(), extremes.end(), [](Extreme const& e) { return e.value == kMaxValue; });
    }

    Extremes FilteredAndSortedExtremes() const {
        Extremes result;

        std::copy_if(extremes.begin(), extremes.end(), std::back_inserter(result), [](Extreme const& e) {
            return e.is_extreme;
        });

        std::sort(result.begin(), result.end());

        return result;
    }

    std::string name;
    cv::Mat_<ImageType> image;
    int distance;
    Extremes extremes;
};

std::vector<Test> GetTests() {
    /*
     * Create the test cases that will be used to test the algorithm.
     *
     * Note that:
     *  - the pixels up to a "distance" from the edge will not be found,
     *    but considered in the maximum finding.
     *  - the x and y values for a extreme value is like:
     *    (x,y) = (1,2)   ->   0 0 0 .
     *                         0 0 0 .
     *                         0 x 0 .
     *                         . . . .
     */
    std::vector<Test> tests;
    {
        tests.emplace_back(
            "1 maximum in 3x3 image",
            /*size*/ 3,
            /*distance*/ 1);
        tests.back().extremes.emplace_back(1, 1);
    }
    {
        tests.emplace_back(
            "2 maxima in 8x8 image",
            /*size*/ 8,
            /*distance*/ 2);
        Extremes& e = tests.back().extremes;
        e.emplace_back(5, 2);
        e.emplace_back(2, 4);
    }
    {
        tests.emplace_back(
            "3 maxima in 8x8 image with one too close",
            /*size*/ 8,
            /*distance*/ 2);
        Extremes& e = tests.back().extremes;
        e.emplace_back(7, 4, kMaxValue, false);
        e.emplace_back(5, 2);
        e.emplace_back(2, 4);
    }
    {
        tests.emplace_back(
            "3 minima in 8x8 image with one too close",
            /*size*/ 8,
            /*distance*/ 2);
        Extremes& e = tests.back().extremes;
        e.emplace_back(7, 4, kMinValue, false);
        e.emplace_back(5, 2, kMinValue);
        e.emplace_back(2, 4, kMinValue);
    }
    {
        tests.emplace_back(
            "2 extremes in 8x8 image",
            /*size*/ 8,
            /*distance*/ 2);
        Extremes& e = tests.back().extremes;
        e.emplace_back(3, 4, kMaxValue);
        e.emplace_back(5, 2, kMinValue);
    }
    {
        // In this test case, the two maxima are too close and so only the
        // "first" will be selected. The same holds for the two minima. The one
        // being first is the one with lowest y value.
        tests.emplace_back(
            "4 extremes in 8x8 image",
            /*size*/ 8,
            /*distance*/ 2);
        Extremes& e = tests.back().extremes;
        e.emplace_back(3, 4, kMaxValue);
        e.emplace_back(2, 5, kMaxValue, false);
        e.emplace_back(5, 2, kMinValue);
        e.emplace_back(5, 3, kMinValue, false);
    }

    // Post processing
    for (Test& t : tests) {
        // Fill the image with the extreme the values
        for (const Extreme& e : t.extremes) {
            t.image(e.point) = e.value;
        }
    }
    return tests;
}

}  // namespace

// Actual tests to be executed

TEST(FindLocalExtremesTest, AllTests) {
    for (::Test const& test : GetTests()) {
        Extremes expected = test.FilteredAndSortedExtremes();

        std::vector<cv::Point2f> actual_features;

        std::vector<bool> actual_feature_types;
        nie::FindLocalExtremes(test.image, test.distance, &actual_features, &actual_feature_types, test.MaxOnly());
        std::vector<cv::Point> actual = FilterAndSortActualPoints(test.image, actual_features);

        ASSERT_EQ(actual.size(), expected.size()) << test.name << ": number of extremes is not matching.";

        for (std::size_t i = 0; i < actual.size(); ++i) {
            ASSERT_EQ(actual[i], expected[i].point) << test.name << ": extreme is different.";
        }
    }
}
