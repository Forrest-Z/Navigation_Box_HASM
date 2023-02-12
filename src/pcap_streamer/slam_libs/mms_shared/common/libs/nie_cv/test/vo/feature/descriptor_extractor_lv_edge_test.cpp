/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <memory>

#include <gtest/gtest.h>

#include <nie/cv/vo/feature/descriptor_extractor_lv_edge.hpp>

/*
 * The simple (size x size) matrix created by the functions CreateMatrix and
 * ImageValue to be tested will look like:
 *   0+0   1+0   2+0   ...
 *   0+ n  1+ n  2+ n  ...
 *   0+2n  1+2n  2+2n  ...
 *   ...   ...   ...   ...
 * Because of this filling, the Sobel response can be calculated to be:
 *   horizontal: -1 0 1
 *               -2 0 2  ->  +8
 *               -1 0 1
 *   vertical:   -1 -2 -1
 *                0  0  0  ->  +8n
 *                1  2  1
 */

namespace detail {

using ImageType = uint8_t;

ImageType ImageValue(ImageType r, ImageType c, ImageType n) { return c + r * n; }

cv::Mat CreateMatrix(ImageType size) {
    cv::Mat_<ImageType> result(size, size);
    for (ImageType r = 0; r < size; ++r) {
        for (ImageType c = 0; c < size; ++c) {
            result(r, c) = ImageValue(r, c, size);
        }
    }
    return cv::Mat(result);
}

struct TestDescription {
    std::string name;
    cv::Mat image;
    nie::KeypointVector features;
    nie::DescriptorVector expected_result;
};

void RunTestCase(nie::DescriptorExtractorPtr const& extractor, TestDescription const& test) {
    nie::DescriptorVector const descriptors = extractor->Describe(test.image, test.features);

    ASSERT_EQ(test.features.size(), descriptors.size())
        << test.name << ": Feature vector and descriptor vector are not of same length.";

    for (int i = 0; i < descriptors.front().rows; ++i) {
        ASSERT_EQ(descriptors.front()[i], test.expected_result.front()[i])
            << test.name << ": Value at position i = " << i << " incorrect.";
    }
}

std::vector<TestDescription> GenerateTestCases() {
    std::vector<TestDescription> tests;
    {
        std::size_t size = 11;
        tests.emplace_back();
        tests.back().name = "Simple test";
        tests.back().image = CreateMatrix(size);
        tests.back().features.emplace_back(size / 2, size / 2);
        tests.back().expected_result.push_back({8, 8, 8, 8, 0,  0,  0,  0,  8,  8,  8,  8,  8,  8,  8,  8,
                                                0, 0, 0, 0, 88, 88, 88, 88, 88, 88, 88, 88, 88, 88, 88, 88});
    }
    {
        tests.emplace_back();
        tests.back().name = "Out-of-bound test";
        tests.back().image = CreateMatrix(1);
        tests.back().features.emplace_back(0, 0);
        tests.back().expected_result.push_back(
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
    }
    {
        tests.emplace_back();
        tests.back().name = "Real test";
        tests.back().image =
            imread("/data/aiim/unit_tests_data/cv/vo/20190214_140759163_09475.jpg", cv::IMREAD_REDUCED_GRAYSCALE_4);
        tests.back().features.emplace_back(361, 234);  // row, col
        tests.back().expected_result.push_back({-40, 43, -146, 101, -36, 76,  31,  -32, -2,  -46, -48,
                                                60,  43, 1,    37,  -49, -16, -29, -34, -49, -60, -50,
                                                15,  34, -6,   -60, -26, 42,  11,  -39, -67, -41});
    }
    return tests;
}

}  // namespace detail

TEST(VoFeatureDescriptorExtractorLvEdgeTest, Tests) {
    for (detail::TestDescription const& test : detail::GenerateTestCases()) {
        nie::DescriptorExtractorPtr const extractor = std::make_unique<nie::DescriptorExtractorLvEdge>();
        RunTestCase(extractor, test);
    }
}
