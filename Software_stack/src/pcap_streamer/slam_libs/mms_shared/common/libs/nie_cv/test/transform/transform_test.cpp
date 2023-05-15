/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <atomic>

#include <gtest/gtest.h>
#include <nie/cv/transform/transform.hpp>

class TransformTest : public testing::Test {
protected:
    static constexpr int kPixelOffset = 1;
    cv::Size const kLutSizeRectangular{64, 32};

    static bool TransformerIdentity(cv::Point2f const& p_u, cv::Point2f* p_d) {
        *p_d = p_u;
        return true;
    }

    static bool TransformerOffset(cv::Point2f const& p_u, cv::Point2f* p_d) {
        *p_d = {p_u.x + kPixelOffset, p_u.y + kPixelOffset};
        return true;
    }

    static bool TransformerSingularity(cv::Point2f const&, cv::Point2f* p_d) {
        *p_d = {1.0f, 1.0f};
        return true;
    }

    static bool TransformerFalse(cv::Point2f const&, cv::Point2f*) { return false; }
};

TEST_F(TransformTest, GetLookUpTable) {
    // generate several luts and check if they are what we expect
    cv::Mat identity_lut;
    ASSERT_NO_THROW(identity_lut = nie::GetLookUpTable(TransformTest::TransformerIdentity, kLutSizeRectangular));
    std::atomic<bool> identity_lut_ok{true};
    identity_lut.forEach<cv::Point2f>([&](cv::Point2f& pixel, const int position[]) -> void {
        if (pixel.y != position[0] || pixel.x != position[1]) {
            identity_lut_ok = false;
        }
    });
    EXPECT_TRUE(identity_lut_ok);

    cv::Mat offset_lut;
    ASSERT_NO_THROW(offset_lut = nie::GetLookUpTable(TransformTest::TransformerOffset, kLutSizeRectangular));
    std::atomic<bool> offset_lut_ok{true};
    offset_lut.forEach<cv::Point2f>([&](cv::Point2f& pixel, const int position[]) -> void {
        if (pixel.y - kPixelOffset != position[0] || pixel.x - kPixelOffset != position[1]) {
            offset_lut_ok = false;
        }
    });
    EXPECT_TRUE(offset_lut_ok);

    cv::Mat singularity_lut;
    ASSERT_NO_THROW(singularity_lut = nie::GetLookUpTable(TransformTest::TransformerSingularity, kLutSizeRectangular));
    std::atomic<bool> singularity_lut_ok{true};
    singularity_lut.forEach<cv::Point2f>([&](cv::Point2f& pixel, const int [2]) -> void {
        if (pixel.y != 1 || pixel.x != 1) {
            singularity_lut_ok = false;
        }
    });
    EXPECT_TRUE(singularity_lut_ok);

    cv::Mat false_lut;
    ASSERT_NO_THROW(false_lut = nie::GetLookUpTable(TransformTest::TransformerFalse, kLutSizeRectangular));
    std::atomic<bool> false_lut_ok{true};
    false_lut.forEach<cv::Point2f>([&](cv::Point2f& pixel, const int [2]) -> void {
        if (pixel.y != -2 || pixel.x != -2) {
            false_lut_ok = false;
        }
    });
    EXPECT_TRUE(false_lut_ok);
}
