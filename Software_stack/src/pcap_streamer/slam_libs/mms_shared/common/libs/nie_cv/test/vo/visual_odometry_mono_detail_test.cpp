/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <memory>
#include <numeric>

#include <gtest/gtest.h>
#include <nie/cv/vo/visual_odometry_mono.hpp>

namespace detail {

class VisualOdometryStateTest {
public:
    VisualOdometryStateTest(std::size_t keyframe_window_size, std::size_t ba_window_size, std::size_t ba_window_overlap)
        : params(CreateParameters(keyframe_window_size, ba_window_size, ba_window_overlap)),
          state(params),
          iterations(CreateIterations(params)) {}

    nie::detail::VisualOdometryMonoParameters const params;
    nie::detail::VisualOdometryState state;

    std::list<std::size_t> const iterations;

private:
    static nie::detail::VisualOdometryMonoParameters CreateParameters(
        std::size_t const& keyframe_window_size,
        std::size_t const& ba_window_size,
        std::size_t const& ba_window_overlap) {
        nie::detail::VisualOdometryMonoParameters params(Eigen::Matrix3d::Identity());
        params.keyframe_window_size = keyframe_window_size;
        params.ba_window_size = ba_window_size;
        params.ba_window_overlap = ba_window_overlap;
        return params;
    }
    static std::list<std::size_t> CreateIterations(nie::detail::VisualOdometryMonoParameters const& params) {
        std::size_t const N = params.keyframe_window_size * params.ba_window_size * 3;
        std::list<std::size_t> result(N);
        std::iota(result.begin(), result.end(), 0);
        return result;
    }
};

}  // namespace detail

//
// Example expected state results (using keyframe_window_size = 4, ba_window_size = 3, ba_window_overlap = 1):
//      0: first image, first motion, key frame,         ,
//      1:            , first motion,          ,         ,
//      2:            , first motion,          ,         ,
//      3:            , first motion, key frame,         ,
//      4:            ,             ,          ,         ,
//      5:            ,             ,          ,         ,
//      6:            ,             , key frame, ba frame,
//      7:            ,             ,          ,         ,
//      8:            ,             ,          ,         ,
//      9:            ,             , key frame,         ,
//     10:            ,             ,          ,         ,
//     11:            ,             ,          ,         ,
//     12:            ,             , key frame, ba frame,
//     13:            ,             ,          ,         ,
//     14:            ,             ,          ,         ,
//     15:            ,             , key frame,         ,
//     16:            ,             ,          ,         ,
//     17:            ,             ,          ,         ,
//     18:            ,             , key frame, ba frame,
//     19:            ,             ,          ,         ,
//

class VisualOdometryMonoStateTest : public ::testing::Test {
protected:
    static void TestIsFirstImage(detail::VisualOdometryStateTest& test) {
        for (std::size_t const& iter : test.iterations) {
            if (iter == 0) {
                EXPECT_TRUE(test.state.IsFirstImage());
            } else {
                EXPECT_FALSE(test.state.IsFirstImage());
            }
            test.state.Next();
        }
    }

    static void TestIsKeyFrame(detail::VisualOdometryStateTest& test) {
        for (std::size_t const& iter : test.iterations) {
            if (iter % (test.params.keyframe_window_size - 1) == 0) {
                EXPECT_TRUE(test.state.IsKeyFrame());
            } else {
                EXPECT_FALSE(test.state.IsKeyFrame());
            }
            test.state.Next();
        }
    }

    static void TestIsFirstMotion(detail::VisualOdometryStateTest& test) {
        for (std::size_t const& iter : test.iterations) {
            if (iter <= (test.params.keyframe_window_size - 1)) {
                EXPECT_TRUE(test.state.IsFirstMotion());
            } else {
                EXPECT_FALSE(test.state.IsFirstMotion());
            }
            test.state.Next();
        }
    }

    static void TestIsBaFrame(detail::VisualOdometryStateTest& test) {
        std::size_t const init_ba = (test.params.keyframe_window_size - 1) * (test.params.ba_window_size - 1);
        std::size_t const diff_ba =
            (test.params.keyframe_window_size - 1) * (test.params.ba_window_size - test.params.ba_window_overlap);
        for (std::size_t const& iter : test.iterations) {
            if (iter < init_ba ? iter % init_ba == 0 && !test.state.IsFirstImage() : (iter - init_ba) % diff_ba == 0) {
                EXPECT_TRUE(test.state.IsBaFrame());
            } else {
                EXPECT_FALSE(test.state.IsBaFrame());
            }
            test.state.Next();
        }
    }

    std::vector<detail::VisualOdometryStateTest> tests_ = GetTests();

private:
    static std::vector<detail::VisualOdometryStateTest> GetTests() {
        std::vector<detail::VisualOdometryStateTest> result = {{2, 2, 1}, {4, 2, 1}, {2, 3, 2}, {4, 3, 1}, {7, 6, 4}};
        return result;
    }
};

TEST_F(VisualOdometryMonoStateTest, TestIsFirstImage) {
    for (detail::VisualOdometryStateTest& t : tests_) {
        TestIsFirstImage(t);
    }
}

TEST_F(VisualOdometryMonoStateTest, TestIsKeyFrame) {
    for (detail::VisualOdometryStateTest& test : tests_) {
        TestIsKeyFrame(test);
    }
}

TEST_F(VisualOdometryMonoStateTest, TestIsFirstMotion) {
    for (detail::VisualOdometryStateTest& test : tests_) {
        TestIsFirstMotion(test);
    }
}

TEST_F(VisualOdometryMonoStateTest, TestIsBaFrame) {
    for (detail::VisualOdometryStateTest& t : tests_) {
        TestIsBaFrame(t);
    }
}
