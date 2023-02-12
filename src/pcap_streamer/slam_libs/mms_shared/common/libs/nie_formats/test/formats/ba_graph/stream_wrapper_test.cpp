/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <cstdio>

#include <gtest/gtest.h>
#include <nie/core/filesystem.hpp>
#include <nie/formats/ba_graph/ba_graph_types.hpp>
#include <nie/formats/ba_graph/stream_wrapper.hpp>

class StreamWrapperTest : public testing::Test {
private:
    // Helper functions and variables
    static Eigen::Matrix<double, 6, 6> CreateCovariance() {
        Eigen::Matrix<double, 6, 6> m;
        for (std::size_t row = 0; row < 6; ++row) {
            for (std::size_t col = row; col < 6; ++col) {
                std::size_t value = row * 6 + col;
                m(row, col) = value;
                m(col, row) = value;
            }
        }
        return m;
    }

    Eigen::Vector3d const vector3d_{1., 2., 3.};
    Eigen::Matrix3d const matrix3d_{(Eigen::Matrix3d() << .36, .48, -.8, -.8, .6, 0., .48, .64, .6).finished()};
    Eigen::Quaterniond quaterniond_{matrix3d_};

protected:
    std::string const kWritePath = "stream_test_write.bin";
    std::string const kValidatedPath = "/data/aiim/unit_tests_data/formats/stream_validated.bin";

    // control variables
    double const control_double = 1.0;
    enum class EnumType { A, B, C } control_enum = EnumType::B;
    std::string const control_string = "hello";
    std::array<int, 3> const control_array{1, 2, 3};
    cv::Point_<float> const control_point2f{1.0f, 2.0f};
    cv::Point3_<double> const control_point3d{1.0, 1.0, 1.0};
    Eigen::Matrix2f const control_matrix2f{matrix3d_.topLeftCorner<2, 2>().cast<float>()};
    Eigen::Matrix<double, 6, 6> const control_covariance{CreateCovariance()};
    nie::Isometry3qf const control_isometry3qf{vector3d_.cast<float>(), quaterniond_.cast<float>()};

    void TearDown() override { std::remove(kWritePath.c_str()); }
};

TEST_F(StreamWrapperTest, Construction) {
    ASSERT_NO_THROW(nie::io::IStreamWrapper isw{new std::ifstream()});
    ASSERT_NO_THROW(nie::io::OStreamWrapper osw{new std::ofstream()});
}

TEST_F(StreamWrapperTest, WriteValue) {
    auto os = std::ofstream(kWritePath, std::ios_base::out | std::ios_base::binary);
    ASSERT_TRUE(os.good());
    nie::io::OStreamWrapper osw{&os};
    ASSERT_NO_THROW(osw.WriteValue(control_double));
    ASSERT_NO_THROW(osw.WriteValue(control_enum));
    ASSERT_NO_THROW(osw.WriteValue(control_string));
    ASSERT_NO_THROW(osw.WriteValue(control_array));
    ASSERT_NO_THROW(osw.WriteValue(control_point2f));
    ASSERT_NO_THROW(osw.WriteValue(control_point3d));
    ASSERT_NO_THROW(osw.WriteValue(control_matrix2f));
    ASSERT_NO_THROW(osw.WriteValue(nie::io::MakeSymmetricMatrixRef(control_covariance)));
    ASSERT_NO_THROW(osw.WriteValue(control_isometry3qf));
    os.close();

    ASSERT_TRUE(nie::BinaryEquals(kValidatedPath, kWritePath));
}

TEST_F(StreamWrapperTest, ReadValue) {
    // experimental variables
    double experimental_double{};
    EnumType experimental_enum{};
    std::string experimental_string{};
    std::array<int, 3> experimental_array{};
    cv::Point_<float> experimental_point2f{};
    cv::Point3_<double> experimental_point3d{};
    Eigen::Matrix2f experimental_matrix2f{};
    Eigen::Matrix<double, 6, 6> experimental_covariance{};
    nie::Isometry3qf experimental_isometry3qf{};

    auto is = std::ifstream(kValidatedPath, std::ios_base::in | std::ios_base::binary);
    ASSERT_TRUE(is.good());
    nie::io::IStreamWrapper isw{&is};
    auto cov_ref = nie::io::MakeSymmetricMatrixRef(experimental_covariance);
    ASSERT_NO_THROW(isw.ReadValue(&experimental_double));
    ASSERT_NO_THROW(isw.ReadValue(&experimental_enum));
    ASSERT_NO_THROW(isw.ReadValue(&experimental_string));
    ASSERT_NO_THROW(isw.ReadValue(&experimental_array));
    ASSERT_NO_THROW(isw.ReadValue(&experimental_point2f));
    ASSERT_NO_THROW(isw.ReadValue(&experimental_point3d));
    ASSERT_NO_THROW(isw.ReadValue(&experimental_matrix2f));
    ASSERT_NO_THROW(isw.ReadValue(&cov_ref));
    ASSERT_NO_THROW(isw.ReadValue(&experimental_isometry3qf));
    is.close();

    EXPECT_DOUBLE_EQ(control_double, experimental_double);
    EXPECT_EQ(control_enum, experimental_enum);
    EXPECT_EQ(control_string, experimental_string);
    EXPECT_EQ(control_array, experimental_array);
    EXPECT_EQ(control_point2f, experimental_point2f);
    EXPECT_EQ(control_point3d, experimental_point3d);
    EXPECT_EQ(control_matrix2f, experimental_matrix2f);
    EXPECT_EQ(control_covariance, cov_ref.get());
    EXPECT_EQ(control_isometry3qf.translation(), experimental_isometry3qf.translation());
    EXPECT_EQ(control_isometry3qf.rotation().coeffs(), experimental_isometry3qf.rotation().coeffs());
}
