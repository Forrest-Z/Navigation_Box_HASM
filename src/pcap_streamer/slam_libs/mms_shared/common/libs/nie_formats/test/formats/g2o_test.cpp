/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include <cstdio>

#include <gtest/gtest.h>

#include <nie/core/filesystem.hpp>
#include <nie/formats/g2o_reader.hpp>
#include <nie/formats/g2o_writer.hpp>

class G2oTest : public testing::Test {
protected:
    std::string const kWritePath = "simple_test.g2o";
    std::string const kValidatedPath = "/data/aiim/unit_tests_data/formats/simple.g2o";

    nie::io::MapOfPoses const poses_ = CreateG2oPoses();
    nie::io::VectorOfConstraints const edges_ = CreateG2oEdges();

    static nie::io::MapOfPoses CreateG2oPoses() {
        nie::io::G2oVertexSe3Quat pose_1{1, {{0., 0., 0.}, Eigen::Quaterniond::Identity()}};
        nie::io::G2oVertexSe3Quat pose_2{2, {{1., 2., 3.}, Eigen::Quaterniond::Identity()}};
        return nie::io::MapOfPoses{{1, std::move(pose_1)}, {2, std::move(pose_2)}};
    }

    static nie::io::VectorOfConstraints CreateG2oEdges() {
        nie::io::G2oEdgeSe3Quat edge{
            1, 2, {{1.1, 2.2, 3.3}, Eigen::Quaterniond::Identity()}, Eigen::Matrix<double, 6, 6>::Identity()};
        edge.information(1, 4) = 3;
        edge.information(4, 1) = 3;
        return nie::io::VectorOfConstraints{std::move(edge)};
    }

    static void Test(nie::io::MapOfPoses const& lhs, nie::io::MapOfPoses const& rhs) {
        ASSERT_EQ(lhs.size(), rhs.size());
        for (auto const& key : lhs) {
            Test(lhs.at(key.first), rhs.at(key.first));
        }
    }

    static void Test(nie::io::VectorOfConstraints const& lhs, nie::io::VectorOfConstraints const& rhs) {
        ASSERT_EQ(lhs.size(), rhs.size());
        for (std::size_t i = 0; i < lhs.size(); ++i) {
            Test(lhs.at(i), rhs.at(i));
        }
    }

    void TearDown() override { std::remove(kWritePath.c_str()); }

private:
    static void Test(nie::io::G2oVertexSe3Quat const& lhs, nie::io::G2oVertexSe3Quat const& rhs) {
        ASSERT_EQ(lhs.id, rhs.id);
        ASSERT_EQ(lhs.v, rhs.v);
    }

    static void Test(nie::io::G2oEdgeSe3Quat const& lhs, nie::io::G2oEdgeSe3Quat const& rhs) {
        ASSERT_EQ(lhs.id_begin, rhs.id_begin);
        ASSERT_EQ(lhs.id_end, rhs.id_end);
        ASSERT_EQ(lhs.t_be, rhs.t_be);
        ASSERT_EQ(lhs.information, rhs.information);
    }
};

TEST_F(G2oTest, Write) {
    ASSERT_NO_THROW(nie::io::WriteG2oFile(kWritePath, poses_, edges_));
    ASSERT_TRUE(nie::BinaryEquals(kWritePath, kValidatedPath));
}

TEST_F(G2oTest, Read) {
    nie::io::MapOfPoses poses{};
    nie::io::VectorOfConstraints edges{};
    nie::io::ReadG2oFile(kValidatedPath, &poses, &edges);

    Test(poses, poses_);
    Test(edges, edges_);
}
