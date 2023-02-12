#include <gtest/gtest.h>

#include <tool/add_edges/add_edges.hpp>

TEST(PoseToolTest, AddEdges) {
    nie::io::PoseCollection pose_collection{};

    nie::io::PoseRecord pr0{};
    pr0.id = 0;
    pr0.isometry = nie::Isometry3qd::FromTranslation(Eigen::Vector3d::Zero());

    pose_collection.poses.push_back(pr0);

    AddEdges(&pose_collection);

    EXPECT_EQ(pose_collection.edges.size(), 0);

    nie::io::PoseRecord pr1{};
    pr1.id = 1;
    pr1.isometry = nie::Isometry3qd::FromTranslation(Eigen::Vector3d(1.0, 0.0, 0.0));

    nie::io::PoseRecord pr2{};
    pr2.id = 2;
    pr2.isometry = nie::Isometry3qd::FromTranslation(Eigen::Vector3d(2.0, 0.0, 0.0));

    // This one is spatially too far to be connected with an edge.
    nie::io::PoseRecord pr3{};
    pr3.id = 3;
    pr3.isometry = nie::Isometry3qd::FromTranslation(Eigen::Vector3d(10.0, 0.0, 0.0));

    pose_collection.poses.push_back(pr1);
    pose_collection.poses.push_back(pr2);
    pose_collection.poses.push_back(pr3);

    AddEdges(&pose_collection);

    // Assert because the checks below it would otherwise die.
    ASSERT_EQ(pose_collection.edges.size(), 2);
    EXPECT_EQ(pose_collection.edges[0].id_begin, 0);
    EXPECT_EQ(pose_collection.edges[0].id_end, 1);
    EXPECT_EQ(pose_collection.edges[1].id_begin, 1);
    EXPECT_EQ(pose_collection.edges[1].id_end, 2);
}
