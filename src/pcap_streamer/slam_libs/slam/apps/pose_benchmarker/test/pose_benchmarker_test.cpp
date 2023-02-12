#include <gtest/gtest.h>

#include <chrono>

#include <nie/core/constants.hpp>
#include <nie/core/geometry/rotation.hpp>  // for nie::EulerToQuaternion

#include <benchmark_poses.hpp>
#include <correct_euler_singularity.hpp>

TEST(PoseBenchmarkerTest, ValidatePoseCollections) {
    nie::io::PoseCollection pc_base{};
    nie::io::PoseCollection pc_other_auth{};
    nie::io::PoseCollection pc_other_time{};
    nie::io::PoseCollection pc_no_time{};

    uint expected_time_diff = 2;

    // Reference positions
    nie::io::PoseRecord pr1{};
    pr1.timestamp = nie::Timestamp_ns(std::chrono::nanoseconds(1));
    nie::io::PoseRecord pr2{};
    pr2.timestamp = nie::Timestamp_ns(std::chrono::nanoseconds(2));
    nie::io::PoseRecord pr3{};
    pr3.timestamp = nie::Timestamp_ns(std::chrono::nanoseconds(2 + expected_time_diff));
    nie::io::PoseRecord pr4{};
    pr4.timestamp = nie::Timestamp_ns(std::chrono::nanoseconds(3 + expected_time_diff));

    pc_base.poses.push_back(pr1);
    pc_base.poses.push_back(pr2);
    pc_other_auth.poses = pc_base.poses;
    pc_other_time.poses.push_back(pr3);
    pc_other_time.poses.push_back(pr4);
    pc_base.header.authority = "ESPG";
    pc_other_auth.header.authority = "NOT_ESPG";
    pc_other_time.header.authority = "ESPG";
    pc_no_time = pc_base;
    pc_base.header.Set(nie::io::PoseHeader::Flag::kHasTimestampPerRecord);
    pc_other_auth.header.Set(nie::io::PoseHeader::Flag::kHasTimestampPerRecord);
    pc_other_time.header.Set(nie::io::PoseHeader::Flag::kHasTimestampPerRecord);

    // Identical collections should be compatible
    nie::ValidatePoseCollectionCompatibility(pc_base, pc_base);

    // Different authorities should result in death
    EXPECT_DEATH(
            nie::ValidatePoseCollectionCompatibility(pc_base, pc_other_auth),
            "PoseCollections not compatible. Pose files do not have the same coordinate system.");

    // No timestamp information should result in death
    EXPECT_DEATH(
            nie::ValidatePoseCollectionCompatibility(pc_base, pc_no_time),
            "Benchmarking pose collection does not have timestamps per record.");
    EXPECT_DEATH(
            nie::ValidatePoseCollectionCompatibility(pc_no_time, pc_base),
            "Reference pose collection does not have timestamps per record.");

    // Different time ranges should result in death
    EXPECT_DEATH(
            nie::ValidatePoseCollectionCompatibility(pc_base, pc_other_time),
            ".* PoseCollections not compatible. Reference time range starts " + std::to_string(expected_time_diff) +
                    " nsec after the end of the benchmarking time range.");
    EXPECT_DEATH(
            nie::ValidatePoseCollectionCompatibility(pc_other_time, pc_base),
            ".* PoseCollections not compatible. Benchmarking time range starts " + std::to_string(expected_time_diff) +
                    " nsec after the end of the reference time range.");
}

TEST(PoseBenchmarkerTest, FindPoseDeltas) {
    constexpr static double kPrecision = 1e-12;
    nie::io::PoseCollection pose_collection_reference{};
    nie::io::PoseCollection pose_collection_benchmark{};

    // Reference positions
    nie::io::PoseRecord pref0{};
    pref0.id = 0;
    pref0.timestamp = nie::Timestamp_ns(std::chrono::nanoseconds(1));
    pref0.isometry = nie::Isometry3qd::FromTranslation(Eigen::Vector3d::Zero());

    nie::io::PoseRecord pref1{};
    pref1.id = 1;
    pref1.timestamp = nie::Timestamp_ns(std::chrono::nanoseconds(3));
    pref1.isometry = nie::Isometry3qd::FromTranslation(Eigen::Vector3d(1.0, 0.0, 0.0));

    // Same timestamp as pref0, but 0.1 x offset in position.
    nie::io::PoseRecord pbench0{};
    pbench0.id = 2;
    pbench0.timestamp = nie::Timestamp_ns(std::chrono::nanoseconds(1));
    pbench0.isometry = nie::Isometry3qd::FromTranslation(Eigen::Vector3d(0.1, 0.0, 0.0));
    pbench0.isometry.rotation() = nie::EulerToQuaternion(0.0, 0.0, nie::kPi<> / 2.0);

    // Timestamp and postion exactly in between pref0 and pref1.
    nie::io::PoseRecord pbench01{};
    pbench01.id = 3;
    pbench01.timestamp = nie::Timestamp_ns(std::chrono::nanoseconds(2));
    pbench01.isometry = nie::Isometry3qd::FromTranslation(Eigen::Vector3d(0.5, 0.0, 0.0));

    pose_collection_reference.poses.push_back(pref0);
    pose_collection_reference.poses.push_back(pref1);
    pose_collection_benchmark.poses.push_back(pbench0);
    pose_collection_benchmark.poses.push_back(pbench01);

    auto poses_matched = nie::MatchReferenceCollection(pose_collection_reference, pose_collection_benchmark);
    auto pose_deltas = nie::FindPoseDeltas(poses_matched);

    EXPECT_NEAR(pose_deltas[0].translation_absolute.x(), 0.1, kPrecision);
    EXPECT_NEAR(pose_deltas[0].translation_relative.x(), 0.0, kPrecision);
    EXPECT_NEAR(pose_deltas[0].translation_relative.y(), 0.1, kPrecision);
    EXPECT_NEAR(pose_deltas[1].translation_absolute.x(), 0.0, kPrecision);
}

TEST(PoseBenchmarkerTest, CorrectEulerSingularity) {
    auto const vec_0 = Eigen::Vector3d::Zero();                      // reference
    Eigen::Vector3d vec_a = Eigen::Vector3d::Constant(-nie::kPi<>);  // all negative
    Eigen::Vector3d vec_b = Eigen::Vector3d::Constant(nie::kPi<>);   // all positive
    Eigen::Vector3d vec_c{nie::kPi<>, -nie::kPi<>, -nie::kPi<>};     // mixed
    Eigen::Vector3d vec_d{0, nie::kPi<>, nie::kPi<>};                // two outside range -> no change

    nie::CorrectEulerSingularity(&vec_a);
    nie::CorrectEulerSingularity(&vec_b);
    nie::CorrectEulerSingularity(&vec_c);
    nie::CorrectEulerSingularity(&vec_d);

    EXPECT_EQ(vec_0, vec_a);
    EXPECT_EQ(vec_0, vec_b);
    EXPECT_EQ(vec_0, vec_c);
    EXPECT_EQ(vec_d, vec_d);
}