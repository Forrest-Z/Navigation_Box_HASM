/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include <string>

#include <gtest/gtest.h>

#include <solve.hpp>

class GraphOptimizerTest : public testing::Test {
    static constexpr double kPrecision = 1e-7;

    static bool ObjectSortComparator(nie::io::ObjectRecord const& l, nie::io::ObjectRecord const& r) {
        return l.id < r.id;
    }

    static bool PoseSortComparator(nie::io::PoseRecord const& l, nie::io::PoseRecord const& r) { return l.id < r.id; }

    static bool EdgeSortComparator(nie::io::PoseEdgeRecord const& l, nie::io::PoseEdgeRecord const& r) {
        return l.id_begin == r.id_begin ? l.id_end < r.id_end : l.id_begin < r.id_begin;
    }

    static void CompareObjectRecord(
            nie::io::ObjectRecord const& l, nie::io::ObjectRecord const& r, bool has_record_information) {
        ASSERT_EQ(l.id, r.id);
        ASSERT_LT((l.position - r.position).norm(), kPrecision);

        if (has_record_information) {
            ASSERT_TRUE(l.information.isApprox(r.information, kPrecision));
        }
    }

    static void ComparePoseRecord(
            nie::io::PoseRecord const& l, nie::io::PoseRecord const& r, bool has_record_information) {
        ASSERT_EQ(l.id, r.id);
        ASSERT_EQ(l.timestamp, r.timestamp);
        if (!l.isometry.isApprox(r.isometry, kPrecision)) {
            LOG(INFO) << " ======  Translation  ===== " << std::endl;
            LOG(INFO) << l.isometry.translation().transpose() << " differs from ";
            LOG(INFO) << r.isometry.translation().transpose() << " diff is ";
            LOG(INFO) << (l.isometry.translation() - r.isometry.translation()).transpose() << " which is ";
            LOG(INFO) << (l.isometry.translation() - r.isometry.translation()).norm();

            LOG(INFO) << " ======  Rotation  ===== " << std::endl;
            LOG(INFO) << l.isometry.rotation().vec().transpose() << " differs from ";
            LOG(INFO) << r.isometry.rotation().vec().transpose() << " diff is ";
            LOG(INFO) << (l.isometry.rotation().vec() - r.isometry.rotation().vec()).transpose() << " which is ";
            LOG(INFO) << (l.isometry.rotation().vec() - r.isometry.rotation().vec()).norm();
            EXPECT_TRUE(l.isometry.isApprox(r.isometry, kPrecision));
        }
        if (has_record_information) {
            if (!l.information.isApprox(r.information, kPrecision) &&
                !(std::isnan(l.information.norm()) || std::isnan(r.information.norm()))) {
                LOG(INFO) << l.information.transpose() << " differs from ";
                LOG(INFO) << r.information.transpose() << " diff is ";
                LOG(INFO) << (l.information - r.information).transpose() << " which is ";
                LOG(INFO) << (l.information - r.information).norm();
                EXPECT_TRUE(l.information.isApprox(r.information, kPrecision));
            }
        }
    }

    static void CompareEdgeRecord(
            nie::io::PoseEdgeRecord const& l, nie::io::PoseEdgeRecord const& r, bool has_record_information) {
        ASSERT_EQ(l.id_begin, r.id_begin);
        ASSERT_EQ(l.id_end, r.id_end);
        ASSERT_EQ(l.category, r.category);
        if (!l.isometry.isApprox(r.isometry, kPrecision)) {
            LOG(INFO) << " ======  Translation  ===== " << std::endl;
            LOG(INFO) << l.isometry.translation().transpose() << " differs from ";
            LOG(INFO) << r.isometry.translation().transpose() << " diff is ";
            LOG(INFO) << (l.isometry.translation() - r.isometry.translation()).transpose() << " which is ";
            LOG(INFO) << (l.isometry.translation() - r.isometry.translation()).norm();

            LOG(INFO) << " ======  Rotation  ===== " << std::endl;
            LOG(INFO) << l.isometry.rotation().vec().transpose() << " differs from ";
            LOG(INFO) << r.isometry.rotation().vec().transpose() << " diff is ";
            LOG(INFO) << (l.isometry.rotation().vec() - r.isometry.rotation().vec()).transpose() << " which is ";
            LOG(INFO) << (l.isometry.rotation().vec() - r.isometry.rotation().vec()).norm();
            EXPECT_TRUE(l.isometry.isApprox(r.isometry, kPrecision));
        }
        if (has_record_information) {
            if (!l.information.isApprox(r.information, kPrecision) &&
                !(std::isnan(l.information.norm()) || std::isnan(r.information.norm()))) {
                LOG(INFO) << l.information.transpose() << " differs from ";
                LOG(INFO) << r.information.transpose() << " diff is ";
                LOG(INFO) << (l.information - r.information).transpose() << " which is ";
                LOG(INFO) << (l.information - r.information).norm();
                EXPECT_TRUE(l.information.isApprox(r.information, kPrecision));
            }
        }
    }

protected:
    std::string const kDataDir = "/data/aiim/unit_tests_data/graph_optimizer/";

    // Takes pointers as it will sort the records to enable comparing them
    static void ComparePoseCollection(nie::io::PoseCollection* l, nie::io::PoseCollection* r) {
        ASSERT_EQ(l->header.HasCodeZ(), r->header.HasCodeZ());
        ASSERT_EQ(l->header.HasPoseInformation(), r->header.HasPoseInformation());
        ASSERT_EQ(l->header.HasPoseInformationPerRecord(), r->header.HasPoseInformationPerRecord());
        ASSERT_EQ(l->header.HasEdgeInformation(), r->header.HasEdgeInformation());
        ASSERT_EQ(l->header.HasEdgeInformationPerRecord(), r->header.HasEdgeInformationPerRecord());

        ASSERT_EQ(l->header.version, r->header.version);
        ASSERT_EQ(l->header.authority, r->header.authority);
        ASSERT_EQ(l->header.code_xy_or_xyz, r->header.code_xy_or_xyz);
        if (l->header.HasCodeZ()) {
            ASSERT_EQ(l->header.code_z, r->header.code_z);
        }

        ASSERT_EQ(l->poses.size(), r->poses.size());
        std::sort(l->poses.begin(), l->poses.end(), PoseSortComparator);
        std::sort(r->poses.begin(), r->poses.end(), PoseSortComparator);
        for (std::size_t i = 0; i < l->poses.size(); ++i) {
            ComparePoseRecord(l->poses[i], r->poses[i], l->header.HasPoseInformationPerRecord());
        }

        ASSERT_EQ(l->edges.size(), r->edges.size());
        std::sort(l->edges.begin(), l->edges.end(), EdgeSortComparator);
        std::sort(r->edges.begin(), r->edges.end(), EdgeSortComparator);
        for (std::size_t i = 0; i < l->edges.size(); ++i) {
            CompareEdgeRecord(l->edges[i], r->edges[i], l->header.HasEdgeInformationPerRecord());
        }

        ASSERT_EQ(l->fixes.size(), r->fixes.size());
    }

    static void CompareObjtCollection(nie::io::ObjectCollection* l, nie::io::ObjectCollection* r) {
        ASSERT_EQ(l->header.HasInformation(), r->header.HasInformation());
        ASSERT_EQ(l->header.HasInformationPerRecord(), r->header.HasInformationPerRecord());
        ASSERT_EQ(l->header.flags, r->header.flags);

        ASSERT_EQ(l->objects.size(), r->objects.size());
        std::sort(l->objects.begin(), l->objects.end(), ObjectSortComparator);
        std::sort(r->objects.begin(), r->objects.end(), ObjectSortComparator);
        for (std::size_t i = 0; i < l->objects.size(); ++i) {
            CompareObjectRecord(l->objects[i], r->objects[i], l->header.HasInformationPerRecord());
        }
    }
};

TEST_F(GraphOptimizerTest, DISABLED_Calibration) {
    std::string const test_dir = kDataDir + "calibration/";

    nie::io::RectifiedCameraParameters camera_parameters =
            nie::io::RectifiedCameraParameters::Read(test_dir + "rectified_intrinsics_mono_vector.json");

    auto const kpnt_constraints = nie::io::ReadCollection<nie::io::KeypointCollection>(test_dir + "constraints.kpnt");
    auto const objt_constraints = nie::io::ReadCollection<nie::io::ObjectCollection>(test_dir + "constraints.objt");
    auto pose_estimates = nie::io::ReadCollection<nie::io::PoseCollection>(test_dir + "estimates.pose");
    auto objt_estimates = nie::io::ReadCollection<nie::io::ObjectCollection>(test_dir + "estimates.objt");
    auto pose_constraints = pose_estimates;

    bool const output_information_matrices = true;
    bool const double_pass = false;
    double const mahalanobis_threshold{};
    double inlier_percentage{};
    bool const output_mahalanobis_distances = false;
    Filenamer const mahalanobis_distances_filenamer{};

    Solve(camera_parameters,
          kpnt_constraints,
          objt_constraints,
          output_information_matrices,
          double_pass,
          mahalanobis_threshold,
          inlier_percentage,
          output_mahalanobis_distances,
          mahalanobis_distances_filenamer,
          &pose_constraints,
          &pose_estimates,
          &objt_estimates);

    auto pose_ref = nie::io::ReadCollection<nie::io::PoseCollection>(test_dir + "ba_ref.pose");
    ComparePoseCollection(&pose_estimates, &pose_ref);

    auto objt_ref = nie::io::ReadCollection<nie::io::ObjectCollection>(test_dir + "ba_ref.objt");
    CompareObjtCollection(&objt_estimates, &objt_ref);
}

TEST_F(GraphOptimizerTest, DISABLED_PoseGraphSingle) {
    std::string const test_dir = kDataDir + "pose_graph_single/";

    auto pose_estimates = nie::io::ReadCollection<nie::io::PoseCollection>(test_dir + "before.pose");
    auto pose_constraints = pose_estimates;

    bool const output_information_matrices = true;

    Solve(output_information_matrices, &pose_constraints, &pose_estimates);

    auto pose_ref = nie::io::ReadCollection<nie::io::PoseCollection>(test_dir + "ba_ref.pose");
    ComparePoseCollection(&pose_estimates, &pose_ref);
}

TEST_F(GraphOptimizerTest, DISABLED_PoseGraphDouble) {
    std::string const test_dir = kDataDir + "pose_graph_double/";

    auto pose_estimates = nie::io::ReadCollection<nie::io::PoseCollection>(test_dir + "before.pose");
    auto pose_constraints = pose_estimates;

    bool const output_information_matrices = true;
    bool const double_pass = true;
    double const mahalanobis_threshold = 0.05;

    Solve(output_information_matrices, double_pass, mahalanobis_threshold, &pose_constraints, &pose_estimates);

    // Estimates were used as constraints, but edge constraints are filtered, so copy those back to the estimates
    pose_estimates.edges = pose_constraints.edges;

    auto pose_ref = nie::io::ReadCollection<nie::io::PoseCollection>(test_dir + "ba_ref.pose");
    ComparePoseCollection(&pose_estimates, &pose_ref);
}

TEST_F(GraphOptimizerTest, DISABLED_PoseGraphDoubleFilter) {
    std::string const test_dir = kDataDir + "pose_graph_double_filter/";

    auto pose_estimates = nie::io::ReadCollection<nie::io::PoseCollection>(test_dir + "before.pose");
    auto pose_constraints = pose_estimates;

    bool const output_information_matrices = true;
    bool const double_pass = true;
    double const mahalanobis_threshold = 500;

    Solve(output_information_matrices, double_pass, mahalanobis_threshold, &pose_constraints, &pose_estimates);

    // Estimates were used as constraints, but edge constraints are filtered, so copy those back to the estimates
    pose_estimates.edges = pose_constraints.edges;

    auto pose_ref = nie::io::ReadCollection<nie::io::PoseCollection>(test_dir + "ba_ref.pose");
    ComparePoseCollection(&pose_estimates, &pose_ref);
}
