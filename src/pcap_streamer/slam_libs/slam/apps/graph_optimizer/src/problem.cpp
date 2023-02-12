/* Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "problem.hpp"

#include <unordered_map>
#include <utility>
#include <vector>

#include <nie/core/geometry/covariance.hpp>
#include <nie/cv/ceres/back_projection_cost.hpp>
#include <nie/cv/ceres/edge_se3_cost.hpp>
#include <nie/cv/ceres/identity_cost_function.hpp>
#include <nie/cv/ceres/identity_se3_cost.hpp>
#include <nie/cv/ceres/weighted_cost_function.hpp>

void AddParameterBlocksPoses(
        ceres::Problem* problem,
        ceres::LocalParameterization* quaternion_local_parameterization,
        nie::io::PoseCollection* p_pose_estimates,
        std::unordered_map<nie::io::PoseId, std::size_t>* p_pose_index_by_pose_id) {
    std::unordered_map<nie::io::PoseId, std::size_t>& pose_index_by_pose_id = *p_pose_index_by_pose_id;

    for (std::size_t index = 0; index < p_pose_estimates->poses.size(); ++index) {
        auto& p = p_pose_estimates->poses[index];
        auto& t = p.isometry.translation();
        auto& r = p.isometry.rotation();
        problem->AddParameterBlock(t.data(), static_cast<int>(t.size()));
        problem->AddParameterBlock(
                r.coeffs().data(), static_cast<int>(r.coeffs().size()), quaternion_local_parameterization);

        pose_index_by_pose_id[p.id] = index;
    }
}

void AddParameterBlocksObjts(
        ceres::Problem* problem,
        nie::io::ObjectCollection* p_objt_estimates,
        std::unordered_map<std::int32_t, std::size_t>* p_objt_index_by_objt_id) {
    std::unordered_map<std::int32_t, std::size_t>& objt_index_by_objt_id = *p_objt_index_by_objt_id;

    for (std::size_t index = 0; index < p_objt_estimates->objects.size(); ++index) {
        auto& o = p_objt_estimates->objects[index];
        auto& x = o.position;
        problem->AddParameterBlock(x.data(), static_cast<int>(x.size()));

        objt_index_by_objt_id[o.id] = index;
    }
}

void AddResidualBlocksKpnt(
        nie::io::KeypointCollection const& kpnt_constraints,
        nie::io::RectifiedCameraParameters const& parameters,
        std::unordered_map<nie::io::PoseId, std::size_t> const& pose_index_by_pose_id,
        std::unordered_map<std::int32_t, std::size_t> const& objt_index_by_objt_id,
        ceres::Problem* problem,
        nie::io::PoseCollection* p_pose_estimates,
        nie::io::ObjectCollection* p_objt_estimates) {
    ceres::LossFunction* loss_function = nullptr;

    std::vector<double> const intrinsics = nie::CreateIntrinsicsVector(parameters.K);

    // TODO(jbr): Not using the covariance yet.
    for (auto const& k : kpnt_constraints.keypoints) {
        auto const pi = pose_index_by_pose_id.find(k.pose_id);
        CHECK(pi != pose_index_by_pose_id.end()) << "Keypoint refers to non-existent pose estimate: " << k.pose_id;
        auto const oi = objt_index_by_objt_id.find(k.object_id);
        CHECK(oi != objt_index_by_objt_id.end()) << "Keypoint refers to non-existent object estimate: " << k.object_id;
        auto& p = p_pose_estimates->poses[pi->second];
        auto& t = p.isometry.translation();
        auto& r = p.isometry.rotation();
        auto& x = p_objt_estimates->objects[oi->second];
        CHECK(k.frame_id < parameters.frames.size()) << "Keypoint refers to non-existent frame: " << k.frame_id;
        auto const& b = parameters.frames[k.frame_id].baseline;

        ceres::CostFunction* cost_function = nullptr;
        if (b == nie::Isometry3qd::Identity()) {
            cost_function = nie::BackprojectionWithExtrinsicsCost::Create(intrinsics, k.position);
        } else {
            cost_function = nie::BackprojectionWithBaselineCost::Create(intrinsics, k.position, b);
        }
        problem->AddResidualBlock(cost_function, loss_function, t.data(), r.coeffs().data(), x.position.data());
    }
}

void AddResidualBlockPose(
        nie::io::PoseRecord const& pc,
        std::unordered_map<nie::io::PoseId, std::size_t> const& pose_index_by_pose_id,
        Eigen::Matrix<double, 6, 6> weights,
        ceres::Problem* problem,
        nie::io::PoseCollection* p_pose_estimates) {
    auto const pi = pose_index_by_pose_id.find(pc.id);
    CHECK(pi != pose_index_by_pose_id.end()) << "Pose constraint refers to non-existent pose estimate: " << pc.id;
    auto& pe = p_pose_estimates->poses[pi->second];
    auto& t = pe.isometry.translation();
    auto& r = pe.isometry.rotation();

    ceres::LossFunction* loss_function = nullptr;
    ceres::CostFunction* cost_function = nie::IdentitySe3Cost::Create(pc.isometry, std::move(weights));
    problem->AddResidualBlock(cost_function, loss_function, t.data(), r.coeffs().data());
}

void AddResidualBlocksPose(
        nie::io::PoseCollection const& pose_constraints,
        std::unordered_map<nie::io::PoseId, std::size_t> const& pose_index_by_pose_id,
        ceres::Problem* problem,
        nie::io::PoseCollection* p_pose_estimates) {
    // If an information matrix equals zero, no residual block is added. More robust, less memory and saves processing
    // time.
    if (pose_constraints.header.HasPoseInformation()) {
        if (!pose_constraints.header.pose_information.isApproxToConstant(0.0)) {
            Eigen::Matrix<double, 6, 6> weights;
            nie::InformationToWeights(pose_constraints.header.pose_information, &weights);
            for (auto& pc : pose_constraints.poses) {
                AddResidualBlockPose(pc, pose_index_by_pose_id, weights, problem, p_pose_estimates);
            }
        }
        // Meaning there is pose information per record.
    } else {
        for (auto& pc : pose_constraints.poses) {
            if (!pc.information.isApproxToConstant(0.0)) {
                Eigen::Matrix<double, 6, 6> weights;
                nie::InformationToWeights(pc.information, &weights);
                AddResidualBlockPose(pc, pose_index_by_pose_id, std::move(weights), problem, p_pose_estimates);
            }
        }
    }
}

void AddResidualBlocksEdge(
        nie::io::PoseCollection const& pose_constraints,
        std::unordered_map<nie::io::PoseId, std::size_t> const& pose_index_by_pose_id,
        ceres::Problem* problem,
        nie::io::PoseCollection* p_pose_estimates) {
    ceres::LossFunction* loss_function = nullptr;

    for (auto& ec : pose_constraints.edges) {
        auto const eib = pose_index_by_pose_id.find(ec.id_begin);
        CHECK(eib != pose_index_by_pose_id.end())
                << "Pose constraint refers to non-existent pose estimate: " << ec.id_begin;
        auto& peb = p_pose_estimates->poses[eib->second];
        auto& tb = peb.isometry.translation();
        auto& rb = peb.isometry.rotation();

        auto const eie = pose_index_by_pose_id.find(ec.id_end);
        CHECK(eie != pose_index_by_pose_id.end())
                << "Pose constraint refers to non-existent pose estimate: " << ec.id_end;
        auto& pee = p_pose_estimates->poses[eie->second];
        auto& te = pee.isometry.translation();
        auto& re = pee.isometry.rotation();

        Eigen::Matrix<double, 6, 6> weights;
        if (pose_constraints.header.HasEdgeInformationPerRecord()) {
            nie::InformationToWeights(ec.information, &weights);
        } else {
            nie::InformationToWeights(pose_constraints.header.edge_information, &weights);
        }

        ceres::CostFunction* cost_function = nie::EdgeSe3Cost::Create(ec.isometry, std::move(weights));

        problem->AddResidualBlock(
                cost_function, loss_function, tb.data(), rb.coeffs().data(), te.data(), re.coeffs().data());
    }
}

void AddResidualBlocksObjt(
        nie::io::ObjectCollection const& objt_constraints,
        std::unordered_map<std::int32_t, std::size_t> const& objt_index_by_objt_id,
        ceres::Problem* problem,
        nie::io::ObjectCollection* p_objt_estimates) {
    ceres::LossFunction* loss_function = nullptr;

    for (auto& oc : objt_constraints.objects) {
        auto const oi = objt_index_by_objt_id.find(oc.id);
        CHECK(oi != objt_index_by_objt_id.end())
                << "Object constraint refers to non-existent object estimate: " << oc.id;
        auto& oe = p_objt_estimates->objects[oi->second];
        auto& op = oe.position;

        Eigen::Matrix3d weights;
        if (objt_constraints.header.HasInformationPerRecord()) {
            nie::InformationToWeights(oc.information, &weights);
        } else {
            nie::InformationToWeights(objt_constraints.header.information, &weights);
        }

        ceres::CostFunction* cost_function = nie::MatWeightedCostFunction::Create<nie::IdentityCostFunction<3>>(
                std::vector<double>(weights.data(), weights.data() + 9), std::vector<double>(op.data(), op.data() + 3));

        problem->AddResidualBlock(cost_function, loss_function, op.data());
    }
}

void SetParameterBlocksConstantPose(
        nie::io::PoseCollection const& pose_constraints,
        std::unordered_map<nie::io::PoseId, std::size_t> const& pose_index_by_pose_id,
        ceres::Problem* problem,
        nie::io::PoseCollection* p_pose_estimates) {
    for (auto& f : pose_constraints.fixes) {
        auto const pi = pose_index_by_pose_id.find(f.id);
        CHECK(pi != pose_index_by_pose_id.end()) << "Pose fix refers to non-existent pose estimate: " << f.id;
        auto& p = p_pose_estimates->poses[pi->second].isometry;
        problem->SetParameterBlockConstant(p.translation().data());
        problem->SetParameterBlockConstant(p.rotation().coeffs().data());
    }
}

void BuildOptimizationProblem(
        nie::io::RectifiedCameraParameters const& parameters,
        nie::io::PoseCollection const& pose_constraints,
        nie::io::KeypointCollection const& kpnt_constraints,
        nie::io::ObjectCollection const& objt_constraints,
        ceres::Problem* problem,
        nie::io::PoseCollection* p_pose_estimates,
        nie::io::ObjectCollection* p_objt_estimates) {
    CHECK(pose_constraints.poses.size() == p_pose_estimates->poses.size());

    std::unordered_map<nie::io::PoseId, std::size_t> pose_index_by_pose_id;
    std::unordered_map<std::int32_t, std::size_t> objt_index_by_objt_id;

    ceres::LocalParameterization* quaternion_local_parameterization = new ceres::EigenQuaternionParameterization;

    AddParameterBlocksPoses(problem, quaternion_local_parameterization, p_pose_estimates, &pose_index_by_pose_id);
    AddParameterBlocksObjts(problem, p_objt_estimates, &objt_index_by_objt_id);

    AddResidualBlocksKpnt(
            kpnt_constraints,
            parameters,
            pose_index_by_pose_id,
            objt_index_by_objt_id,
            problem,
            p_pose_estimates,
            p_objt_estimates);

    if (pose_constraints.header.HasAnyPoseInformation()) {
        AddResidualBlocksPose(pose_constraints, pose_index_by_pose_id, problem, p_pose_estimates);
    }
    if (pose_constraints.header.HasAnyEdgeInformation()) {
        AddResidualBlocksEdge(pose_constraints, pose_index_by_pose_id, problem, p_pose_estimates);
    }
    if (objt_constraints.header.HasAnyInformation()) {
        AddResidualBlocksObjt(objt_constraints, objt_index_by_objt_id, problem, p_objt_estimates);
    }

    SetParameterBlocksConstantPose(pose_constraints, pose_index_by_pose_id, problem, p_pose_estimates);
}

// Returns true if the solve was successful.
ceres::Solver::Summary SolveOptimizationProblem(ceres::Problem* problem, std::uint32_t const& num_threads) {
    CHECK(problem != nullptr);

    ceres::Solver::Options options;
    options.max_num_iterations = 200;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.num_threads = num_threads;

    ceres::Solver::Summary summary;
    ceres::Solve(options, problem, &summary);

    std::cout << summary.FullReport() << std::endl;

    return summary;
}
