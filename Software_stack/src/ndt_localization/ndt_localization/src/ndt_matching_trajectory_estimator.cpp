/*
 * Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission.
 */

#include "ndt_matching_trajectory_estimator.hpp"

namespace nie {

namespace ndt {

void TrajectoryEstimator::Update(Pose const& ndt_pose, double dt) {
    // If buffer is full, remove oldest to make room the the newest pose.
    if (pose_history_.size() >= kHistorySize) {
        pose_history_.pop_front();
    }
    pose_history_.push_back(ndt_pose);

    // Do the same for Delta_t's.
    if (delta_t_history_.size() >= kHistorySize) {
        delta_t_history_.pop_front();
    }
    delta_t_history_.push_back(dt);
}

void TrajectoryEstimator::ReplaceLast(Pose const& ndt_pose, double dt) {
    pose_history_.back() = ndt_pose;
    delta_t_history_.back() = dt;
}

Pose TrajectoryEstimator::EstimateLatest() const { return EstimateWithLag(0); }

Pose TrajectoryEstimator::EstimateWithLag(unsigned int lag) const {

    // We need at least three for a good estimate that preserves acceleration.
    // If we don't have enough, let's just return the last pose we got.
    if (pose_history_.size() < 3 + lag) {
        return pose_history_.back();
    }

    // From here on, newest, oldest, etc. Are relative to the 3-pose
    // interval we always use for estimation.
    Pose const oldest_pose = *(pose_history_.crbegin() + lag + 2);
    Pose const middle_pose = *(pose_history_.crbegin() + lag + 1);
    Pose const newest_pose = *(pose_history_.crbegin() + lag);

    // Delta_t's wrt. the previous pose.
    double const middle_delta_t = *(delta_t_history_.crbegin() + lag + 1);
    double const newest_delta_t = *(delta_t_history_.crbegin() + lag);

    RateOfChange const middle_velocity = RateOfChange::FromPoseDifference(middle_pose, oldest_pose, middle_delta_t);
    RateOfChange const newest_velocity = RateOfChange::FromPoseDifference(newest_pose, middle_pose, newest_delta_t);

    RateOfChange const newest_acceleration =
            RateOfChange::FromDifference(newest_velocity, middle_velocity, newest_delta_t);

    Pose predicted_pose = PredictPoseQuadratic(newest_pose, newest_velocity, newest_acceleration, newest_delta_t);

    // If working with a particular lag setting, we don't want to return the
    // immediate prediction, but rather want to predict an extra [lag] times
    // until we arrive at the future pose.
    while (lag > 0) {
        predicted_pose = PredictPoseQuadratic(predicted_pose, newest_velocity, newest_acceleration, newest_delta_t);
        --lag;
    }
    return predicted_pose;
}

void TrajectoryEstimator::Reset() {
    pose_history_.clear();
    delta_t_history_.clear();
}

}  // namespace ndt

}  // namespace nie
