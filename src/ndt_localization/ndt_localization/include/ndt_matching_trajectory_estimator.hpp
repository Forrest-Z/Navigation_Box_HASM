/*
 * Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission.
 */

#pragma once

#include <deque>

#include "ndt_matching_pose_prediction.hpp"
#include "ndt_matching_types.hpp"

namespace nie {

namespace ndt {

class TrajectoryEstimator {
public:
    /// Update the state of the trajectory estimator. Receives a pose and the
    /// difference in time between this pose and whichever came before it.
    /// \param ndt_pose - Instantaneous vehicle pose.
    /// \param dt - Difference in time between this pose and the previous one.
    void Update(Pose const& ndt_pose, double dt);

    /// Update the latest set of state variables received with the arguments.
    /// Used to replace the latest pose with the results of a better estimate,
    // allowing for continued improvement of the estimates on subsequent calls.
    /// \param ndt_pose - Instantaneous vehicle pose.
    /// \param dt - Difference in time between this pose and the previous one.
    void ReplaceLast(Pose const& ndt_pose, double dt);

    /// \return Would-be vehicle pose given our accumulated history and no lag.
    Pose EstimateLatest() const;

    /// Estimating a pose relies on our accumulated state (history of poses and
    /// Delta_T's. Always taking three poses and estimating the would-be future
    /// pose. The 'lag' parameter allows the client to pick which three poses
    /// should be used for the prediction, which leads to slightly different
    /// estimated trajectories. For example, given that our history contains
    /// four poses A -> B -> C -> D, calling this method with lag = 0 would
    /// would estimate a new pose E based on on the {B, C, D} set of poses
    /// Conversely, calling this method with lag = 1, would estimate poses D'
    /// and E' (note that we still want to estimate the FUTURE pose) based on
    /// poses {A, B, C}.
    /// \param lag - Skip the [lag] newest poses when estimating the trajectory.
    /// \return Would-be vehicle pose given our currently accumulated history.
    Pose EstimateWithLag(unsigned int lag) const;

    /// Reset the pose and delta time histories of the estimator.
    void Reset();

private:
    std::deque<Pose> pose_history_;
    std::deque<double> delta_t_history_;
    size_t const kHistorySize = 9;
};

}  // namespace ndt

}  // namespace nie
