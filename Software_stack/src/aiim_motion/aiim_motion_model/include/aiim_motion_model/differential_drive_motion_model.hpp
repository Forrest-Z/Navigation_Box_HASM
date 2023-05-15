// Copyright 2021 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Developed by Apex.AI, Inc.
//
// The changes made in this file, of which a summary is listed below, are
// copyrighted:
//
// Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights
// reserved Information classification: Confidential This content is protected
// by international copyright laws. Reproduction and distribution is prohibited
// without written permission.
//
// List of changes:
// * Adding aiim namespace
// * Adding aiim prefix
#ifndef MOTION_MODEL__DIFFERENTIAL_DRIVE_MOTION_MODEL_HPP_
#define MOTION_MODEL__DIFFERENTIAL_DRIVE_MOTION_MODEL_HPP_

#include <aiim_motion_model/motion_model_interface.hpp>
#include <aiim_motion_model/visibility_control.hpp>
#include <aiim_state_vector/common_states.hpp>
#include <aiim_state_vector/generic_state.hpp>

namespace aiim {
namespace motion {
namespace motion_model {

/// @brief      A generic differential motion model. This class only exists to
/// be specialized for
///             specific motion model implementations.
template <typename StateT>
class MOTION_MODEL_PUBLIC DifferentialDriveMotionModel
    : public MotionModelInterface<DifferentialDriveMotionModel<StateT>> {
public:
  using State = StateT;

protected:
  // Allow the CRTP interface to call private functions.
  friend MotionModelInterface<DifferentialDriveMotionModel<StateT>>;

  /// @brief      A crtp-called function that predicts the state forward.
  State crtp_predict(const State &, const std::chrono::nanoseconds &) const {
    static_assert(sizeof(StateT) == 0,
                  "Function crtp_predict is expected to be specialized for "
                  "every state it is used with.");
  }

  /// @brief      A crtp-called function that computes a Jacobian.
  typename State::Matrix crtp_jacobian(const State &,
                                       const std::chrono::nanoseconds &) const {
    static_assert(sizeof(StateT) == 0,
                  "Function crtp_jacobian is expected to be specialized for "
                  "every state it is used with.");
  }
};

/// @brief      An alias of the differential drive motion model for the
///             aiim::state_vector::ConstantVelocityAndTurnRate state.
using CvtrMotionModel = DifferentialDriveMotionModel<
    aiim::state_vector::ConstantVelocityAndTurnRate>;
/// @brief      An alias of the differential drive motion model for the
///             aiim::state_vector::ConstantAccelerationAndTurnRate state.
using CatrMotionModel = DifferentialDriveMotionModel<
    aiim::state_vector::ConstantAccelerationAndTurnRate>;

/// @brief      A crtp-called function that predicts the state forward.
template <>
MOTION_MODEL_PUBLIC CvtrMotionModel::State
CvtrMotionModel::crtp_predict(const CvtrMotionModel::State &state,
                              const std::chrono::nanoseconds &dt) const;

/// @brief      A crtp-called function that computes a Jacobian.
template <>
MOTION_MODEL_PUBLIC CvtrMotionModel::State::Matrix
CvtrMotionModel::crtp_jacobian(const CvtrMotionModel::State &state,
                               const std::chrono::nanoseconds &dt) const;

/// @brief      A crtp-called function that predicts the state forward.
template <>
MOTION_MODEL_PUBLIC CatrMotionModel::State
CatrMotionModel::crtp_predict(const CatrMotionModel::State &state,
                              const std::chrono::nanoseconds &dt) const;

/// @brief      A crtp-called function that computes a Jacobian.
template <>
MOTION_MODEL_PUBLIC CatrMotionModel::State::Matrix
CatrMotionModel::crtp_jacobian(const CatrMotionModel::State &state,
                               const std::chrono::nanoseconds &dt) const;

} // namespace motion_model
} // namespace motion
} // namespace aiim

#endif // MOTION_MODEL__DIFFERENTIAL_DRIVE_MOTION_MODEL_HPP_
