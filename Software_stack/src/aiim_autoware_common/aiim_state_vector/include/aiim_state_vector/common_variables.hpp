// Copyright 2021 the Autoware Foundation
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
/// \copyright Copyright 2021 the Autoware Foundation
/// All rights reserved.
/// \file
/// \brief This file defines the common variables used within the filter implementations.
//
// The changes made in this file, of which a summary is listed below, are copyrighted:
//
// Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
// Information classification: Confidential
// This content is protected by international copyright laws.
// Reproduction and distribution is prohibited without written permission.
//
// List of changes:
// * Added aiim prefix
// * Added aiim namespace
#ifndef STATE_VECTOR__COMMON_VARIABLES_HPP_
#define STATE_VECTOR__COMMON_VARIABLES_HPP_

#include <aiim_state_vector/variable.hpp>

#include <type_traits>

namespace aiim {
namespace state_vector {

namespace variable {
struct X : Variable {};
struct Y : Variable {};
struct Z : Variable {};
struct ROLL : AngleVariable {};
struct PITCH : AngleVariable {};
struct YAW : AngleVariable {};

struct X_VELOCITY : Variable {};
struct Y_VELOCITY : Variable {};
struct Z_VELOCITY : Variable {};
struct ROLL_CHANGE_RATE : Variable {};
struct PITCH_CHANGE_RATE : Variable {};
struct YAW_CHANGE_RATE : Variable {};

struct X_ACCELERATION : Variable {};
struct Y_ACCELERATION : Variable {};
struct Z_ACCELERATION : Variable {};
struct ROLL_CHANGE_ACCELERATION : Variable {};
struct PITCH_CHANGE_ACCELERATION : Variable {};
struct YAW_CHANGE_ACCELERATION : Variable {};

/// Velocity in xy plane. Used in differential drive models.
struct XY_VELOCITY : Variable {};
/// Acceleration in xy plane. Used in differential drive models.
struct XY_ACCELERATION : Variable {};

}  // namespace variable

}  // namespace state_vector
}  // namespace aiim

#endif  // STATE_VECTOR__COMMON_VARIABLES_HPP_
