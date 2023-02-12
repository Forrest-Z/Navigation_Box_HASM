// Copyright 2019 the Autoware Foundation
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
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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
// * Added aiim prefix
// * Added aiim namespace
// * Removing hardcoded topic names
#include <aiim_autoware_common/types.hpp>
#include <aiim_motion_common/motion_common.hpp>

#include <memory>
#include <string>

#include "pure_pursuit_nodes/pure_pursuit_node.hpp"

using aiim::common::types::bool8_t;
using aiim::common::types::float32_t;
using aiim::common::types::float64_t;

namespace aiim {
namespace control {
/// \brief Resources relating to the pure pursuit node package
namespace pure_pursuit_nodes {

PurePursuitNode::PurePursuitNode(const std::string &node_name,
                                 const std::string &node_namespace)
    : ControllerBaseNode{node_name, node_namespace} {
  pure_pursuit::Config cfg{
      static_cast<float32_t>(
          declare_parameter("controller.minimum_lookahead_distance")
              .get<float64_t>()),
      static_cast<float32_t>(
          declare_parameter("controller.maximum_lookahead_distance")
              .get<float64_t>()),
      static_cast<float32_t>(
          declare_parameter("controller.speed_to_lookahead_ratio")
              .get<float64_t>()),
      declare_parameter("controller.is_interpolate_lookahead_point")
          .get<bool8_t>(),
      declare_parameter("controller.is_delay_compensation").get<bool8_t>(),
      static_cast<float32_t>(
          declare_parameter("controller.emergency_stop_distance")
              .get<float64_t>()),
      static_cast<float32_t>(
          declare_parameter("controller.speed_thres_traveling_direction")
              .get<float64_t>()),
      static_cast<float32_t>(
          declare_parameter("controller.dist_front_rear_wheels")
              .get<float64_t>())};
  set_controller(std::make_unique<pure_pursuit::PurePursuit>(cfg));
}
////////////////////////////////////////////////////////////////////////////////
PurePursuitNode::PurePursuitNode(const std::string &node_name,
                                 const pure_pursuit::Config &cfg,
                                 const std::string &node_namespace)
    : ControllerBaseNode{node_name, node_namespace} {
  set_controller(std::make_unique<pure_pursuit::PurePursuit>(cfg));
}
} // namespace pure_pursuit_nodes
} // namespace control
} // namespace aiim
