// Copyright 2020 The Autoware Foundation
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

/// \copyright Copyright 2020 The Autoware Foundation
/// \file
/// \brief This file defines the trajectory_smoother class.

#ifndef TRAJECTORY_SMOOTHER__TRAJECTORY_SMOOTHER_HPP_
#define TRAJECTORY_SMOOTHER__TRAJECTORY_SMOOTHER_HPP_

#include <aiim_autoware_common/types.hpp>
#include <cmath>
#include <vector>
#include "aiim_autoware_msgs/msg/trajectory.hpp"

#include "trajectory_smoother/visibility_control.hpp"

namespace aiim {
namespace planning {
namespace trajectory_smoother {

using aiim::common::types::float32_t;
using aiim_autoware_msgs::msg::Trajectory;

typedef struct {
    float32_t standard_deviation;  // standard deviation of the gaussian kernel
    uint32_t kernel_size;          // length of the gaussian kernel
} TrajectorySmootherConfig;

/// \brief Smooth over the trajectory by passing it through a gaussian filter
class TRAJECTORY_SMOOTHER_PUBLIC TrajectorySmoother {
public:
    /// \brief Initialise the gaussian kernel in the constructor
    /// \param[in] config Configuration containing parameters for the kernel
    explicit TrajectorySmoother(TrajectorySmootherConfig config);

    /// \brief Make the trajectory velocity smooth by passing it through a gaussian filter.
    /// \param[inout] trajectory The trajectory to be smoothed. This is modified in place.
    void Filter(Trajectory& trajectory);

private:
    std::vector<float32_t> m_kernel{};
};

}  // namespace trajectory_smoother
}  // namespace planning
}  // namespace aiim

#endif  // TRAJECTORY_SMOOTHER__TRAJECTORY_SMOOTHER_HPP_
