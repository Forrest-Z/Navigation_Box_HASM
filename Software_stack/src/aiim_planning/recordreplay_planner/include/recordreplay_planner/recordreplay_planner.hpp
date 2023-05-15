// Copyright 2020 Embotech AG, Zurich, Switzerland, inspired by Christopher Ho's mpc code
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// The changes made in this file, of which a summary is listed below, are copyrighted:
//
// Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
// Information classification: Confidential
// This content is protected by international copyright laws.
// Reproduction and distribution is prohibited without written permission.
//
// List of changes:
// * Using aiim namespace
// * Adding aiim prefix
#ifndef RECORDREPLAY_PLANNER__RECORDREPLAY_PLANNER_HPP_
#define RECORDREPLAY_PLANNER__RECORDREPLAY_PLANNER_HPP_

#include <aiim_autoware_common/types.hpp>
#include <aiim_autoware_msgs/msg/trajectory.hpp>
#include <aiim_autoware_msgs/msg/vehicle_kinematic_state.hpp>
#include <aiim_motion_common/config.hpp>
#include <recordreplay_planner/visibility_control.hpp>

#include <deque>
#include <fstream>
#include <map>
#include <string>

using aiim::common::types::bool8_t;
using aiim::common::types::float64_t;

namespace aiim {
namespace planning {
namespace recordreplay_planner {
using State = aiim_autoware_msgs::msg::VehicleKinematicState;
using aiim_autoware_msgs::msg::Trajectory;
using Heading = decltype(decltype(State::state)::heading);

enum class RecordReplayState { IDLE, RECORDING, REPLAYING };  // enum class RecordReplayState

/// \brief A class for recording trajectories and replaying them as plans
class RECORDREPLAY_PLANNER_PUBLIC RecordReplayPlanner {
public:
    RecordReplayPlanner();

    // Record and replay control
    bool8_t is_recording() const noexcept;
    bool8_t is_replaying() const noexcept;
    void start_recording(const std::string& file_path = "");
    void stop_recording();
    void start_replaying() noexcept;
    void stop_replaying() noexcept;

    // Clear the internal recording buffer
    void clear_record() noexcept;

    /// \brief Add a new state to the record buffer
    /// \param[in] state_to_record A state to attempt to add to the recording buffer
    /// \return True if state was added to record buffer, False otherwise
    bool record_state(const State& state_to_record);

    // Replay trajectory from stored plan. The current state of the vehicle is given
    // and the trajectory will be chosen from the stored plan such that the starting
    // point of the trajectory is as close as possible to this current state.
    const Trajectory& plan(const State& current_state);

    // Return the number of currently-recorded State messages
    std::size_t get_record_length() const noexcept;

    // Heading weight configuration
    void set_heading_weight(float64_t heading_weight);
    float64_t get_heading_weight();

    void set_min_record_distance(float64_t min_record_distance);
    float64_t get_min_record_distance() const;

    // Loading buffered trajectory information to/from disk
    void readTrajectoryBufferFromFile(const std::string& replay_path);

    /**
     * \brief Judges whether current_state has reached the last point in record buffer
     * \param current_state current state of the vehicle
     * \param distance_thresh threshold of euclidean distance between the current statethe and the last point in meters
     * \param angle_thresh threshold of difference in the headings of the current_state and the last point in radians
     * \return true if both distance and angle conditions are satisfied
     */
    bool8_t reached_goal(
            const State& current_state, const float64_t& distance_thresh, const float64_t& angle_thresh) const;

private:
    // Obtain a trajectory from the internally-stored recording buffer
    RECORDREPLAY_PLANNER_LOCAL const Trajectory& from_record(const State& current_state);
    RECORDREPLAY_PLANNER_LOCAL std::size_t get_closest_state(const State& current_state);
    RECORDREPLAY_PLANNER_LOCAL void open_record_file(const std::string&);
    RECORDREPLAY_PLANNER_LOCAL void write_record_file_header(std::ofstream&);
    RECORDREPLAY_PLANNER_LOCAL void write_state(std::ofstream&, const State& state);

    // Weight of heading in computations of differences between states
    float64_t m_heading_weight = 0.1;
    float64_t m_min_record_distance = 0.0;
    float64_t m_min_record_distance_squared = 0.0;

    std::size_t m_traj_start_idx{};
    std::size_t m_traj_end_idx{};
    std::deque<State> m_record_buffer;
    Trajectory m_trajectory{};
    RecordReplayState m_recordreplaystate{RecordReplayState::IDLE};
    std::ofstream m_record_file_stream;
};  // class RecordReplayPlanner
}  // namespace recordreplay_planner
}  // namespace planning
}  // namespace aiim
#endif  // RECORDREPLAY_PLANNER__RECORDREPLAY_PLANNER_HPP_
