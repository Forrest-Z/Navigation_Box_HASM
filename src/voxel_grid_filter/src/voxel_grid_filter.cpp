/**
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * ------------------------------------------------------------------------------------
 * This is a modified version of the original voxel_grid_filter.cpp code from Autoware.
 * The changes made to this code, of which a summary is listed below, are copyrighted:
 * ------------------------------------------------------------------------------------
 * Copyright (C) 2020,2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission.
 * ------------------------------------------------------------------------------------
 *
 * List of changes:
 * * A lot of work has been done to refactor this file, with an focus on readability and maintainability.
 * * The code style has been adjusted in many places to conform to the AIIM Code Style.
 * * The input/tuning parameters are generalized and the defaults have been moved to the accompanying launch file.
 * * These parameters are now also all checked whether they are present, so the launch file is considered mandatory to
 * be used.
 * * The option to adjust the filter using a configuration message has been removed as we do not see an use for it in
 * our projects
 * * Implemented dynamic range filtering where the max range can be increased if the number of points is below a given
 * threshold.
 * * Changed function names to confirm to AIIM code style.
 * * Changed function arguments to use pointers to reduce copying and increase readability of the code.
 * * Made a new node so the old one got removed.
 */

#include "voxel_grid_filter_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VoxelGridFilterNode>());
    rclcpp::shutdown();
    return 0;
}
