/* Copyright (C) 2020, 2021 by NavInfo Europe B.V. The Netherlands - All rights
 * reserved Information classification: Confidential This content is protected
 * by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */

#include "path_cluster_detection_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PathClusterDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
