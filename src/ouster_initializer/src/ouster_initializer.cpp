/**
 * Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission.
 */

#include <iostream>
#include <string>

#include "ouster_initializer_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::make_shared<OusterInitializerNode>();
    rclcpp::shutdown();
    return 0;
}
