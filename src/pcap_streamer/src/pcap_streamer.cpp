/**
 * Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission.
 */
#include "pcap_streamer_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PcapStreamerNode>());
    rclcpp::shutdown();
    return 0;
}