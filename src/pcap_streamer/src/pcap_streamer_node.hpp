/**
 * Copyright (C) 2021 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission.
 */
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <aiim_roscpp/logging.hpp>

class PcapPublisher {
public:
    virtual ~PcapPublisher() = default;
};

class PcapStreamerNode : public rclcpp::Node {
public:
    PcapStreamerNode();

private:
    template <typename T>
    void GetParameter(std::string const& key, T* value) {
        declare_parameter(key);
        RCHECK(get_logger(), "Parameter " << key << " not set.", get_parameter(key, *value));
    }

    std::unique_ptr<PcapPublisher> publisher_;
};
