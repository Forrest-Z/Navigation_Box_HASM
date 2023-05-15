/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

namespace nie {

struct TimeNode {
    // unique name provided by the user
    std::string unique_id;
    // implementation specific
    std::chrono::time_point<std::chrono::steady_clock> tic;
    // accumulated time for this node
    std::chrono::nanoseconds sum;
    // number of times this node was instantiated
    std::uint32_t hits;  // average = sum / hits
    // identifiers of the children nodes
    std::vector<std::size_t> children;
};

class ScopedTimer {
public:
    explicit ScopedTimer(std::string const& unique_id);
    ~ScopedTimer();

    static void SetEnabled(bool isEnabled = true);

    // Statistics for each thread
    static std::vector<std::vector<TimeNode>> GetStatistics();
    static std::string FormattedStatistics();

    ScopedTimer() = delete;
    ScopedTimer(const ScopedTimer&) = delete;
    ScopedTimer(const ScopedTimer&&) = delete;
    ScopedTimer& operator=(const ScopedTimer&) = delete;

private:
    std::size_t unique_id_;
};

}  // namespace nie
