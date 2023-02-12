/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#include "scoped_timer.hpp"

#include <glog/logging.h>

#include <algorithm>
#include <chrono>
#include <map>
#include <sstream>
#include <stack>
#include <thread>

namespace nie {

namespace detail {

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

class Timer {
public:
    Timer() : data_() {}
    Timer(const Timer&) = delete;
    Timer(const Timer&&) = delete;
    Timer& operator=(const Timer&) = delete;

    static Timer& GetInstance() {
        CHECK(instances_.size() <= 1) << "Multi threading timing is not implemented yet";
        // TODO(RaC) Replace map with a lock free data structure to allow multi thread timing
        return instances_[std::this_thread::get_id()];
    }

    static std::vector<std::vector<TimeNode>> GetStatistics() {
        std::vector<std::vector<TimeNode>> ret;
        ret.reserve(instances_.size());

        // For each thread
        for (auto const& thread_id_and_time_node : instances_) {
            ret.emplace_back(thread_id_and_time_node.second.data_);
        }
        return ret;
    }

    std::size_t Tic(std::string const& unique_id) {
        if (!is_enabled_) {
            return 0;
        }
        std::size_t id = GetNode(unique_id);

        if (!current_stack_.empty()) {
            DCHECK(NotInStack(id)) << "Recursive functions are not handled";

            auto it = std::find(
                std::begin(data_[current_stack_.top()].children), std::end(data_[current_stack_.top()].children), id);

            if (it == std::end(data_[current_stack_.top()].children)) {
                data_[current_stack_.top()].children.emplace_back(id);
            }
        }
        current_stack_.push(id);

        data_[id].hits++;
        data_[id].tic = Clock::now();
        return id;
    }

    void Tac(std::size_t const& id) {
        if (!is_enabled_) {
            return;
        }
        auto tac = Clock::now();

        std::chrono::nanoseconds elapsed = tac - data_[id].tic;
        data_[id].sum += elapsed;
        data_[id].tic = TimePoint();  // reset start
        current_stack_.pop();
    }

    static void SetIsEnabled(bool isEnabled) { is_enabled_ = isEnabled; }

private:
    std::size_t GetNode(std::string const& unique_id) {
        // If the element is already indexed
        auto it = cache_.find(unique_id);
        if (it != cache_.end()) {
            return it->second;
        }

        // Create new node as children of the last open node
        // Or as sibling of last open node
        data_.push_back({unique_id, TimePoint(), std::chrono::nanoseconds(0), 0, {}});

        auto id = static_cast<std::size_t>(data_.size() - 1);

        cache_.insert({unique_id, id});
        return id;
    }

    bool NotInStack(std::size_t id) {
        std::vector<std::size_t> my_stack;
        while (!current_stack_.empty()) {
            my_stack.push_back(current_stack_.top());
            current_stack_.pop();
        }
        auto it = std::find(my_stack.begin(), my_stack.end(), id);
        auto i = my_stack.size();
        while (i-- > 0) {
            current_stack_.push(my_stack[i]);
        }
        return it == my_stack.end();
    }

    std::vector<TimeNode> data_;
    std::map<std::string, std::size_t> cache_;
    std::stack<std::size_t> current_stack_;

    static bool is_enabled_;
    static std::map<std::thread::id, Timer> instances_;
};

std::map<std::thread::id, Timer> Timer::instances_{};
bool Timer::is_enabled_ = false;

void PrintFormatedTimings(
    std::ostream& out, std::vector<TimeNode> nodes, std::size_t id, std::size_t level, std::size_t* highest_index) {
    TimeNode& node = nodes[id];
    std::string const tabs(level * 2, ' ');

    if (id > *highest_index) {
        *highest_index = id;
    }
    out << tabs << node.unique_id
        << " mean: " << (std::chrono::duration_cast<std::chrono::milliseconds>(node.sum) / node.hits).count()
        << "ms for " << node.hits << " hits" << std::endl;

    for (auto child_id : nodes[id].children) {
        PrintFormatedTimings(out, nodes, child_id, level + 1, highest_index);
    }
}

}  // namespace detail

ScopedTimer::ScopedTimer(std::string const& unique_id) : unique_id_(detail::Timer::GetInstance().Tic(unique_id)) {}

ScopedTimer::~ScopedTimer() { detail::Timer::GetInstance().Tac(unique_id_); }

void ScopedTimer::SetEnabled(bool isEnabled) { detail::Timer::SetIsEnabled(isEnabled); }

std::vector<std::vector<TimeNode>> ScopedTimer::GetStatistics() { return detail::Timer::GetStatistics(); }

std::string ScopedTimer::FormattedStatistics() {
    std::vector<std::vector<TimeNode>> stats = detail::Timer::GetStatistics();
    std::stringstream ss;

    std::string const separator(40, '=');
    for (std::size_t i = 0; i < stats.size(); ++i) {
        ss << std::endl
           << separator << std::endl
           << "Beginning of thread " << i << std::endl;

        std::size_t highest = 0;
        do {
            detail::PrintFormatedTimings(ss, stats[i], highest, 0, &highest);
            highest++;
        } while (highest < stats[i].size());

        ss << "End of thread " << i << std::endl
           << separator << std::endl;
    }

    return ss.str();
}

}  // namespace nie
