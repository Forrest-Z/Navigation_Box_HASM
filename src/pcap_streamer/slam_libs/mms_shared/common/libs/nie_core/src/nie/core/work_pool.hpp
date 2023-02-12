/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include <nie/core/container/mt_circular_buffer.hpp>

namespace nie {

template <typename Work>
class WorkPool {
public:
    WorkPool(std::size_t thread_count, std::size_t queue_size, std::function<void(std::unique_ptr<Work> const&)> worker)
        : queue_(queue_size), worker_(std::move(worker)) {
        for (std::size_t i = 0; i < thread_count; ++i) {
            threads_.push_back(std::thread(&WorkPool::Run, this));
        }
    }

    ~WorkPool() {
        for (auto&& thread [[maybe_unused]] : threads_) {
            queue_.BlockingPushBack(nullptr);
        }

        for (auto&& thread : threads_) {
            // All threads should always be joinable due to bounded blocking...
            // But perhaps someone enqueued a null ptr.
            if (thread.joinable()) thread.join();
        }
    }

    mt::MtCircularBuffer<std::unique_ptr<Work>>& queue() { return queue_; };

private:
    void Run() {
        while (true) {
            std::unique_ptr<Work> work;
            queue_.BlockingPopFront(&work);

            if (work == nullptr) return;

            worker_(work);
        }
    }

    std::vector<std::thread> threads_;
    mt::MtCircularBuffer<std::unique_ptr<Work>> queue_;
    std::function<void(std::unique_ptr<Work> const&)> worker_;
};

}  // namespace nie
