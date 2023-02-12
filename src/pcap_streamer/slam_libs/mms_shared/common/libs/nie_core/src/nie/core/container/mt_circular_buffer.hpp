/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <condition_variable>
#include <mutex>
#include <vector>

#include "circular_buffer.hpp"

namespace nie {

namespace mt {

template <typename T, typename Alloc = std::allocator<T>>
class MtCircularBuffer {
public:
    MtCircularBuffer() = default;

    explicit MtCircularBuffer(std::size_t size);

    /// Attempts a push onto the circular buffer. Returns true on success.
    bool TryPushBack(T const& in);
    /// Attempts a push onto the circular buffer. Returns true on success.
    bool TryPushBack(T&& in);
    /// Attempts to pop from the circular buffer. Returns true on success.
    bool PopFront(T* out);
    /// Pushes to the back of the buffer. If the buffer is full, the calling thread goes into a wait state until another
    /// thread pops from the buffer.
    void BlockingPushBack(T const& in);
    /// Pushes to the back of the buffer. If the buffer is full, the calling thread goes into a wait state until another
    /// thread pops from the buffer.
    void BlockingPushBack(T&& in);
    /// Pops from the front of the buffer. If the buffer is empty, the calling thread goes into a wait state until
    /// another thread pushes to the buffer.
    void BlockingPopFront(T* out);
    /// Pushes to the back of the buffer. If, typename Alloc = std::allocator<T> the buffer is full, the calling thread
    /// goes into a wait state until another thread pops from the buffer or the request takes more than timeout_ms
    /// milliseconds to finish. Returns false when it times out and true otherwise.
    bool WaitForPushBack(T const& in, std::uint32_t timeout_ms);
    /// Pushes to the back of the buffer. If the buffer is full, the calling tread goes into a wait state until another
    /// thread pops from the buffer or the request takes more than timeout_ms milliseconds to finish. Returns false
    /// when it times out and true otherwise.
    bool WaitForPushBack(T&& in, std::uint32_t timeout_ms);
    /// Pops from the front of the buffer. If the buffer is empty, the calling thread goes into a wait state until
    /// another thread pushes to the buffer or the request takes more than timeout_ms milliseconds to finish. Returns
    /// false when it times out and true otherwise.
    bool WaitForPopFront(T* out, std::uint32_t timeout_ms);
    /// Pushes to the back of the buffer. If the buffer is full, the front gets overwritten.
    void PushBack(T const& in);
    /// Pushes to the back of the buffer. If the buffer is full, the front gets overwritten.
    void PushBack(T&& in);
    /// Clears the buffer.
    void Clear();

    /// Resize buffer
    void Resize(std::size_t size);

    /// Returns the newest item pushed in the buffer. Causes undefined behaviour if the buffer is empty.
    const T& Back() const;
    /// Returns the oldest item pushed in the buffer. Causes undefined behaviour if the buffer is empty.
    const T& Front() const;
    std::size_t Count() const;
    std::size_t Size() const;
    bool Empty() const;
    bool Full() const;

private:
    ::nie::CircularBuffer<T, Alloc> buffer_;
    // Otherwise we have to discard various const qualifiers, which we don't want (really!)
    mutable std::mutex mutex_;
    // Used by all blocking push functions. Pops notify.
    std::condition_variable cv_push_;
    // Used by all blocking pop functions. Pushes notify.
    std::condition_variable cv_pop_;

    bool TryPushBack_(const T& in);
    bool TryPushBack_(T&& in);
    bool PopFront_(T* out);
    void PushBack_(const T& in);
    void PushBack_(T&& in);
};

template <typename T, typename Alloc>
MtCircularBuffer<T, Alloc>::MtCircularBuffer(size_t const size) : buffer_{size}, cv_push_{}, cv_pop_{} {}

template <typename T, typename Alloc>
bool MtCircularBuffer<T, Alloc>::TryPushBack(T const& in) {
    bool pushed;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        pushed = TryPushBack_(in);
    }

    cv_pop_.notify_one();

    return pushed;
}

template <typename T, typename Alloc>
bool MtCircularBuffer<T, Alloc>::TryPushBack(T&& in) {
    bool pushed;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        pushed = TryPushBack_(std::move(in));
    }

    cv_pop_.notify_one();

    return pushed;
}

template <typename T, typename Alloc>
bool MtCircularBuffer<T, Alloc>::PopFront(T* out) {
    bool popped;

    {
        std::lock_guard<std::mutex> lock(mutex_);
        popped = PopFront_(out);
    }

    cv_push_.notify_one();

    return popped;
}

template <typename T, typename Alloc>
void MtCircularBuffer<T, Alloc>::BlockingPushBack(T const& in) {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_push_.wait(lock, [this, &in]() { return TryPushBack_(in); });
    }

    cv_pop_.notify_one();
}

template <typename T, typename Alloc>
void MtCircularBuffer<T, Alloc>::BlockingPushBack(T&& in) {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_push_.wait(lock, [this, &in]() { return TryPushBack_(std::move(in)); });
    }

    cv_pop_.notify_one();
}

template <typename T, typename Alloc>
void MtCircularBuffer<T, Alloc>::BlockingPopFront(T* out) {
    {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_pop_.wait(lock, [this, &out]() { return PopFront_(out); });
    }

    cv_push_.notify_one();
}

template <typename T, typename Alloc>
bool MtCircularBuffer<T, Alloc>::WaitForPushBack(T const& in, std::uint32_t timeout_ms) {
    bool success = false;
    {
        std::unique_lock<std::mutex> lock(mutex_);
        success = cv_push_.wait_for(
                lock, std::chrono::milliseconds(timeout_ms), [this, &in]() { return TryPushBack_(in); });
    }

    cv_pop_.notify_one();

    return success;
}

template <typename T, typename Alloc>
bool MtCircularBuffer<T, Alloc>::WaitForPushBack(T&& in, std::uint32_t timeout_ms) {
    bool success = false;
    {
        std::unique_lock<std::mutex> lock(mutex_);
        success = cv_push_.wait_for(
                lock, std::chrono::milliseconds(timeout_ms), [this, &in]() { return TryPushBack_(std::move(in)); });
    }

    cv_pop_.notify_one();

    return success;
}

template <typename T, typename Alloc>
bool MtCircularBuffer<T, Alloc>::WaitForPopFront(T* out, std::uint32_t timeout_ms) {
    bool success = false;
    {
        std::unique_lock<std::mutex> lock(mutex_);
        success = cv_pop_.wait_for(
                lock, std::chrono::milliseconds(timeout_ms), [this, &out]() { return PopFront_(out); });
    }

    cv_push_.notify_one();

    return success;
}

template <typename T, typename Alloc>
void MtCircularBuffer<T, Alloc>::PushBack(T const& in) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        PushBack_(in);
    }

    cv_pop_.notify_one();
}

template <typename T, typename Alloc>
void MtCircularBuffer<T, Alloc>::PushBack(T&& in) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        PushBack_(std::move(in));
    }

    cv_pop_.notify_one();
}

template <typename T, typename Alloc>
void MtCircularBuffer<T, Alloc>::Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.Clear();
}

template <typename T, typename Alloc>
void MtCircularBuffer<T, Alloc>::Resize(size_t const size) {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.Resize(size);
}

template <typename T, typename Alloc>
const T& MtCircularBuffer<T, Alloc>::Back() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.Back();
}

template <typename T, typename Alloc>
const T& MtCircularBuffer<T, Alloc>::Front() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.Front();
}

template <typename T, typename Alloc>
std::size_t MtCircularBuffer<T, Alloc>::Count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.Count();
}

template <typename T, typename Alloc>
std::size_t MtCircularBuffer<T, Alloc>::Size() const {
    // Size never changes so no mutex needed.
    return buffer_.Size();
}

template <typename T, typename Alloc>
bool MtCircularBuffer<T, Alloc>::Empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.Empty();
}

template <typename T, typename Alloc>
bool MtCircularBuffer<T, Alloc>::Full() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return buffer_.Full();
}

template <typename T, typename Alloc>
bool MtCircularBuffer<T, Alloc>::TryPushBack_(T const& in) {
    if (buffer_.Full()) {
        return false;
    }

    buffer_.PushBack(in);

    return true;
}

template <typename T, typename Alloc>
bool MtCircularBuffer<T, Alloc>::TryPushBack_(T&& in) {
    if (buffer_.Full()) {
        return false;
    }

    buffer_.PushBack(std::move(in));

    return true;
}

template <typename T, typename Alloc>
bool MtCircularBuffer<T, Alloc>::PopFront_(T* out) {
    if (buffer_.Empty()) {
        return false;
    }

    buffer_.PopFront(out);

    return true;
}

template <typename T, typename Alloc>
void MtCircularBuffer<T, Alloc>::PushBack_(T const& in) {
    buffer_.PushBack(in);
}

template <typename T, typename Alloc>
void MtCircularBuffer<T, Alloc>::PushBack_(T&& in) {
    buffer_.PushBack(std::move(in));
}

}  // end namespace mt

}  // end namespace nie
