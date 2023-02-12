/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#pragma once

#include <functional>
#include <stdexcept>
#include <vector>

namespace nie {

namespace detail {

/// struct to perform circular arithmetic, i.e.: perform modulo after each operation.
/// A faster version of this struct would be to always use power of two for @param{size}.
/// We could check at construction/Resize of the CircularBuffer:
///     if its size is 2^n then use the faster indexing scheme
///     else use regular indexing
struct CircularArithmeticSimple {
    /// Increment @param{index} modulo @param{size}
    constexpr static size_t Increment(size_t index, size_t size) {
        ++index;
        if (index == size) {
            index = 0;
        }
        return index;
    }
    /// Decrement @param{index} modulo @param{size}
    constexpr static size_t Decrement(size_t index, size_t size) {
        if (index != 0) {
            --index;
        } else {
            index = size - 1;
        }
        return index;
    }
    /// Add @param{delta} to @param{index} modulo @param{size}
    constexpr static size_t Plus(size_t index, size_t delta, size_t size) { return (index + delta) % size; }
    /// Subtract @param{delta} to @param{index} modulo @param{size}
    constexpr static size_t Minus(size_t index, size_t delta, size_t size) {
        delta = delta % size;
        return (index + size - delta) % size;
    }
    constexpr static size_t Offset(size_t index, int offset, size_t size) {
        return offset < 0 ? Minus(index, -offset, size) : Plus(index, offset, size);
    }
};

/// struct that implements finite counting so that @param{lower} <= @param{index} <= @param{upper}.
struct SaturatingCounter {
    /// Increment @param{index} only if @param{index} < @param{upper}
    constexpr static size_t Increment(size_t index, size_t upper) {
        if (index < upper) {
            ++index;
        }
        return index;
    }
    /// Decrement @param{index} only if @param{index} > @param{lower}
    constexpr static size_t Decrement(size_t index, size_t lower) {
        if (index > lower) {
            --index;
        }
        return index;
    }
};

}  // namespace detail

/// Not thread-safe.
/// Push will overwrite the oldest element in the buffer with a new element.
/// Pop will remove and return the oldest element in the buffer.
///
/// \tparam T                   Date type in the buffer.
/// \tparam Alloc               Allocator passed directly to std::vector.
/// \tparam CircularArithmetic  This can be easily changed to a more efficient version later. The "Circularity" of the
///                             buffer is completely defined by the circular indexing scheme.
template <typename T, typename Alloc = std::allocator<T>, class CircularArithmetic = detail::CircularArithmeticSimple>
class CircularBuffer {
private:
    using value_type = typename Alloc::value_type;
    using difference_type = typename Alloc::difference_type;
    using reference = typename Alloc::reference;
    using const_reference = typename Alloc::const_reference;
    using pointer = typename Alloc::pointer;
    using const_pointer = typename Alloc::const_pointer;

    /// Custom iterator
    // As this concerns a circular buffer, the naive end would be the start iterator! There the iterator will store the
    // logical position (0 = oldest, buffer.size = past newest)
    // Implementation details after ideas from
    //   - https://stackoverflow.com/a/7759622
    //   - http://www.sj-vs.net/c-implementing-const_iterator-and-non-const-iterator-without-code-duplication/
    //   - https://quuxplusone.github.io/blog/2018/12/01/const-iterator-antipatterns/
    template <bool is_const>
    class Iterator {
    public:
        using iterator_category = std::random_access_iterator_tag;
        using value_type = CircularBuffer::value_type;
        using difference_type = CircularBuffer::difference_type;
        using reference =
                typename std::conditional_t<is_const, CircularBuffer::const_reference, CircularBuffer::reference>;
        using pointer = typename std::conditional_t<is_const, CircularBuffer::const_pointer, CircularBuffer::pointer>;

        explicit Iterator(std::function<reference(int const&)> func, size_t i) : deref_func_(func), index_(i) {}
        // Besides the trivial copy constructors, also define the copy constructor from non-const to const.
        // Note that this constructor is explicitly not marked as "explicit" in order to get a converting constructor,
        // like done for stl iterators.
        template <bool is_const_ = is_const, typename = std::enable_if_t<is_const_>>
        Iterator(Iterator<false> const& o) : deref_func_(o.deref_func_), index_(o.index_) {}

        reference operator*() { return deref_func_(index_); }

        // Besides the trivial assignment operators, also define the assignment operator from non-const to const
        template <bool is_const_ = is_const, typename = std::enable_if_t<is_const_>>
        Iterator& operator=(Iterator<false> const& o) {
            deref_func_ = o.deref_func_;
            index_ = o.index_;
            return *this;
        }

        template <bool o_is_const>
        bool operator==(Iterator<o_is_const> const& o) const {
            // Assumes the same function
            return index_ == o.index_;
        }
        template <bool o_is_const>
        bool operator!=(Iterator<o_is_const> const& o) const {
            return not(*this == o);
        }

        Iterator& operator++() {
            ++index_;
            return *this;
        }
        Iterator& operator--() {
            --index_;
            return *this;
        }
        Iterator& operator+=(std::size_t const& step) {
            index_ += step;
            return *this;
        }
        Iterator operator+(std::size_t const& step) const {
            Iterator result(*this);
            result += step;
            return result;
        }
        Iterator& operator-=(std::size_t const& step) {
            index_ -= step;
            return *this;
        }
        Iterator operator-(std::size_t const& step) const {
            Iterator result(*this);
            result -= step;
            return result;
        }

    private:
        friend class Iterator<true>;

        // As this class contains the logical index to the circular buffer, somehow the original container should be
        // accessible in order to "dereference"/look up the value based on the iterator. A (const) reference leads to
        // all kinds of problems, so therefore a function should be supplied at construction get the container value
        // based on the index.
        std::function<reference(int const&)> deref_func_;
        size_t index_;
    };

public:
    using iterator = Iterator<false>;
    using const_iterator = Iterator<true>;

    CircularBuffer() : index_{0}, count_{0}, buffer_() {}
    explicit CircularBuffer(size_t size) : index_{0}, count_{0}, buffer_(size) {}

    /// Add element to the buffer overwriting the oldest element.
    void PushBack(T&& in) {
        buffer_[index_] = std::move(in);
        count_ = detail::SaturatingCounter::Increment(count_, buffer_.size());
        index_ = CircularArithmetic::Increment(index_, buffer_.size());
    }

    /// Add element to the buffer overwriting the oldest element.
    void PushBack(T const& in) {
        buffer_[index_] = in;
        count_ = detail::SaturatingCounter::Increment(count_, buffer_.size());
        index_ = CircularArithmetic::Increment(index_, buffer_.size());
    }

    /// Remove the newest element from the buffer.
    bool PopBack(T* out) {
        if (count_ == 0) {
            return false;
        } else {
            count_ = detail::SaturatingCounter::Decrement(count_, 0);
            index_ = CircularArithmetic::Decrement(index_, buffer_.size());
            *out = std::move(buffer_[index_]);
            return true;
        }
    }

    /// Remove the oldest element from the buffer.
    bool PopFront(T* out) {
        if (count_ == 0) {
            return false;
        } else {
            size_t index = CircularArithmetic::Minus(index_, count_, buffer_.size());
            count_ = detail::SaturatingCounter::Decrement(count_, 0);
            *out = std::move(buffer_[index]);
            return true;
        }
    }

    void Clear() {
        count_ = 0;
        index_ = 0;
    }

    /// Change capacity of the buffer.
    void Resize(size_t const size) {
        if (size > Size()) {
            Grow(size);
        } else if (size < Size()) {
            Shrink(size);
        }
    }

    /// @brief: Logical access of the elements in order from oldest to newest, but with the supplied index wrapped.
    /// The buffer can be accessed in the logical/natural way in which asking for element 0 will return the oldest one.
    /// Similarly, size - 1 will result in the newest element. Input values are wrapped, so also -1 and size will return
    /// the newest element.
    T const& operator[](int const& index) const { return buffer_[LogicalIndex(index)]; }
    T& operator[](int const& index) { return buffer_[LogicalIndex(index)]; }

    /// Returns the newest element in the buffer
    T const& Back() const { return (*this)[-1]; }
    T& Back() { return (*this)[-1]; }

    /// Returns the oldest element in the buffer
    T const& Front() const { return (*this)[0]; }
    T& Front() { return (*this)[0]; }

    /// Returns the begin iterators
    // Function has to start with lowercase character in order to be used in a range-based for loop
    const_iterator begin() const {
        return const_iterator([this](int const& i) -> T const& { return (*this)[i]; }, 0);
    }
    iterator begin() {
        return iterator([this](int const& i) -> T& { return (*this)[i]; }, 0);
    }

    /// Returns the past-the-end iterators (which is actually valid!)
    // Function has to start with lowercase character in order to be used in a range-based for loop
    const_iterator end() const {
        return const_iterator([this](int const& i) -> T const& { return (*this)[i]; }, count_);
    }
    iterator end() {
        return iterator([this](int const& i) -> T& { return (*this)[i]; }, count_);
    }

    /// Returns number of elements currently in the buffer.
    size_t Count() const { return count_; }

    /// Returns size of the buffer including empty elements.
    size_t Size() const { return buffer_.size(); }

    bool Empty() const { return count_ == 0; }

    bool Full() const { return count_ == buffer_.size(); }

    /// Reorders the internal buffer so that Front is at index 0.
    void Reorder() {
        if (Full() && index_ == 0) {
            // buffer is full and ordered
            return;
        } else if (!Full() && index_ == count_) {
            // buffer is not full but is ordered
            return;
        }

        // Create resized buffer
        std::vector<T> vec(buffer_.size());

        // Move elements to resized buffer
        for (size_t i = 0; i < count_; ++i) {
            vec[i] = std::move((*this)[i]);
        }

        // Swap buffers
        std::swap(buffer_, vec);

        if (Full()) {
            // If buffer is full then index must point to 0 (oldest element)
            index_ = 0;
        } else {
            // If buffer is not full then index must point to the next empty element
            index_ = count_;
        }
    }

    /// Returns underlying data.
    std::vector<T> const& Data() const { return buffer_; }
    std::vector<T>& Data() { return buffer_; }

private:
    size_t index_;  // Points to past the newest element
    size_t count_;
    std::vector<T, Alloc> buffer_;

    /// The logical index (0 = oldest, buffer.size - 1 = newest) is converted to the actual circular buffer index, incl.
    /// wrap around so -1 is also newest and buffer.size is oldest and so on.
    inline size_t LogicalIndex(int const& offset) const {
        // First calculate the relative position
        size_t result = CircularArithmetic::Offset(0, offset, count_);
        // Then calculate the absolute position
        result = CircularArithmetic::Offset(index_, static_cast<int>(result) - count_, Size());
        return result;
    }

    /// Increase the size of the buffer to @param{size}
    /// Preserves all elements.
    void Grow(size_t const size) {
        // First reorder current buffer, then simply resize it to be bigger
        Reorder();
        buffer_.resize(size);
        // Index must point to next empty element
        index_ = count_;
    }

    /// Decrease the size of the buffer to @param{size}
    /// Preserves only youngest elements.
    void Shrink(size_t const size) {
        // Create resized buffer
        std::vector<T> vec(size);
        T t;
        auto iter = vec.rbegin();
        size_t count_saved = count_;

        // Copy elements to resized buffer, starting with the youngest element
        while (iter != vec.rend() && PopBack(&t)) {
            *iter = t;
            ++iter;
        }

        // Swap buffers
        std::swap(buffer_, vec);

        // Check if some elements were lost in resizing
        if (count_saved > size) {
            // Elements were lost so the current buffer is full
            count_ = size;
        } else {
            // Elements were not lost, buffer is not full
            count_ = count_saved;
        }
        if (!Full()) {
            index_ = count_;
        } else {
            index_ = 0;
        }
    }
};

}  // end namespace nie
