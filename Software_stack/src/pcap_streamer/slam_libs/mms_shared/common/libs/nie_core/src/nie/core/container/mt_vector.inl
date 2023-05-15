/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
namespace nie {

namespace mt {

template <typename T, typename Alloc>
T MtVector<T, Alloc>::operator[](std::size_t i) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return vector_[i];
}

template <typename T, typename Alloc>
void MtVector<T, Alloc>::PushBack(T const& t) {
    std::lock_guard<std::mutex> lock(mutex_);
    vector_.push_back(t);
}

template <typename T, typename Alloc>
void MtVector<T, Alloc>::PushBack(T&& t) {
    std::lock_guard<std::mutex> lock(mutex_);
    vector_.push_back(std::move(t));
}

template <typename T, typename Alloc>
template <typename InputIterator>
void MtVector<T, Alloc>::Insert(InputIterator first, InputIterator last) {
    std::lock_guard<std::mutex> lock(mutex_);
    vector_.insert(vector_.end(), first, last);
}

template <typename T, typename Alloc>
bool MtVector<T, Alloc>::empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return vector_.size() == 0;
}

template <typename T, typename Alloc>
std::size_t MtVector<T, Alloc>::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return vector_.size();
}

}  // namespace mt

}  // namespace nie
