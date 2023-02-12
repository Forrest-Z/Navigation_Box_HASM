/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
namespace nie {

// template <typename T>
// ResourcePool<T>::ResourcePoolDeleter::ResourcePoolDeleter(std::weak_ptr<ResourcePool<T>* > pool) : pool_(pool)
//{
//    //
//}
//
// template <typename T>
// void ResourcePool<T>::ResourcePoolDeleter::operator()(T* ptr)
//{
//    if (auto pool_ptr = pool_.lock())
//    {
//        try
//        {
//            (*pool_ptr.get())->Push(std::unique_ptr<T>(ptr));
//            return;
//        }
//        catch(...)
//        {
//
//        }
//    }
//    std::default_delete<T>()(ptr);
//}

template <typename T>
ResourcePool<T>::ResourcePool()  //: this_ptr_(std::make_shared<ResourcePool<T>*>(this))
{
    //
}

template <typename T>
void ResourcePool<T>::Push(std::unique_ptr<T> t) {
    std::lock_guard<std::mutex> lock(mutex_);
    Push_(std::move(t));
}

template <typename T>
typename ResourcePool<T>::Ptr ResourcePool<T>::Pop() {
    std::lock_guard<std::mutex> lock(mutex_);
    return Pop_();
}

template <typename T>
typename ResourcePool<T>::Ptr ResourcePool<T>::Get() {
    std::lock_guard<std::mutex> lock(mutex_);
    return Get_();
}

template <typename T>
bool ResourcePool<T>::empty() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return empty_();
}

template <typename T>
std::size_t ResourcePool<T>::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return size_();
}

template <typename T>
void ResourcePool<T>::Push_(std::unique_ptr<T> t) {
    pool_.push(std::move(t));
}

template <typename T>
typename ResourcePool<T>::Ptr ResourcePool<T>::Pop_() {
    // Ptr t(pool_.top().release(), ResourcePoolDeleter(this_ptr_ ));
    Ptr t = std::move(pool_.top());
    pool_.pop();

    return std::move(t);
}

template <typename T>
typename ResourcePool<T>::Ptr ResourcePool<T>::Get_() {
    if (empty_()) {
        return std::move(std::make_unique<T>());
        // return std::move(Ptr(new T(), ResourcePoolDeleter(this_ptr_ )));
    }

    return std::move(Pop_());
}

template <typename T>
bool ResourcePool<T>::empty_() const {
    return pool_.empty();
}

template <typename T>
std::size_t ResourcePool<T>::size_() const {
    return pool_.size();
}

}  // namespace nie
