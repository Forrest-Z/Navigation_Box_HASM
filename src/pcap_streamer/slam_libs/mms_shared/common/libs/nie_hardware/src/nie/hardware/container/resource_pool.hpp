/* Copyright (C) 2020 by NavInfo Europe B.V. The Netherlands - All rights reserved
 * Information classification: Confidential
 * This content is protected by international copyright laws.
 * Reproduction and distribution is prohibited without written permission. */
#ifndef NIE_HARDWARE_CONTAINER_RESOURCE_POOL_HPP
#define NIE_HARDWARE_CONTAINER_RESOURCE_POOL_HPP

#include <memory>
#include <mutex>
#include <stack>

namespace nie {

template <typename T>
class ResourcePool {
private:
    //    class ResourcePoolDeleter
    //    {
    //    public:
    //        explicit ResourcePoolDeleter(std::weak_ptr<ResourcePool<T>* > pool);
    //
    //        void operator()(T* ptr);
    //    private:
    //        std::weak_ptr<ResourcePool<T>*> pool_;
    //    };

public:
    // using Ptr = std::unique_ptr<T, ResourcePoolDeleter>;
    using Ptr = std::unique_ptr<T>;

    ResourcePool();

    // Conceptually it is bad to copy a resource pool.
    // Also... if you were allowed to copy there would be a problem with the shared_ptr to pointer that gets
    // copied and points to the wrong resource pool. Then people think they use a pool but they don't.
    ResourcePool(const ResourcePool&) = delete;
    ResourcePool& operator=(const ResourcePool&) = delete;

    virtual ~ResourcePool() = default;

    void Push(std::unique_ptr<T> t);
    // Attempts to pop from the pool. Undefined behaviour if the pool was empty.
    Ptr Pop();
    // Creates a new one if the pool was empty or otherwise returns a single resource.
    Ptr Get();

    bool empty() const;
    std::size_t size() const;

private:
    void Push_(std::unique_ptr<T> t);
    Ptr Pop_();
    Ptr Get_();

    bool empty_() const;
    std::size_t size_() const;

    // std::shared_ptr<ResourcePool<T>*> this_ptr_;
    std::stack<std::unique_ptr<T>> pool_;
    // Otherwise we have to discard various const qualifiers, which we don't want (really!)
    mutable std::mutex mutex_;
};

}  // namespace nie

#include "resource_pool.inl"

#endif  // NIE_HARDWARE_CONTAINER_RESOURCE_POOL_HPP
