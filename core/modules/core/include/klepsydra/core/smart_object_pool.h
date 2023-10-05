/*
 * Copyright 2023 Klepsydra Technologies AG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SMART_OBJECT_POOL_H
#define SMART_OBJECT_POOL_H

#include <functional>
#include <klepsydra/core/lock_free_stack.h>
#include <memory>
#include <stdexcept>

#include <spdlog/spdlog.h>

namespace kpsr {
template<class T, class D = std::default_delete<T>>
/*!
 * @brief The SmartObjectPool class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-application
 *
 * @details Part of the memory allocation optimisation API, this class provide a generic way to create a pool objects. This class is available for developers to use, but it is also transparently integrated
 * into the publishers for the different middleware, for the disruptor and in other places of the Klepsydra core toolset.
 *
 */
class SmartObjectPool
{
private:
    struct ReturnToPool_Deleter
    {
        explicit ReturnToPool_Deleter(std::weak_ptr<SmartObjectPool<T, D> *> pool)
            : pool_(pool)
        {}

        void operator()(T *ptr)
        {
            if (auto pool_ptr = pool_.lock()) {
                spdlog::trace("{}. {}.", __PRETTY_FUNCTION__, (*pool_ptr)->_name);
                (*pool_ptr)->add(std::unique_ptr<T, D>{ptr});
            } else
                D{}(ptr);
        }

    private:
        std::weak_ptr<SmartObjectPool<T, D> *> pool_;
    };

    void add(std::unique_ptr<T, D> t) { pool_->push(std::move(t)); }

public:
    using ptr_type = std::unique_ptr<T, ReturnToPool_Deleter>;

    /*!
     * @brief SmartObjectPool
     * @param size
     * @param initializerFunction optional std::function to initialise the objects.
     */
    SmartObjectPool(const std::string &name,
                    int size,
                    std::function<void(T &)> initializerFunction = nullptr,
                    std::function<void(T &)> finalizerFunction = nullptr)
        : _finalizerFunction(finalizerFunction)
        , this_ptr_(std::make_shared<SmartObjectPool<T, D> *>(this))
        , _name(name)
    {
        std::function<void(std::unique_ptr<T, D> &)> poolInitializer =
            [&](std::unique_ptr<T, D> &data) {
                data.reset(new T());
                if (initializerFunction != nullptr) {
                    initializerFunction(*data);
                }
            };

        pool_ = new LockFreeStack<std::unique_ptr<T, D>>(size, poolInitializer);

        spdlog::trace("{}. Init function for smartpool {} and size {}",
                      __PRETTY_FUNCTION__,
                      name,
                      size);
    }

    virtual ~SmartObjectPool()
    {
        std::unique_ptr<T, D> t_ptr;
        while (pool_->pop(t_ptr)) {
            if (_finalizerFunction) {
                _finalizerFunction(*t_ptr);
            }
            t_ptr.reset();
        }
        delete pool_;
    }

    /*!
     * @brief acquire fetch an element of the pool. It comes as a unique pointer. When no references are hanging, the object will return to the pool.
     */
    ptr_type acquire()
    {
        spdlog::trace("{}. {}.", __PRETTY_FUNCTION__, _name);
        std::unique_ptr<T, D> element;
        if (!pool_->pop(element)) {
            objectPoolFails++;
            spdlog::error("Cannot acquire object from an empty pool {}.", _name);
            throw std::out_of_range("Cannot acquire object from an empty pool.");
        }

        ptr_type tmp(element.release(),
                     ReturnToPool_Deleter{std::weak_ptr<SmartObjectPool<T, D> *>{this_ptr_}});
        return tmp;
    }

    /*!
     * @brief empty
     */
    bool empty() const { throw std::exception("Unsupported operation."); }

    /*!
     * @brief size
     * @return
     */
    size_t size() const { throw std::exception("Unsupported operation."); }

    /*!
     * @brief objectPoolFails
     */
    long objectPoolFails = 0;

private:
    std::function<void(T &)> _finalizerFunction;
    std::shared_ptr<SmartObjectPool<T, D> *> this_ptr_;
    LockFreeStack<std::unique_ptr<T, D>> *pool_;
    const std::string _name;
};
} // namespace kpsr
#endif // SMART_OBJECT_POOL_H
