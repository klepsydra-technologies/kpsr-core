/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************************/

#ifndef SMART_OBJECT_POOL_H
#define SMART_OBJECT_POOL_H

#include <memory>
#include <stdexcept>
#include <functional>
#include <klepsydra/core/lock_free_stack.h>

namespace kpsr {
template <class T, class D = std::default_delete<T>>
/*!
 * @brief The SmartObjectPool class
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
    struct ReturnToPool_Deleter {
        explicit ReturnToPool_Deleter(std::weak_ptr<SmartObjectPool<T, D>* > pool)
            : pool_(pool) {}

        void operator()(T* ptr) {
            if (auto pool_ptr = pool_.lock()) {
                (*pool_ptr.get())->add(std::unique_ptr<T, D>{ptr});
            }
            else
                D{}(ptr);
        }
    private:
        std::weak_ptr<SmartObjectPool<T, D>* > pool_;
    };

    void add(std::unique_ptr<T, D> t) {
        pool_->push(std::move(t));
    }

public:
    using ptr_type = std::unique_ptr<T, ReturnToPool_Deleter >;

    /*!
     * @brief SmartObjectPool
     * @param size
     * @param initializerFunction optional std::function to initialise the objects.
     */
    SmartObjectPool(int size, std::function<void(T &)> initializerFunction = nullptr)
        : this_ptr_(new SmartObjectPool<T, D>*(this))
    {
        std::function<void(std::unique_ptr<T, D> &)> poolInitializer = [&] (std::unique_ptr<T, D> & data) {
            data.release();
            data.reset(nullptr);
        };

        pool_ = new LockFreeStack<std::unique_ptr<T, D> >(size, poolInitializer);
        for (int i = 0; i < size; i ++) {
            T * t = new T();
            add(std::unique_ptr<T, D>(t));
            if (initializerFunction != nullptr) {
                initializerFunction(*t);
            }
        }
    }

    virtual ~SmartObjectPool(){}

    /*!
     * @brief acquire fetch an element of the pool. It comes as a unique pointer. When no references are hanging, the object will return to the pool.
     */
    ptr_type acquire() {
        std::unique_ptr<T, D> element;
        if (! pool_->pop(element)) {
            objectPoolFails++;
            throw std::out_of_range("Cannot acquire object from an empty pool.");
        }

        ptr_type tmp(element.release(),
                     ReturnToPool_Deleter{
                         std::weak_ptr<SmartObjectPool<T, D>*>{this_ptr_}});
        return std::move(tmp);
    }

    /*!
     * @brief empty
     */
    bool empty() const {
        throw std::exception("Unsupported operation.");
    }

    /*!
     * @brief size
     * @return
     */
    size_t size() const {
        throw std::exception("Unsupported operation.");
    }

    /*!
     * @brief objectPoolFails
     */
    long objectPoolFails = 0;

private:
    std::shared_ptr<SmartObjectPool<T, D>* > this_ptr_;
    LockFreeStack<std::unique_ptr<T, D> > * pool_;
};
}
#endif // SMART_OBJECT_POOL_H
