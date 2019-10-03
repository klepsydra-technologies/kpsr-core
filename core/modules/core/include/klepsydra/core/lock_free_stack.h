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

#ifndef LOCK_FREE_STACK_H
#define LOCK_FREE_STACK_H

#include <atomic>
#include <iostream>

namespace kpsr {

template<typename T>
class LockFreeStack
{
    struct node_t final
    {
        T value;
        node_t *next;
    };

    struct head_t final
    {
        uintptr_t aba;
        node_t *node;
        head_t() noexcept :aba(0),node(nullptr)
        {}
        head_t(node_t* ptr) noexcept :aba(0),node(ptr)
        {}
    };

    using aligned_node_t = typename std::aligned_storage<sizeof(node_t), std::alignment_of<node_t>::value>::type;

    static_assert(sizeof(head_t) == 2*sizeof(uintptr_t), "Stack head should be 2 pointers size.");

    std::unique_ptr<aligned_node_t[]> buffer_ptr;
    node_t *node_buffer;
    std::atomic<head_t> head;
    std::atomic<head_t> free_nodes;

public:

    LockFreeStack(size_t capacity, std::function<void(T &)> initFunction)
    {
        size_t realCapacity = capacity;
        head.store( head_t(), std::memory_order_relaxed );
        // preallocate nodes
        buffer_ptr.reset(new aligned_node_t[realCapacity]);
        node_buffer = reinterpret_cast<node_t*>(buffer_ptr.get());
        for(size_t i = 0; i < realCapacity - 1; ++i)
        {
            if (initFunction != nullptr) {
                initFunction(node_buffer[i].value);
            }
            node_buffer[i].next = &node_buffer[i + 1];
        }
        if (initFunction != nullptr) {
            initFunction(node_buffer[realCapacity-1].value);
        }
        node_buffer[realCapacity-1].next = nullptr;
        free_nodes.store( head_t(node_buffer), std::memory_order_relaxed );
    }

    template<class U>
    bool push(U && data)
    {
        node_t *node = _pop(free_nodes);
        if (node == nullptr)
            return false;
        node->value = std::forward<U>(data);
        _push(head, node);
        return true;
    }

    bool pop(T& data)
    {
        node_t *node = _pop(head);
        if (node == nullptr)
            return false;
        data = std::move(node->value);
        _push(free_nodes, node);
        return true;
    }

private:
    node_t* _pop(std::atomic<head_t>& h)
    {
        head_t next, orig = h.load(std::memory_order_relaxed);
        do {
            if (orig.node == nullptr)
                return nullptr;
            next.aba = orig.aba + 1;
            next.node = orig.node->next;
        } while (!h.compare_exchange_weak(orig, next,
                                          std::memory_order_acq_rel,
                                          std::memory_order_acquire));
        return orig.node;
    }

    void _push(std::atomic<head_t>& h, node_t* node)
    {
        head_t next, orig = h.load(std::memory_order_relaxed);
        do {
            node->next = orig.node;
            next.aba = orig.aba + 1;
            next.node = node;
        } while (!h.compare_exchange_weak(orig, next,
                                          std::memory_order_acq_rel,
                                          std::memory_order_acquire));
    }
};
}

#endif // LOCK_FREE_STACK_H
