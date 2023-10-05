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

#ifndef LOCK_FREE_STACK_H
#define LOCK_FREE_STACK_H

#include <atomic>
#include <iostream>
#include <memory>
#include <vector>

namespace kpsr {

template<typename T>
class LockFreeStack
{
    static const int UNDEF_INDEX = 65535;

    // TODO see if constructor should throw if requested capacity is > (UNDEF_INDEX - 1)

    struct node_t final
    {
        uint16_t index_next;
    };

    struct head_t final
    {
        uint16_t aba;
        uint16_t node_index;
        head_t() noexcept
            : aba(0)
            , node_index(UNDEF_INDEX)
        {}
        head_t(node_t *ptr, node_t *start_addr) noexcept
            : aba(0)
            , node_index(
                  (reinterpret_cast<uintptr_t>(ptr) - reinterpret_cast<uintptr_t>(start_addr)) /
                  sizeof(node_t))
        {}
    };

    using aligned_node_t =
        typename std::aligned_storage<sizeof(node_t), std::alignment_of<node_t>::value>::type;

    static_assert(sizeof(head_t) == 2 * sizeof(uint16_t), "Stack head should be 2 pointers size.");

    std::unique_ptr<aligned_node_t[]> node_buffer_ptr;
    node_t *node_buffer;
    std::atomic<head_t> head;
    std::atomic<head_t> free_nodes;
    std::vector<T> data_buffer;

public:
    LockFreeStack(size_t capacity, std::function<void(T &)> initFunction)
    {
        size_t realCapacity = capacity;
        free_nodes.store(head_t(), std::memory_order_relaxed);
        // preallocate nodes
        node_buffer_ptr.reset(new aligned_node_t[realCapacity]);
        node_buffer = reinterpret_cast<node_t *>(node_buffer_ptr.get());
        data_buffer.resize(realCapacity);
        for (size_t i = 0; i < realCapacity - 1; ++i) {
            if (initFunction != nullptr) {
                initFunction(data_buffer[i]);
            }
            node_buffer[i].index_next = i + 1;
        }
        if (initFunction != nullptr) {
            initFunction(data_buffer[realCapacity - 1]);
        }
        node_buffer[realCapacity - 1].index_next = UNDEF_INDEX;
        head.store(head_t(node_buffer, &node_buffer[0]), std::memory_order_relaxed);
    }

    template<class U>
    bool push(U &&data)
    {
        node_t *node = _pop(free_nodes);
        if (node == nullptr)
            return false;
        data_buffer[_getIndex(node)] = std::forward<U>(data);
        _push(head, node);
        return true;
    }

    bool pop(T &data)
    {
        node_t *node = _pop(head);
        if (node == nullptr)
            return false;
        data = std::move(data_buffer[_getIndex(node)]);
        _push(free_nodes, node);
        return true;
    }

private:
    node_t *_pop(std::atomic<head_t> &h)
    {
        head_t next, orig = h.load(std::memory_order_relaxed);
        do {
            if (orig.node_index == UNDEF_INDEX)
                return nullptr;
            next.aba = orig.aba + 1;
            next.node_index = node_buffer[orig.node_index].index_next;
        } while (!h.compare_exchange_weak(orig,
                                          next,
                                          std::memory_order_acq_rel,
                                          std::memory_order_acquire));
        return &node_buffer[orig.node_index];
    }

    void _push(std::atomic<head_t> &h, node_t *node)
    {
        head_t next, orig = h.load(std::memory_order_relaxed);
        do {
            node->index_next = orig.node_index;
            next.aba = orig.aba + 1;
            next.node_index = _getIndex(node);
        } while (!h.compare_exchange_weak(orig,
                                          next,
                                          std::memory_order_acq_rel,
                                          std::memory_order_acquire));
    }

    int _getIndex(const node_t *node)
    {
        return (reinterpret_cast<uintptr_t>(node) - reinterpret_cast<uintptr_t>(node_buffer)) /
               sizeof(node_t);
    }
};
} // namespace kpsr

#endif // LOCK_FREE_STACK_H
