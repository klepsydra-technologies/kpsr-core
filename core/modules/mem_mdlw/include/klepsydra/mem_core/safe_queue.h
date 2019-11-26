/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*                            All Rights Reserved.
*
*  This file is subject to the terms and conditions defined in
*  file 'LICENSE.md', which is part of this source code package.
*
*  NOTICE:  All information contained herein is, and remains the property of Klepsydra
*  Technologies GmbH and its suppliers, if any. The intellectual and technical concepts
*  contained herein are proprietary to Klepsydra Technologies GmbH and its suppliers and
*  may be covered by Swiss and Foreign Patents, patents in process, and are protected by
*  trade secret or copyright law. Dissemination of this information or reproduction of
*  this material is strictly forbidden unless prior written permission is obtained from
*  Klepsydra Technologies GmbH.
*
****************************************************************************/

#ifndef SAFE_QUEUE_H
#define SAFE_QUEUE_H

#include <queue>
#include <list>
#include <mutex>
#include <thread>
#include <cstdint>
#include <condition_variable>

namespace kpsr {
namespace mem {

template <class T, class Container = std::list<T>>
/**
 * @brief The SafeQueue class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-mem-composition
 *
 * @details A thread-safe asynchronous blocking queue.
 *
 */
class SafeQueue
{

    typedef typename Container::value_type value_type;
    typedef typename Container::size_type size_type;
    typedef Container container_type;

public:

    /**
     * @brief SafeQueue
     * @param max_num_items
     */
    SafeQueue(unsigned int max_num_items)
        : m_max_num_items(max_num_items) {}

    /**
     * @brief SafeQueue
     * @param sq
     */
    SafeQueue (SafeQueue&& sq) {
        m_queue = std::move (sq.m_queue);
    }

    /**
     * @brief SafeQueue
     * @param sq
     */
    SafeQueue (const SafeQueue& sq) {
        std::lock_guard<std::mutex> lock (sq.m_mutex);
        m_queue = sq.m_queue;
    }


    ~SafeQueue()
    {
        std::lock_guard<std::mutex> lock (m_mutex);
    }

    /**
     * @brief set_max_num_items Sets the maximum number of items in the queue. Defaults is 0: No limit
     * @param max_num_items
     */
    void set_max_num_items (unsigned int max_num_items)
    {
        m_max_num_items = max_num_items;
    }

    /**
     * @brief push Pushes the item into the queue. It blocks if the queue is full
     * @param item An item.
     * @return true if an item was pushed into the queue
     */
    bool push (const value_type& item)
    {
        std::unique_lock<std::mutex> lock (m_mutex);
        m_non_full_condition.wait (lock, [this]() // Lambda funct
        {
                                       return m_queue.size() < m_max_num_items;
                                   });


        m_queue.push (item);
        m_non_empty_condition.notify_one();
        return true;
    }

    /**
     * @brief push Pushes the item into the queue. It blocks if the queue is full
     * @param item An item.
     * @return true if an item was pushed into the queue
     */
    bool push (const value_type&& item)
    {
        std::unique_lock<std::mutex> lock (m_mutex);
        m_non_full_condition.wait (lock, [this]() // Lambda funct
        {
                                       return m_queue.size() < m_max_num_items;
                                   });

        m_queue.push (item);
        m_non_empty_condition.notify_one();
        return true;
    }

    /**
     * @brief move_push Move ownership and pushes the item into the queue. It blocks if the queue is full
     * @param item An item.
     * @return true if an item was pushed into the queue
     */
    bool move_push (const value_type& item)
    {
        std::unique_lock<std::mutex> lock (m_mutex);
        m_non_full_condition.wait (lock, [this]() // Lambda funct
        {
                                       return m_queue.size() < m_max_num_items;
                                   });

        m_queue.push (std::move(item));
        m_non_empty_condition.notify_one();
        return true;
    }

    /**
     * @brief try_push Pushes the item into the queue. Not blocking call.
     * @param item An item.
     * @return true if an item was pushed into the queue
     */
    bool try_push (const value_type& item)
    {
        std::lock_guard<std::mutex> lock (m_mutex);

        if ((m_max_num_items > 0 && m_queue.size() >= m_max_num_items)) {
            return false;
        }

        m_queue.push (item);
        m_non_empty_condition.notify_one();
        return true;
    }

    /**
     * @brief try_push Pushes the item into the queue. Not blocking call.
     * @param item An item.
     * @return true if an item was pushed into the queue
     */
    bool try_push (const value_type&& item)
    {
        std::lock_guard<std::mutex> lock (m_mutex);

        if ((m_max_num_items > 0 && m_queue.size() >= m_max_num_items)) {
            return false;
        }

        m_queue.push (item);
        m_non_empty_condition.notify_one();
        return true;
    }

    /**
     * @brief try_push Move ownership and pushesthe item into the queue. Not blocking call.
     * @param item An item.
     * @return true if an item was pushed into the queue
     */
    bool try_move_push (const value_type& item)
    {
        std::lock_guard<std::mutex> lock (m_mutex);

        if ((m_max_num_items > 0 && m_queue.size() >= m_max_num_items)) {
            return false;
        }

        m_queue.push (std::move(item));
        m_non_empty_condition.notify_one();
        return true;
    }

    /**
     * @brief force_push Move ownership and pushesthe item into the queue, remove previous items if the queue is full.
     * @param item An item.
     * @return number of items that were removed in order to push the new one.
     */
    uint force_move_push (const value_type& item)
    {
        std::lock_guard<std::mutex> lock (m_mutex);

        uint discardedItems = 0;
        while ((m_max_num_items > 0 && m_queue.size() >= m_max_num_items)) {
            m_queue.pop();
            discardedItems++;
        }

        m_queue.push (std::move(item));
        m_non_empty_condition.notify_one();
        return discardedItems;
    }

    /**
     * @brief pop Pops item from the queue. If queue is empty, this function blocks until item becomes available.
     * @param item The item.
     */
    void pop (value_type& item)
    {
        std::unique_lock<std::mutex> lock (m_mutex);
        m_non_empty_condition.wait (lock, [this]() // Lambda funct
        {
                                        return !m_queue.empty();
                                    });
        item = m_queue.front();
        m_queue.pop();
        m_non_full_condition.notify_one();
    }

    /**
     * @brief move_pop Pops item from the queue using the contained type's move assignment operator, if it has one.
     * This method is identical to the pop() method if that type has no move assignment operator.
     * If queue is empty, this function blocks until item becomes available.
     * @param item The item.
     */
    void move_pop (value_type& item)
    {
        std::unique_lock<std::mutex> lock (m_mutex);
        m_non_empty_condition.wait (lock, [this]() // Lambda funct
        {
                                        return !m_queue.empty();
                                    });
        item = std::move (m_queue.front());
        m_queue.pop();
        m_non_full_condition.notify_one();
    }

    /**
     * @brief try_pop Tries to pop item from the queue.
     * @param item The item.
     * @return False is returned if no item is available.
     */
    bool try_pop (value_type& item)
    {
        std::unique_lock<std::mutex> lock (m_mutex);

        if (m_queue.empty())
            return false;

        item = m_queue.front();
        m_queue.pop();
        m_non_full_condition.notify_one();
        return true;
    }

    /**
     * @brief try_move_pop Tries to pop item from the queue using the contained type's move assignment operator, if it has one.
     * This method is identical to the try_pop() method if that type has no move assignment operator.
     * @param item The item.
     * @return False is returned if no item is available.
     */
    bool try_move_pop (value_type& item)
    {
        std::unique_lock<std::mutex> lock (m_mutex);

        if (m_queue.empty())
            return false;

        item = std::move (m_queue.front());
        m_queue.pop();
        m_non_full_condition.notify_one();
        return true;
    }

    /**
     * @brief timeout_pop Pops item from the queue. If the queue is empty, blocks for timeout microseconds, or until item becomes available.
     * @param item An item.
     * @param timeout The number of microseconds to wait.
     * @return  true if get an item from the queue, false if no item is received before the timeout.
     */
    bool timeout_pop (value_type& item, std::uint64_t timeout)
    {
        std::unique_lock<std::mutex> lock (m_mutex);

        if (m_queue.empty())
        {
            if (timeout == 0)
                return false;

            if (m_non_empty_condition.wait_for (lock, std::chrono::microseconds (timeout)) == std::cv_status::timeout)
                return false;
        }

        item = m_queue.front();
        m_queue.pop();
        m_non_full_condition.notify_one();
        return true;
    }

    /**
     * @brief timeout_move_pop Pops item from the queue using the contained type's move assignment operator, if it has one..
     * If the queue is empty, blocks for timeout microseconds, or until item becomes available.
     * This method is identical to the try_pop() method if that type has no move assignment operator.
     * @param item An item.
     * @param timeout The number of microseconds to wait.
     * @return
     * \return true if get an item from the queue, false if no item is received before the timeout.
     */
    bool timeout_move_pop (value_type& item, std::uint64_t timeout)
    {
        std::unique_lock<std::mutex> lock (m_mutex);

        if (m_queue.empty())
        {
            if (timeout == 0)
                return false;

            if (m_non_empty_condition.wait_for (lock, std::chrono::microseconds (timeout)) == std::cv_status::timeout)
                return false;
        }

        item = std::move (m_queue.front());
        m_queue.pop();
        m_non_full_condition.notify_one();
        return true;
    }

    /**
     * @brief size Gets the number of items in the queue.
     * @return Number of items in the queue.
     */
    size_type size() const
    {
        std::lock_guard<std::mutex> lock (m_mutex);
        return m_queue.size();
    }

    /**
     * @brief empty Check if the queue is empty.
     * @return true if queue is empty.
     */
    bool empty() const
    {
        std::lock_guard<std::mutex> lock (m_mutex);
        return m_queue.empty();
    }

    /**
     * @brief full Check if the queue is full.
     * @return true if queue is full.
     */
    bool full() const
    {
        std::lock_guard<std::mutex> lock (m_mutex);
        return (m_max_num_items > 0 && m_queue.size() >= m_max_num_items);
    }

    /**
     * @brief swap Swaps the contents.
     * @param sq The SafeQueue to swap with 'this'.
     */
    void swap (SafeQueue& sq)
    {
        if (this != &sq)
        {
            std::lock_guard<std::mutex> lock1 (m_mutex);
            std::lock_guard<std::mutex> lock2 (sq.m_mutex);
            m_queue.swap (sq.m_queue);

            if (!m_queue.empty())
                m_non_empty_condition.notify_all();

            if (!sq.m_queue.empty())
                sq.m_non_empty_condition.notify_all();
        }
    }

    /**
     * @brief operator = The copy assignment operator
     * @param sq
     * @return
     */
    SafeQueue& operator= (const SafeQueue& sq)
    {
        if (this != &sq)
        {
            std::lock_guard<std::mutex> lock1 (m_mutex);
            std::lock_guard<std::mutex> lock2 (sq.m_mutex);
            std::queue<T, Container> temp {sq.m_queue};
            m_queue.swap (temp);

            if (!m_queue.empty())
                m_non_empty_condition.notify_all();
        }

        return *this;
    }

    /**
     * @brief operator = The move assignment operator
     * @param sq
     * @return
     */
    SafeQueue& operator= (SafeQueue && sq)
    {
        std::lock_guard<std::mutex> lock (m_mutex);
        m_queue = std::move (sq.m_queue);

        if (!m_queue.empty())  m_non_empty_condition.notify_all();

        return *this;
    }


private:

    std::queue<T, Container> m_queue;
    mutable std::mutex m_mutex;
    std::condition_variable m_non_empty_condition;
    std::condition_variable m_non_full_condition;
    unsigned int m_max_num_items = 0;
};
template <class T, class Container>
/**
 * @brief swap Swaps the contents of two SafeQueue objects.
 * @param q1
 * @param q2
 */
void swap (SafeQueue<T, Container>& q1, SafeQueue<T, Container>& q2)
{
    q1.swap (q2);
}
}
}
#endif /* SAFE_QUEUE_H */

