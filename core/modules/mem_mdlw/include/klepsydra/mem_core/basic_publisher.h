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

#ifndef BASIC_PUBLISHER_H
#define BASIC_PUBLISHER_H

#include <klepsydra/core/object_pool_publisher.h>

#include <klepsydra/mem_core/basic_event_data.h>
#include <klepsydra/mem_core/safe_queue.h>

namespace kpsr {
namespace mem {
template<class T>
/*!
 * \brief The BasicPublisher class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-mem-composition
 *
 * @details Publishing class that puts events in the safequeue. It has additional configuration to optimise
 * memory allocation.
 *
 */
class BasicPublisher : public ObjectPoolPublisher<T>
{
public:
    /**
     * @brief BasicPublisher
     * @param container
     * @param name
     * @param poolSize
     * @param initializerFunction function to be invoked after event instantiaion.
     * @param eventCloner a function used to clone events after copied in the publish method.
     * @param safeQueue
     * @param discardItemsWhenFull when true, old events will be deleted when the queue is full and new one need to be put.
     * In the false case, the publisher will block until there is free space to put new events in the queue, if the queue
     * is full.
     */
    BasicPublisher(Container *container,
                   const std::string &name,
                   int poolSize,
                   std::function<void(T &)> initializerFunction,
                   std::function<void(const T &, T &)> eventCloner,
                   SafeQueue<EventData<const T>> &safeQueue,
                   bool discardItemsWhenFull)
        : ObjectPoolPublisher<T>(container,
                                 name,
                                 "SAFE_QUEUE",
                                 poolSize,
                                 initializerFunction,
                                 eventCloner)
        , _internalQueue(safeQueue)
        , _discardItemsWhenFull(discardItemsWhenFull)
    {
        if (discardItemsWhenFull) {
            _internalQueuePush = [&](EventData<const T> &safeQueueEvent) {
                // Non-blocking call
                uint discardedItems = _internalQueue.force_move_push(safeQueueEvent);
                this->publicationStats.totalDiscardedEvents += discardedItems;
            };
        } else {
            _internalQueuePush = [&](EventData<const T> &safeQueueEvent) {
                // Blocking call
                _internalQueue.move_push(safeQueueEvent);
            };
        }
        if (container) {
            _updateTime = TimeUtils::getCurrentNanosecondsAsLlu;
        } else {
            _updateTime = []() { return 0llu; };
        }
    }

    /**
     * @brief internalPublish publish by a safe queue push into queue.
     * @param eventData
     */
    void internalPublish(std::shared_ptr<const T> eventData) override
    {
        EventData<const T> safeQueueEvent;
        safeQueueEvent.enqueuedTimeInNs = _updateTime();
        safeQueueEvent.eventData = eventData;
        _internalQueuePush(safeQueueEvent);
    }

private:
    SafeQueue<EventData<const T>> &_internalQueue;
    bool _discardItemsWhenFull;
    std::function<void(EventData<const T> &)> _internalQueuePush;
    std::function<long long unsigned int(void)> _updateTime;
};
} // namespace mem
} // namespace kpsr
#endif
