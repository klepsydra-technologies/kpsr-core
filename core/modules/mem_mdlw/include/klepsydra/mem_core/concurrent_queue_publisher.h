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

#ifndef CONCURRENT_PUBLISHER_H
#define CONCURRENT_PUBLISHER_H

#include <klepsydra/core/object_pool_publisher.h>

#include <concurrentqueue.h>
#include <klepsydra/mem_core/basic_event_data.h>

namespace kpsr {
namespace mem {
template<class T>
/*!
 * \brief The ConcurrentPublisher class
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
class ConcurrentQueuePublisher : public ObjectPoolPublisher<T>
{
public:
    /**
     * @brief ConcurrentQueuePublisher
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
    ConcurrentQueuePublisher(Container *container,
                             const std::string &name,
                             int poolSize,
                             std::function<void(T &)> initializerFunction,
                             std::function<void(const T &, T &)> eventCloner,
                             moodycamel::ConcurrentQueue<EventData<const T>> &safeQueue,
                             bool discardItemsWhenFull,
                             moodycamel::ProducerToken &token)
        : ObjectPoolPublisher<T>(container,
                                 name,
                                 "SAFE_QUEUE",
                                 poolSize,
                                 initializerFunction,
                                 eventCloner)
        , _internalQueue(safeQueue)
        , _discardItemsWhenFull(discardItemsWhenFull)
        , _token(token)
    {
        if (discardItemsWhenFull) {
            _internalQueuePush = [&](EventData<const T> &safeQueueEvent) {
                // Non-blocking call
                uint discardedItems = 0;
                while (!_internalQueue.try_enqueue(_token, safeQueueEvent)) {
                    EventData<const T> dummyEvent;
                    bool removed = _internalQueue.try_dequeue_from_producer(_token, dummyEvent);
                    if (removed) {
                        discardedItems++;
                    }
                }
                this->publicationStats.totalDiscardedEvents += discardedItems;
            };
        } else {
            _internalQueuePush = [&](EventData<const T> &safeQueueEvent) {
                // Blocking call
                while (!_internalQueue.try_enqueue(_token, safeQueueEvent)) {
                    std::this_thread::sleep_for(std::chrono::microseconds(10));
                }
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
    moodycamel::ConcurrentQueue<EventData<const T>> &_internalQueue;
    bool _discardItemsWhenFull;
    moodycamel::ProducerToken &_token;
    std::function<void(EventData<const T> &)> _internalQueuePush;
    std::function<long long unsigned int(void)> _updateTime;
};
} // namespace mem
} // namespace kpsr
#endif
