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

#ifndef CONCURRENT_MIDDLEWARE_PROVIDER_H
#define CONCURRENT_MIDDLEWARE_PROVIDER_H

#include <klepsydra/core/event_emitter_subscriber.h>
#include <klepsydra/sdk/event_transform_forwarder.h>

#include <concurrentqueue.h>
#include <klepsydra/mem_core/concurrent_queue_poller.h>
#include <klepsydra/mem_core/concurrent_queue_publisher.h>
#include <klepsydra/mem_core/in_memory_middleware_provider.h>

namespace kpsr {
namespace mem {

template<class T>
/**
 * @brief The ConcurrentMiddlewareProvider class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-mem-composition
 *
 * @details This class is a wizard that creates the safequeue (using
 * the lock free concurrent queue), publishers and subscriber in
 * combination with some memory management functions.
 */
class ConcurrentMiddlewareProvider : public InMemoryMiddlewareProvider<T>
{
public:
    /**
     * @brief ConcurrentMiddlewareProvider
     * @param container
     * @param eventName
     * @param queueSize
     * @param poolSize
     * @param initializerFunction function to be invoked after event instantiaion.
     * @param eventCloner a function used to clone events after copied in the publish method.
     * @param discardItemsWhenFull when true, old events will be deleted when the queue is full and new one need to be put.
     * In the false case, the publisher will block until there is free space to put new events in the queue, if the queue
     * is full.
     */
    ConcurrentMiddlewareProvider(Container *container,
                                 std::string eventName,
                                 int queueSize,
                                 int poolSize,
                                 std::function<void(T &)> initializerFunction,
                                 std::function<void(const T &, T &)> eventCloner,
                                 bool discardItemsWhenFull,
                                 unsigned int sleepPeriodUS)
        : InMemoryMiddlewareProvider<T>(container, eventName)
        , _internalQueue(queueSize, 1, 1)
        , _token(_internalQueue)
    {
        this->_publisher = std::unique_ptr<ConcurrentQueuePublisher<T>>(
            new ConcurrentQueuePublisher<T>(container,
                                            eventName,
                                            poolSize,
                                            initializerFunction,
                                            eventCloner,
                                            _internalQueue,
                                            discardItemsWhenFull,
                                            _token));
        this->_poller = std::unique_ptr<ConcurrentQueuePoller<T>>(
            new ConcurrentQueuePoller<T>(_internalQueue,
                                         this->_eventEmitter,
                                         eventName,
                                         sleepPeriodUS,
                                         _token));
    }

    /**
     * @brief _internalQueue
     */
    moodycamel::ConcurrentQueue<EventData<const T>> _internalQueue;

private:
    moodycamel::ProducerToken _token;
};
} // namespace mem
} // namespace kpsr

#endif // CONCURRENT_MIDDLEWARE_PROVIDER_H
