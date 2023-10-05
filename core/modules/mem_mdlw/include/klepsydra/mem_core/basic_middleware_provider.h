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

#ifndef BASIC_MIDDLEWARE_PROVIDER_H
#define BASIC_MIDDLEWARE_PROVIDER_H

#include <klepsydra/core/event_emitter_subscriber.h>
#include <klepsydra/sdk/event_transform_forwarder.h>

#include <klepsydra/mem_core/basic_publisher.h>
#include <klepsydra/mem_core/in_memory_middleware_provider.h>
#include <klepsydra/mem_core/safe_queue_poller.h>

namespace kpsr {
namespace mem {

template<class T>
/**
 * @brief The BasicMiddlewareProvider class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-mem-composition
 *
 * @details This class is a wizard that creates the safequeue, publishers and subscriber in combination
 * with some memory management functions.
 */
class BasicMiddlewareProvider : public InMemoryMiddlewareProvider<T>
{
public:
    /**
     * @brief BasicMiddlewareProvider
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
    BasicMiddlewareProvider(Container *container,
                            std::string eventName,
                            int queueSize,
                            int poolSize,
                            std::function<void(T &)> initializerFunction,
                            std::function<void(const T &, T &)> eventCloner,
                            bool discardItemsWhenFull)
        : InMemoryMiddlewareProvider<T>(container, eventName)
        , _internalQueue(queueSize)
    {
        this->_publisher = std::unique_ptr<BasicPublisher<T>>(
            new BasicPublisher<T>(container,
                                  eventName,
                                  poolSize,
                                  initializerFunction,
                                  eventCloner,
                                  _internalQueue,
                                  discardItemsWhenFull));
        // By default, we use a sleep/wait period of 1000 microseconds for the poller.
        this->_poller = std::unique_ptr<SafeQueuePoller<T>>(
            new SafeQueuePoller<T>(_internalQueue, this->_eventEmitter, eventName, 1000));
    }

    SafeQueue<EventData<const T>> _internalQueue;
};
} // namespace mem
} // namespace kpsr

#endif // BASIC_MIDDLEWARE_PROVIDER_H
