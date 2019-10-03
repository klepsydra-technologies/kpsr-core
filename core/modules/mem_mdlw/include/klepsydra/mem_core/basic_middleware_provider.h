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

#ifndef BASIC_MIDDLEWARE_PROVIDER_H
#define BASIC_MIDDLEWARE_PROVIDER_H

#include <klepsydra/core/event_transform_forwarder.h>
#include <klepsydra/core/event_emitter_subscriber.h>

#include <klepsydra/mem_core/in_memory_middleware_provider.h>
#include <klepsydra/mem_core/basic_publisher.h>
#include <klepsydra/mem_core/safe_queue_poller.h>

namespace kpsr {
namespace mem {

template <class T>
/**
 * @brief The BasicMiddlewareProvider class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-mem-composition
 *
 * @details This class is a wizard that creates the safequeue, publishers and subscriber in combination
 * with some memory management functions.
 */
class BasicMiddlewareProvider : public InMemoryMiddlewareProvider<T> {
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
    BasicMiddlewareProvider(Container * container,
                            std::string eventName,
                            int queueSize,
                            int poolSize,
                            std::function<void(T &)> initializerFunction,
                            std::function<void(const T &, T &)> eventCloner,
                            bool discardItemsWhenFull)
        : InMemoryMiddlewareProvider<T>(container, eventName)
        , _internalQueue(queueSize)
    {
        this->_publisher = new BasicPublisher<T>(container, eventName, poolSize, initializerFunction, eventCloner, _internalQueue, discardItemsWhenFull);
        // By default, we use a sleep/wait period of 1000 microseconds for the poller.
        this->_poller = new SafeQueuePoller<T>(_internalQueue, this->_eventEmitter, eventName, 1000);
    }

    SafeQueue <EventData<const T>>_internalQueue;

};
}
}

#endif // BASIC_MIDDLEWARE_PROVIDER_H
