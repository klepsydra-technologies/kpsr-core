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

#ifndef EVENT_EMITTER_PUBLISHER_H
#define EVENT_EMITTER_PUBLISHER_H

#include <klepsydra/core/event_emitter_interface.h>
#include <klepsydra/core/object_pool_publisher.h>

/**
*/
namespace kpsr {
template<class T>
/*!
 * @brief The EventEmitterPublisher class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-test
 *
 * @details This publisher implementation is intended for testing purposes. It is synchronous and single-threaded. The goal is to use this publisher as a testing publisher for unit and integration testing. It derives from the ObjectPoolPublisher, which
 * allows to pre-allocate the message instances in a pool for increasing the performance.
 *
 */
class EventEmitterPublisher : public ObjectPoolPublisher<T>
{
public:
    /*!
     * @brief EventEmitterPublisher
     * @param container
     * @param eventName
     * @param eventEmitter
     * @param poolSize
     * @param initializerFunction
     * @param eventCloner
     */
    EventEmitterPublisher(
        Container *container,
        std::string eventName,
        std::shared_ptr<EventEmitterInterface<std::shared_ptr<const T>>> &eventEmitter,
        int poolSize,
        std::function<void(T &)> initializerFunction,
        std::function<void(const T &, T &)> eventCloner)
        : ObjectPoolPublisher<T>(container,
                                 eventName,
                                 "EVENT_EMITTER",
                                 poolSize,
                                 initializerFunction,
                                 eventCloner)
        , _eventEmitter(eventEmitter)
        , _eventName(eventName)
    {}

    /*!
     * @brief internalPublish publish events without copying.
     * @param event
     */
    void internalPublish(std::shared_ptr<const T> event) override
    {
        _eventEmitter->emitEvent(_eventName, 0, event);
    }

private:
    std::shared_ptr<EventEmitterInterface<std::shared_ptr<const T>>> _eventEmitter;
    std::string _eventName;
};
} // namespace kpsr
#endif
