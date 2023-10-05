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

#ifndef EVENT_LOOP_SUBSCRIBER_H
#define EVENT_LOOP_SUBSCRIBER_H

#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <klepsydra/core/event_emitter_factory.h>
#include <klepsydra/core/event_emitter_subscriber.h>

#include <klepsydra/high_performance/eventloop_data_type.h>

namespace kpsr {
namespace high_performance {
template<class T>
/**
 * @brief The EventLoopSubscriber class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-eventloop-composition
 *
 * @details Although this function is not really used by the client code directly, it is documented due to its close
 * relation with the event loop. The EventLoopPublisher extends from the EventEmitterSubscriber and it is contains a
 * map of nested EventEmitterSubscriber one per pub/sub.
 */
class EventLoopSubscriber : public EventEmitterSubscriber<T>
{
public:
    /**
     * @brief EventLoopSubscriber
     * @param container
     * @param eventEmitter
     * @param eventName
     */
    EventLoopSubscriber(
        Container *container,
        std::shared_ptr<EventEmitterInterface<EventloopDataWrapper>> &externalEventEmitter,
        const std::string &eventName,
        EventEmitterType eventEmitterType)
        : EventEmitterSubscriber<T>(container, eventEmitterType, eventName)
        , _externalEventEmitter(externalEventEmitter)
        , _eventName(eventName)
    {
        _eventLoopListener = [this](const EventloopDataWrapper &eventDataType) {
            std::shared_ptr<const T> reinterpreted = std::static_pointer_cast<const T>(
                eventDataType.eventData);
            this->_eventEmitter->emitEvent(this->_eventName,
                                           eventDataType.enqueuedTimeInNs,
                                           reinterpreted);
        };
        _listenerId = _externalEventEmitter->on(this->container,
                                                eventName + "_external",
                                                eventName + "_external",
                                                _eventLoopListener);
    }

    ~EventLoopSubscriber() { _externalEventEmitter->removeListener(this->container, _listenerId); }

private:
    std::shared_ptr<EventEmitterInterface<EventloopDataWrapper>> _externalEventEmitter;
    std::string _eventName;
    std::function<void(const EventloopDataWrapper &)> _eventLoopListener;
    int _listenerId;
};
} // namespace high_performance
} // namespace kpsr
#endif
