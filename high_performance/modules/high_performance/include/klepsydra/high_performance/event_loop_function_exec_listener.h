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

#ifndef EVENT_LOOP_FUNCTION_EXEC_LISTENER_H
#define EVENT_LOOP_FUNCTION_EXEC_LISTENER_H

#include <functional>
#include <memory>
#include <string>

#include <klepsydra/core/core_container.h>
#include <klepsydra/core/safe_event_emitter.h>

#include <klepsydra/high_performance/eventloop_data_type.h>

namespace kpsr {
namespace high_performance {
/**
 * @brief The EventLoopFunctionExecutorListener class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-eventloop-internal
 *
 */
class EventLoopFunctionExecutorListener
{
public:
    /**
     * @brief EventLoopFunctionExecutorListener
     * @param container
     * @param eventEmitter
     * @param eventName
     */
    EventLoopFunctionExecutorListener(
        Container *container,
        std::shared_ptr<EventEmitterInterface<EventloopDataWrapper>> externalEventEmitter,
        const std::string &eventName)
        : _container(container)
        , _eventName(eventName)
        , _externalEventEmitter(externalEventEmitter)
    {
        _eventLoopListener = [](const EventloopDataWrapper &eventDataType) {
            std::shared_ptr<const std::function<void()>> functionEvent =
                std::static_pointer_cast<const std::function<void()>>(eventDataType.eventData);
            (*functionEvent.get())();
        };
        _listenerId = _externalEventEmitter->on(this->_container,
                                                eventName + "_external",
                                                eventName + "_external",
                                                _eventLoopListener);
    }

    ~EventLoopFunctionExecutorListener()
    {
        _externalEventEmitter->removeListener(this->_container, _listenerId);
    }

    void setContainer(Container *container)
    {
        _container = container;
        if (this->_container != nullptr) {
            this->_container->attach(_externalEventEmitter->getListenerStats(_listenerId).get());
        }
    }

private:
    Container *_container;
    std::string _eventName;
    std::shared_ptr<EventEmitterInterface<EventloopDataWrapper>> _externalEventEmitter;
    std::function<void(const EventloopDataWrapper &)> _eventLoopListener;
    int _listenerId;
};
} // namespace high_performance
} // namespace kpsr
#endif // EVENT_LOOP_FUNCTION_EXEC_LISTENER_H
