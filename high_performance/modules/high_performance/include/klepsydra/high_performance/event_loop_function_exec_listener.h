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

#ifndef EVENT_LOOP_FUNCTION_EXEC_LISTENER_H
#define EVENT_LOOP_FUNCTION_EXEC_LISTENER_H

#include <functional>
#include <string>
#include <memory>

#include <klepsydra/core/container.h>
#include <klepsydra/core/event_emitter.h>

#include <klepsydra/high_performance/eventloop_data_type.h>

namespace kpsr
{
namespace high_performance
{
/**
 * @brief The EventLoopFunctionExecutorListener class
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
    EventLoopFunctionExecutorListener(Container * container, EventEmitter & eventEmitter, std::string eventName)
        : _container(container)
        , _eventName(eventName)
        , _externalEventEmitter(eventEmitter)
    {
        _eventLoopListener = [this] (const EventloopDataWrapper & eventDataType) {
            std::shared_ptr<const std::function<void()>> functionEvent = std::static_pointer_cast<const std::function<void()>>(eventDataType.eventData);
            (*functionEvent.get())();
        };
        _listenerId =_externalEventEmitter.on(eventName, "FUNCTION_EXECUTOR", _eventLoopListener);
        if (this->_container != nullptr) {
            this->_container->attach(_externalEventEmitter._listenerStats[_listenerId].get());
        }
    }

    ~EventLoopFunctionExecutorListener() {
        if (this->_container != nullptr) {
            this->_container->detach(_externalEventEmitter._listenerStats[_listenerId].get());
        }
        _externalEventEmitter.remove_listener(_listenerId);
    }

private:
    Container * _container;
    std::string _eventName;
    EventEmitter & _externalEventEmitter;
    std::function<void(const EventloopDataWrapper &)> _eventLoopListener;
    int _listenerId;
};
}
}
#endif // EVENT_LOOP_FUNCTION_EXEC_LISTENER_H
