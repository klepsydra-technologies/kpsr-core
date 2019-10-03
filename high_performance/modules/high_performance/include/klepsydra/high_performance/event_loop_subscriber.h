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

#ifndef EVENT_LOOP_SUBSCRIBER_H
#define EVENT_LOOP_SUBSCRIBER_H

#include <map>
#include <string>
#include <memory>
#include <iostream>

#include <klepsydra/core/event_emitter_subscriber.h>

#include <klepsydra/high_performance/eventloop_data_type.h>

namespace kpsr
{
namespace high_performance
{
template<class T>
/**
 * @brief The EventLoopSubscriber class
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
    EventLoopSubscriber(Container * container, EventEmitter & eventEmitter, std::string eventName)
        : EventEmitterSubscriber<T>(container, _internalEventEmitter, eventName)
        , _externalEventEmitter(eventEmitter)
        , _internalEventEmitter()
        , _eventName(eventName)
    {
        _eventLoopListener = [this] (const EventloopDataWrapper & eventDataType) {
            std::shared_ptr<const T> reinterpreted = std::static_pointer_cast<const T>(eventDataType.eventData);
            this->_internalEventEmitter.emitEvent(this->_eventName, eventDataType.enqueuedTimeInNs, *reinterpreted.get());
        };
        _listenerId =_externalEventEmitter.on(eventName, "event_loop_subscriber", _eventLoopListener);
        if (this->_container != nullptr) {
            this->_container->attach(_externalEventEmitter._listenerStats[_listenerId].get());
        }
    }

    ~EventLoopSubscriber() {
        if (this->_container != nullptr) {
            this->_container->detach(_externalEventEmitter._listenerStats[_listenerId].get());
        }
        _externalEventEmitter.remove_listener(_listenerId);
    }

private:
    EventEmitter & _externalEventEmitter;
    EventEmitter _internalEventEmitter;
    std::string _eventName;
    std::function<void(const EventloopDataWrapper &)> _eventLoopListener;
    int _listenerId;
};
}
}
#endif
