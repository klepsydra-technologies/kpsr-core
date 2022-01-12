/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*                            All Rights Reserved.
*
*  This file is subject to the terms and conditions defined in
*  file 'LICENSE.md', which is part of this source code package.
*
*  NOTICE:  All information contained herein is, and remains the property of Klepsydra
*  Technologies GmbH and its suppliers, if any. The intellectual and technical concepts
*  contained herein are proprietary to Klepsydra Technologies GmbH and its suppliers and
*  may be covered by Swiss and Foreign Patents, patents in process, and are protected by
*  trade secret or copyright law. Dissemination of this information or reproduction of
*  this material is strictly forbidden unless prior written permission is obtained from
*  Klepsydra Technologies GmbH.
*
****************************************************************************/

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
        _listenerId = _externalEventEmitter->on(this->_container,
                                                eventName + "_external",
                                                eventName + "_external",
                                                _eventLoopListener);
    }

    ~EventLoopSubscriber() { _externalEventEmitter->removeListener(this->_container, _listenerId); }

private:
    std::shared_ptr<EventEmitterInterface<EventloopDataWrapper>> _externalEventEmitter;
    std::string _eventName;
    std::function<void(const EventloopDataWrapper &)> _eventLoopListener;
    int _listenerId;
};
} // namespace high_performance
} // namespace kpsr
#endif
