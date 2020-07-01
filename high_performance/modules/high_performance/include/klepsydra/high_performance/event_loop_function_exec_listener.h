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
    EventLoopFunctionExecutorListener(Container * container, EventEmitter & eventEmitter, const std::string & eventName)
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

    void setContainer(Container * container) {
        _container = container;
        if (this->_container != nullptr) {
            this->_container->attach(_externalEventEmitter._listenerStats[_listenerId].get());
        }
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
