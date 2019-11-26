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

#ifndef EVENT_LOOP_EVENT_HANDLER_H
#define EVENT_LOOP_EVENT_HANDLER_H

#include <functional>
#include <iostream>
#include <memory>

#include <klepsydra/core/event_emitter.h>

#include <klepsydra/high_performance/eventloop_data_type.h>
#include <klepsydra/high_performance/disruptor4cpp/disruptor4cpp.h>

namespace kpsr
{
namespace high_performance
{
/**
 * @brief The EventLoopEventHandler class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-eventloop-internal
 *
 */
class EventLoopEventHandler : public disruptor4cpp::event_handler<EventloopDataWrapper>
{
public:
    /**
     * @brief EventLoopEventHandler
     * @param eventEmitter
     */
    EventLoopEventHandler(EventEmitter & eventEmitter)
        : _eventEmitter(eventEmitter)
    {}

    ~EventLoopEventHandler() = default;

    /**
     * @brief on_start
     */
    void on_start() {
    }

    /**
     * @brief on_shutdown
     */
    void on_shutdown() { }

    /**
     * @brief on_event as opossed to its high_performance equivalent, this implementation process all messages.
     * @param event
     * @param sequence
     * @param end_of_batch
     */
    void on_event(EventloopDataWrapper& event, int64_t sequence, bool end_of_batch) {
        _eventEmitter.emitEvent(event.eventName, event.enqueuedTimeInNs, event);
    }

    /**
     * @brief on_timeout
     * @param sequence
     */
    void on_timeout(int64_t sequence) { }

    /**
     * @brief on_event_exception
     * @param ex
     * @param sequence
     * @param event
     */
    void on_event_exception(const std::exception& ex, int64_t sequence, EventloopDataWrapper* event) { }

    /**
     * @brief on_start_exception
     * @param ex
     */
    void on_start_exception(const std::exception& ex) { }

    /**
     * @brief on_shutdown_exception
     * @param ex
     */
    void on_shutdown_exception(const std::exception& ex) { }

private:
    EventEmitter & _eventEmitter;

};
}
}

#endif
