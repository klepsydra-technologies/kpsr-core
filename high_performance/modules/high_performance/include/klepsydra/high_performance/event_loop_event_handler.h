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

#ifndef EVENT_LOOP_EVENT_HANDLER_H
#define EVENT_LOOP_EVENT_HANDLER_H

#include <functional>
#include <iostream>
#include <memory>

#include <klepsydra/core/safe_event_emitter.h>

#include <klepsydra/high_performance/disruptor4cpp/disruptor4cpp.h>
#include <klepsydra/high_performance/eventloop_data_type.h>

namespace kpsr {
namespace high_performance {
/**
 * @brief The EventLoopEventHandler class
 *
 * @copyright 2023 Klepsydra Technologies AG
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
    EventLoopEventHandler(
        std::shared_ptr<EventEmitterInterface<EventloopDataWrapper>> externalEventEmitter)
        : _externalEventEmitter(externalEventEmitter)
    {
        if (externalEventEmitter == nullptr) {
            spdlog::error("External Event Emitter is null.");
        }
    }

    ~EventLoopEventHandler() = default;

    /**
     * @brief on_start
     */
    void on_start() {}

    /**
     * @brief on_shutdown
     */
    void on_shutdown() {}

    /**
     * @brief on_event as opossed to its high_performance equivalent, this implementation process all messages.
     * @param event
     * @param sequence
     * @param end_of_batch
     */
    void on_event(EventloopDataWrapper &event, int64_t sequence, bool end_of_batch)
    {
        _externalEventEmitter->emitEvent(event.eventName + "_external",
                                         event.enqueuedTimeInNs,
                                         event);
        event.eventData.reset();
    }

    /**
     * @brief on_timeout
     * @param sequence
     */
    void on_timeout(int64_t sequence) {}

    /**
     * @brief on_event_exception
     * @param ex
     * @param sequence
     * @param event
     */
    void on_event_exception(const std::exception &ex, int64_t sequence, EventloopDataWrapper *event)
    {}

    /**
     * @brief on_start_exception
     * @param ex
     */
    void on_start_exception(const std::exception &ex) {}

    /**
     * @brief on_shutdown_exception
     * @param ex
     */
    void on_shutdown_exception(const std::exception &ex) {}

private:
    std::shared_ptr<EventEmitterInterface<EventloopDataWrapper>> _externalEventEmitter;
};
} // namespace high_performance
} // namespace kpsr

#endif
