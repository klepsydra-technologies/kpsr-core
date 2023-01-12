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

#ifndef DATA_MULTIPLEXER_EVENT_HANDLER_H
#define DATA_MULTIPLEXER_EVENT_HANDLER_H

#include <functional>
#include <iostream>
#include <memory>

#include <klepsydra/core/event_emitter_interface.h>

#include <klepsydra/high_performance/data_multiplexer_event_data.h>
#include <klepsydra/high_performance/disruptor4cpp/disruptor4cpp.h>

namespace kpsr {
namespace high_performance {
template<typename TEvent>
/**
 * @brief The DataMultiplexerEventHandler class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-high_performance-internal
 *
 * @details DataMultiplexer event handler implementation for Klepsydra. It invokes the map of listeners when new meesages
 * arrive. Very important to notice is that in the high_performance, only the last event is invoked. This means that
 * faster listeners will process all messages, while slower listener will only process the laster messages while
 * discarding any older ones.
 */
class DataMultiplexerEventHandler
    : public disruptor4cpp::event_handler<DataMultiplexerDataWrapper<TEvent>>
{
public:
    /**
     * @brief DataMultiplexerEventHandler
     * @param listener
     * @param listenerStat
     */
    DataMultiplexerEventHandler(
        const std::string &name,
        std::shared_ptr<EventEmitterInterface<std::shared_ptr<const TEvent>>> &eventEmitter)
        : _name(name)
        , _eventEmitter(eventEmitter)
    {}

    ~DataMultiplexerEventHandler() = default;

    /**
     * @brief on_start
     */
    void on_start() {}

    /**
     * @brief on_shutdown
     */
    void on_shutdown() {}

    /**
     * @brief on_event
     * @param event
     * @param sequence
     * @param end_of_batch
     */
    void on_event(DataMultiplexerDataWrapper<TEvent> &event, int64_t sequence, bool end_of_batch)
    {
        spdlog::debug("{}. Subscriber name: {}. sequence: {}, end_of_batch: {}",
                      __PRETTY_FUNCTION__,
                      _name,
                      sequence,
                      end_of_batch);
        if (end_of_batch) {
            _eventEmitter->emitEvent(_name,
                                     event.enqueuedTimeInNs,
                                     static_cast<std::shared_ptr<const TEvent>>(event.eventData));
        } else {
            _eventEmitter->discardEvent(_name);
        }
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
    void on_event_exception(const std::exception &ex,
                            int64_t sequence,
                            DataMultiplexerDataWrapper<TEvent> *event)
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
    const std::string _name;
    std::shared_ptr<EventEmitterInterface<std::shared_ptr<const TEvent>>> _eventEmitter;
};
} // namespace high_performance
} // namespace kpsr

#endif // DATA_MULTIPLEXER_EVENT_HANDLER_H
