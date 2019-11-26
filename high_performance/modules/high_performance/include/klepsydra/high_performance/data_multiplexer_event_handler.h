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

#ifndef DATA_MULTIPLEXER_EVENT_HANDLER_H
#define DATA_MULTIPLEXER_EVENT_HANDLER_H

#include <functional>
#include <iostream>
#include <memory>

#include <klepsydra/core/subscription_stats.h>

#include <klepsydra/high_performance/disruptor4cpp/disruptor4cpp.h>
#include <klepsydra/high_performance/data_multiplexer_event_data.h>

namespace kpsr
{
namespace high_performance
{
template <typename TEvent>
/**
 * @brief The DataMultiplexerEventHandler class
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
class DataMultiplexerEventHandler : public disruptor4cpp::event_handler<EventData<TEvent>>
{
public:
    /**
     * @brief DataMultiplexerEventHandler
     * @param listener
     * @param listenerStat
     */
    DataMultiplexerEventHandler(const std::function<void(TEvent)> & listener, std::shared_ptr<SubscriptionStats> listenerStat)
        : _listener(listener)
        , _listenerStat(listenerStat)
    {}

    ~DataMultiplexerEventHandler() = default;

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
     * @brief on_event
     * @param event
     * @param sequence
     * @param end_of_batch
     */
    void on_event(EventData<TEvent>& event, int64_t sequence, bool end_of_batch)
    {
        _listenerStat->_totalEnqueuedTimeInNs += TimeUtils::getCurrentNanosecondsAsLlu() - event.enqueuedTimeInNs;
        if (end_of_batch) {
            _listenerStat->startProcessMeassure();
            _listener(event.eventData);
            _listenerStat->stopProcessMeassure();
        } else {
            _listenerStat->_totalDiscardedEvents++;
        }
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
    void on_event_exception(const std::exception& ex, int64_t sequence, EventData<TEvent>* event) { }

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
    std::function<void(TEvent)> _listener;
    std::shared_ptr<SubscriptionStats> _listenerStat;

};
}
}

#endif // DATA_MULTIPLEXER_EVENT_HANDLER_H
