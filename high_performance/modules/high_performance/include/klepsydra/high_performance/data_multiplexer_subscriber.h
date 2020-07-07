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

#ifndef DATA_MULTIPLEXER_SUBSCRIBER_H
#define DATA_MULTIPLEXER_SUBSCRIBER_H

#include <mutex>
#include <chrono>
#include <map>
#include <memory>
#include <iostream>
#include <spdlog/spdlog.h>

#include <klepsydra/core/subscriber.h>

#include <klepsydra/high_performance/disruptor4cpp/disruptor4cpp.h>
#include <klepsydra/high_performance/data_multiplexer_listener.h>
#include <klepsydra/high_performance/data_multiplexer_event_data.h>

namespace kpsr
{
namespace high_performance
{
template <typename TEvent, std::size_t BufferSize>
/**
 * @brief The DataMultiplexerSubscriber class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-high_performance-composition
 *
 * @details This class is not actually used by the client code, but it is documented due to its close relation to the
 * provider. This class has the same API as most subscribers, but it is not based in the event emmitter as the most
 * of them. Very important to notice is that in the high_performance, only the last event is invoked. This means that
 * faster listeners will process all messages, while slower listener will only process the laster messages while
 * discarding any older ones.
 *
 */
class DataMultiplexerSubscriber : public Subscriber<TEvent> {
public:
    using RingBuffer = disruptor4cpp::ring_buffer<EventData<TEvent>, BufferSize, disruptor4cpp::blocking_wait_strategy, disruptor4cpp::producer_type::single, disruptor4cpp::sequence>;

    /**
     * @brief DataMultiplexerSubscriber
     * @param container
     * @param name
     * @param ringBuffer
     */
    DataMultiplexerSubscriber(Container * container, const std::string & name, RingBuffer & ringBuffer)
        : Subscriber<TEvent>(container, name, "DATA_MULTIPLEXER")
        , _ringBuffer(ringBuffer)
    {}

    /**
     * @brief registerListener
     * @param name
     * @param listener
     */
    void registerListener(const std::string & name, const std::function<void(const TEvent &)> listener) {
        std::lock_guard<std::mutex> lock (m_mutex);
        listenerStats.insert(std::make_pair(name, std::make_shared<kpsr::SubscriptionStats>(name, this->_name, "DATA_MULTIPLEXER")));
        if (this->_container != nullptr) {
            this->_container->attach(listenerStats[name].get());
        }
        subscriberMap.insert(std::make_pair(name, std::make_shared<DataMultiplexerListener<TEvent, BufferSize>>(listener, _ringBuffer, listenerStats[name])));
        subscriberMap[name]->start();
    }

    /**
     * @brief registerListenerOnce not support at the moment
     * @param listener
     */
    void registerListenerOnce(const std::function<void(const TEvent &)> listener) {
        // NOT SUPPORTED YET.
    }

    /**
     * @brief removeListener
     * @param name
     */
    void removeListener(const std::string & name) {
        std::lock_guard<std::mutex> lock (m_mutex);
        if (subscriberMap.find(name) != subscriberMap.end()) {
            subscriberMap[name]->stop();
            if (this->_container != nullptr) {
                this->_container->detach(listenerStats[name].get());
            }
            subscriberMap.erase(name);
        }
    }

    /**
     * @brief getSubscriptionStats getSubscriptionStats retrieves the performance information of the listener.
     * @param name
     * @return
     */
    std::shared_ptr<SubscriptionStats> getSubscriptionStats(const std::string & name) {
        return listenerStats[name];
    }

    /**
     * @brief subscriberMap
     */
    std::map<std::string, std::shared_ptr<DataMultiplexerListener<TEvent, BufferSize>>> subscriberMap;

    void setContainer(Container * container) {
        std::lock_guard<std::mutex> lock (m_mutex);
        this->_container = container;
        if (this->_container) {
            for (auto& keyValue: subscriberMap) {
                if (!keyValue.second->batchEventProcessor->is_running()) {
                    this->_container->attach(getSubscriptionStats(keyValue.first).get());
                } else {
                    spdlog::info("Cannot attach container to Subscriber listeners which are running.");
                }
            }
        }
    }
        
private:
    mutable std::mutex m_mutex;
    RingBuffer & _ringBuffer;
    std::map<std::string, std::shared_ptr<SubscriptionStats>> listenerStats;
};
}
}

#endif // DATA_MULTIPLEXER_SUBSCRIBER_H
