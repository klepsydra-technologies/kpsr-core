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

#ifndef EVENT_EMITTER_INTERFACE_H
#define EVENT_EMITTER_INTERFACE_H

#include <functional>
#include <memory>

#include <klepsydra/sdk/container.h>
#include <klepsydra/sdk/subscription_stats.h>

namespace kpsr {
/**
 * @brief The EventEmitterInterface class
 */
template<typename Event>
class EventEmitterInterface
{
public:
    /**
     * @brief on
     * @param eventId
     * @param listenerName
     * @param callback
     * @return
     */
    virtual unsigned int on(Container *container,
                            const std::string &eventId,
                            const std::string &listenerName,
                            std::function<void(const Event &event)> callback) = 0;

    /**
     * @brief once
     * @param eventId
     * @param callback
     * @return
     */
    virtual unsigned int once(const std::string &eventId,
                              std::function<void(const Event &event)> callback) = 0;

    /**
     * @brief removeListener
     * @param listenerId
     */
    virtual void removeListener(Container *container, unsigned int listenerId) = 0;

    /**
     * @brief emitEvent
     * @param eventId
     * @param enqueuedTimeNs
     * @param args
     */
    virtual void emitEvent(const std::string &eventId,
                           long long unsigned int enqueuedTimeNs,
                           const Event &args) = 0;

    /**
     * @brief discardEvent
     * @param eventId Subscriber on which event is received.
     *
     */
    virtual void discardEvent(const std::string &eventId) = 0;

    /**
     * @brief getSubscriptionStats
     * @param listenerId
     * @return
     */
    virtual std::shared_ptr<SubscriptionStats> getListenerStats(const unsigned int &listenerId) = 0;

    /**
     * @brief removeAllListeners
     *
     */
    virtual void removeAllListeners(Container *container) = 0;
};
} // namespace kpsr

#endif // EVENT_EMITTER_INTERFACE_H
