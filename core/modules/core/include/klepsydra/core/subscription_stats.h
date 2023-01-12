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

#ifndef SUBSCRIPTION_STATS_H
#define SUBSCRIPTION_STATS_H

#include <klepsydra/core/function_stats.h>

namespace kpsr {
/*!
 * @brief SubscriptionStats class.
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-monitoring
 *
 * @details Statistics associated to the performance of the listeners. The messures include the FunctionStats messures plus the total enqueued time of the events and the number of discarded events. 
 * Events can be discarded due to a number of reasons, depending on the actual underlying implementation of the middleware.
 */
struct SubscriptionStats : public FunctionStats
{
    /*!
     * @brief _subscriberName
     * @param subscriberName name of the subscriber containing the listener
     * @param listenerName  name of the listener
     * @param type type of the subscriber for information purpuses (examples are: EVENT_EMITTER, EVENT_LOOP, DISRUPTOR, ROS, ZMQ, DDS)
     * 
     */
    SubscriptionStats(const std::string &subscriberName,
                      const std::string &listenerName,
                      const std::string &type)
        : FunctionStats(listenerName + "_" + subscriberName)
        , _type(type)
        , totalEnqueuedTimeInNs(0)
        , totalDiscardedEvents(0)
    {}

    /*!
     * @brief _subscriberName
     */
    const std::string _type;

    /*!
     * @brief totalEnqueuedTimeInNs
     */
    std::atomic_ullong totalEnqueuedTimeInNs;

    /*!
     * @brief totalDiscardedEvents
     */
    std::atomic_ullong totalDiscardedEvents;
};
} // namespace kpsr

#endif // SUBSCRIPTION_STATS_H
