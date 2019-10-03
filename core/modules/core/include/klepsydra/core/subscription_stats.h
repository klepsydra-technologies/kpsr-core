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

#ifndef SUBSCRIPTION_STATS_H
#define SUBSCRIPTION_STATS_H

#include <klepsydra/core/function_stats.h>

namespace kpsr {
/*!
 * @brief SubscriptionStats class.
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-monitoring
 *
 * @details Statistics associated to the performance of the listeners. The messures include the FunctionStats messures plus the total enqueued time of the events and the number of discarded events. 
 * Events can be discarded due to a number of reasons, depending on the actual underlying implementation of the middleware.
 */
struct SubscriptionStats : public FunctionStats {

    /*!
     * @brief _subscriberName
     * @param listenerName name of the listener
     * @param subscriberName name of the subscriber containing the listener
     * @param type type of the subscriber for information purpuses (examples are: EVENT_EMITTER, EVENT_LOOP, DISRUPTOR, ROS, ZMQ, DDS)
     * 
     */
    SubscriptionStats(const std::string listenerName, const std::string subscriberName, const std::string type)
        : FunctionStats(listenerName + "_" + subscriberName)
        , _type(type)
        , _totalEnqueuedTimeInNs(0)
        , _totalDiscardedEvents(0)
    {}

    /*!
     * @brief _subscriberName
     */
    const std::string _type;

    /*!
     * @brief _totalDiscardedEvents
     */
    std::atomic_ullong _totalEnqueuedTimeInNs;

    /*!
     * @brief _totalDiscardedEvents
     */
    std::atomic_ullong _totalDiscardedEvents;
};
}

#endif // SUBSCRIPTION_STATS_H
