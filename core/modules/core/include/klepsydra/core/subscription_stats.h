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
