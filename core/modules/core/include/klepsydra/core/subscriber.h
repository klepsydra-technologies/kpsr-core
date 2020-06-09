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

#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include <string>
#include <functional>

#include <klepsydra/core/container.h>
#include <klepsydra/core/subscription_stats.h>

namespace kpsr
{
template <class T>
/*!
 * @brief The Subscriber class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-application
 *
 * @details Subscriber interface. It keeps track of the listeners that registers for events coming from a middleware.
 */
class Subscriber
{
public:

    /*!
     * @brief Subscriber
     * @param container
     * @param name
     * @param type
     */
    Subscriber(Container * container, const std::string& name, const std::string& type)
        : _container(container)
        , _name(name)
        , _type(type)
    {}

    virtual ~Subscriber() {}
    /*!
     * @brief registerListener registers an std::function to be invoked everything an event is received.
     * @param name with which the listener is registered.
     * @param listener function to be invoked for an event.
     */
    virtual void registerListener(const std::string & name, const std::function<void(const T &)> listener) = 0;

    /*!
     * @brief registerListenerOnce registers an std::function to be invoked when an event is received.
     * Once invoked this listeners is removed from the list of listeners.
     * @param listener function to be invoked for an event.
     */
    virtual void registerListenerOnce(const std::function<void(const T &)> listener) = 0;

    /*!
     * @brief removeListener removes the listener from the list of active listeners.
     * @param name
     */
    virtual void removeListener(const std::string & name) = 0;

    /*!
     * @brief getSubscriptionStats retrieves the performance information of the listener.
     * @param name
     */
    virtual std::shared_ptr<SubscriptionStats> getSubscriptionStats(const std::string & name) = 0;

    Container * _container;

    std::string _name;

    std::string _type;

};
}
#endif
