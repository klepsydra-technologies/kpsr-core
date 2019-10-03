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
    Subscriber(Container * container, std::string name, std::string type)
        : _container(container)
        , _name(name)
        , _type(type)
    {}

    /*!
     * @brief registerListener registers an std::function to be invoked everything an event is received.
     * @param name with which the listener is registered.
     * @param listener function to be invoked for an event.
     */
    virtual void registerListener(const std::string name, const std::function<void(const T &)> listener) = 0;

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
    virtual void removeListener(const std::string name) = 0;

    /*!
     * @brief getSubscriptionStats retrieves the performance information of the listener.
     * @param name
     */
    virtual std::shared_ptr<SubscriptionStats> getSubscriptionStats(const std::string name) = 0;

    Container * _container;

    std::string _name;

    std::string _type;

};
}
#endif
