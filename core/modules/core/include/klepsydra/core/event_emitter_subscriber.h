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

#ifndef EVENT_EMITTER_SUBSCRIBER_H
#define EVENT_EMITTER_SUBSCRIBER_H

#include <map>
#include <string>

#include <klepsydra/core/subscriber.h>
#include <klepsydra/core/event_emitter.h>

namespace kpsr
{
template<class T>
/*!
 * @brief The EventEmitterSubscriber class
 * 
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-composition
 *
 * @details Main implementation of the subscriber based on the event emitter class. It is basically a wrapper to the event emitter subscription API It is used in most of the Klepsydra subscription implementations.
 *
 */
class EventEmitterSubscriber : public Subscriber<T>
{
public:
    /*!
     * @brief EventEmitterSubscriber
     * @param container If not null, this container will be used to register all listeners passing through this subscriber instance.
     * @param eventEmitter The event emitter instance.
     * @param eventName A name to be assigned to this subscriber.
     */
    EventEmitterSubscriber(Container * container, EventEmitter & eventEmitter, std::string eventName)
        : Subscriber<T>(container, eventName, "EVENT_EMITTER")
        , _eventEmitter(eventEmitter)
    {}

    /*!
     * @brief registerListenerOnce
     * @param listener
     */
    void registerListenerOnce(const std::function<void(const T &)> listener) {
        _eventEmitter.once(this->_name, listener);
    }

    /*!
     * @brief registerListener
     * @param name
     * @param listener
     */
    void registerListener(const std::string name, const std::function<void(const T &)> listener) {
        unsigned int listenerId = _eventEmitter.on(this->_name, name, listener);
        _listenersMap[name] = listenerId;
        if (this->_container != nullptr) {
            this->_container->attach(_eventEmitter._listenerStats[listenerId].get());
        }
    }

    /*!
     * @brief removeListener
     * @param name
     */
    void removeListener(const std::string name) {
        if (_listenersMap.find(name) != _listenersMap.end()) {
            unsigned int listenerId = _listenersMap[name];
            if (this->_container != nullptr) {
                this->_container->detach(_eventEmitter._listenerStats[listenerId].get());
            }
            _eventEmitter.remove_listener(listenerId);
            _listenersMap.erase(name);
        }
    }

    /*!
     * @brief getSubscriptionStats retrieves the performance information of the listener.
     * @param name
     */
    std::shared_ptr<SubscriptionStats> getSubscriptionStats(const std::string name) {
        return _eventEmitter._listenerStats[_listenersMap[name]];
    }

private:
    std::map<std::string, unsigned int> _listenersMap;
    typedef typename std::map<std::string, unsigned int>::iterator it_type;

    EventEmitter & _eventEmitter;
};
}
#endif
