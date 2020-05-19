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
    EventEmitterSubscriber(Container * container, EventEmitter & eventEmitter, const std::string & eventName)
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
    void registerListener(const std::string & name, const std::function<void(const T &)> listener) {
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
    void removeListener(const std::string & name) {
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
    std::shared_ptr<SubscriptionStats> getSubscriptionStats(const std::string & name) {
        return _eventEmitter._listenerStats[_listenersMap[name]];
    }

private:
    std::map<std::string, unsigned int> _listenersMap;
    typedef typename std::map<std::string, unsigned int>::iterator it_type;

    EventEmitter & _eventEmitter;
};
}
#endif
