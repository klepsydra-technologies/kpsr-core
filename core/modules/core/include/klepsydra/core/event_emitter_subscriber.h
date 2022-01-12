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

#include <klepsydra/core/event_emitter_factory.h>
#include <klepsydra/core/event_emitter_interface.h>
#include <klepsydra/core/subscriber.h>

namespace kpsr {
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
    EventEmitterSubscriber(
        Container *container,
        std::shared_ptr<EventEmitterInterface<std::shared_ptr<const T>>> &eventEmitter,
        const std::string &eventName)
        : Subscriber<T>(container, eventName, "EVENT_EMITTER")
        , _eventEmitter(eventEmitter)
    {}

    /*!
     * \brief EventEmitterSubscriber
     * \param container
     * \param eventEmitterType
     * \param eventName
     */
    EventEmitterSubscriber(Container *container,
                           EventEmitterType eventEmitterType,
                           const std::string &eventName)
        : Subscriber<T>(container, eventName, "EVENT_EMITTER")
        , _eventEmitter(
              EventEmitterFactory::createEventEmitter<std::shared_ptr<const T>>(eventEmitterType))
    {}

    ~EventEmitterSubscriber() { _eventEmitter->removeAllListeners(this->_container); }

    /*!
     * @brief registerListenerOnce
     * @param listener
     */
    void registerListenerOnce(const std::function<void(const T &)> listener)
    {
        if (!listener) {
            throw std::invalid_argument(
                "kpsr::EventEmitterSubscriber::registerListener: No callbak provided.");
        }

        std::function<void(const std::shared_ptr<const T> &)> internalListener =
            [listener](const std::shared_ptr<const T> &event) { listener(*event.get()); };
        _eventEmitter->once(this->_name, internalListener);
    }

    /*!
     * @brief registerListener
     * @param name
     * @param listener
     */
    void registerListener(const std::string &name, const std::function<void(const T &)> listener)
    {
        if (!listener) {
            throw std::invalid_argument(
                "kpsr::EventEmitterSubscriber::registerListener: No callbak provided.");
        }

        std::function<void(const std::shared_ptr<const T> &)> internalListener =
            [listener](const std::shared_ptr<const T> &event) { listener(*event.get()); };
        unsigned int listenerId = _eventEmitter->on(this->_container,
                                                    name,
                                                    this->_name,
                                                    internalListener);
        _listenersMap[name] = listenerId;
        spdlog::trace("{}. ListenerId {} for name: {}", __PRETTY_FUNCTION__, listenerId, name);
    }

    /*!
     * \brief registerSharedPtrListener
     * \param name
     * \param listener
     */
    virtual void registerSharedPtrListener(
        const std::string &name,
        const std::function<void(const std::shared_ptr<const T> &)> listener)
    {
        if (!listener) {
            throw std::invalid_argument(
                "kpsr::EventEmitterSubscriber::registerSharedPtrListener: No callbak provided.");
        }

        unsigned int listenerId = _eventEmitter->on(this->_container, name, this->_name, listener);
        _listenersMap[name] = listenerId;
    }

    /*!
     * \brief registerSharedPtrListenerOnce
     * \param listener
     */
    virtual void registerSharedPtrListenerOnce(
        const std::function<void(const std::shared_ptr<const T> &)> listener)
    {
        if (!listener) {
            throw std::invalid_argument("kpsr::EventEmitterSubscriber::"
                                        "registerSharedPtdListenerOnce: No callbak provided.");
        }

        _eventEmitter->once(this->_name, listener);
    }

    /*!
     * @brief removeListener
     * @param name
     */
    void removeListener(const std::string &name)
    {
        if (_listenersMap.find(name) != _listenersMap.end()) {
            unsigned int listenerId = _listenersMap[name];
            _eventEmitter->removeListener(this->_container, listenerId);
            _listenersMap.erase(name);
        }
    }

    /*!
     * @brief getSubscriptionStats retrieves the performance information of the listener.
     * @param name
     */
    std::shared_ptr<SubscriptionStats> getSubscriptionStats(const std::string &name)
    {
        spdlog::trace("{}. ListenerId {} for name: {}",
                      __PRETTY_FUNCTION__,
                      _listenersMap[name],
                      name);
        return _eventEmitter->getListenerStats(_listenersMap[name]);
    }

protected:
    std::shared_ptr<EventEmitterInterface<std::shared_ptr<const T>>> _eventEmitter;

private:
    std::map<std::string, unsigned int> _listenersMap;
    typedef typename std::map<std::string, unsigned int>::iterator it_type;
};
} // namespace kpsr
#endif
