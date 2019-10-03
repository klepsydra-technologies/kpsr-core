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

#ifndef EVENT_EMITTER_MIDDLEWARE_PROVIDER_H
#define EVENT_EMITTER_MIDDLEWARE_PROVIDER_H

#include <klepsydra/core/event_emitter_publisher.h>
#include <klepsydra/core/event_emitter_subscriber.h>

#include <klepsydra/core/event_transform_forwarder.h>

namespace kpsr {
template <class T>
/*!
 * @brief The EventEmitterMiddlewareProvider class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-test
 *
 * @details This class a is facility wizard to create pub-sub pairs. It keeps tracks of them in a map. This particular implementation of a provider is intenteded for testing purposes only.
 *
 */
class EventEmitterMiddlewareProvider {
public:
    /*!
     * \brief EventEmitterMiddlewareProvider
     * \param container
     * \param eventName
     * \param poolSize
     * \param initializerFunction
     * \param eventCloner
     */
    EventEmitterMiddlewareProvider(Container * container,
                                   std::string eventName,
                                   int poolSize,
                                   std::function<void(T &)> initializerFunction,
                                   std::function<void(const T &, T &)> eventCloner)
        : _eventEmitter()
        , _eventName(eventName)
        , _publisher(container, eventName, _eventEmitter, poolSize, initializerFunction, eventCloner)
        , _subscriber(container, _eventEmitter, eventName)
    {}

    /*!
     * @brief getPublisher
     * @return
     */
    Publisher<T> * getPublisher() {
        return &_publisher;
    }

    /*!
     * @brief getSubscriber
     * @return
     */
    Subscriber<T> * getSubscriber() {
        return &_subscriber;
    }

    /*!
     * @brief getProcessForwarder creates a listener forwarder to transform or process an event on arrival and for further publication.
     * @param transformFunction
     * @return
     */
    template <class S>
    std::shared_ptr<EventTransformForwarder<S, T>>
    getProcessForwarder(const std::function<void(const S &, T &)> & transformFunction) {
        return std::shared_ptr<EventTransformForwarder<S, T>>(new EventTransformForwarder<S, T>(
                                                                  transformFunction,
                                                                  getPublisher()));
    }

private:
    EventEmitter _eventEmitter;
    std::string _eventName;
    EventEmitterPublisher<T> _publisher;
    EventEmitterSubscriber<T> _subscriber;
};
}

#endif // EVENT_EMITTER_MIDDLEWARE_PROVIDER_H
