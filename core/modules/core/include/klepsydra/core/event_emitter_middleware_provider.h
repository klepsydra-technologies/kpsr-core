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
