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

#ifndef IN_MEMORY_MIDDLEWARE_PROVIDER_H
#define IN_MEMORY_MIDDLEWARE_PROVIDER_H

#include <klepsydra/core/event_transform_forwarder.h>
#include <klepsydra/core/event_emitter_subscriber.h>

#include <klepsydra/mem_core/in_memory_queue_poller.h>

namespace kpsr {
namespace mem {

template <class T>
/**
 * @brief The InMemoryMiddlewareProvider class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-mem-composition
 *
 * @details This class is an abstract class that provides the API for
 * the wizard that creates the internal queue, publishers and subscriber in
 * combination with some memory management functions.
 */
class InMemoryMiddlewareProvider {
public:
    /**
     * @brief InMemoryMiddlewareProvider
     * @param container
     * @param eventName
     * @param poolSize
     */
    InMemoryMiddlewareProvider(Container * container,
                               std::string eventName)
        : _eventEmitter()
        , _eventName(eventName)
        , _publisher(nullptr)
        , _poller(nullptr)
        , _subscriber(container, _eventEmitter, eventName)
    {}

    virtual ~InMemoryMiddlewareProvider() {
        delete _publisher;
        delete _poller;
    }

    /**
     * @brief start
     */
    virtual void start () {
        _poller->start();
    }

    /**
     * @brief stop
     */
    virtual void stop() {
        _poller->stop();
    }

    /**
     * @brief isRunning
     * @return
     */
    virtual bool isRunning() {
        return _poller->_running;
    }

    /**
     * @brief getPublisher
     * @return
     */
    Publisher<T> * getPublisher() {
        return _publisher;
    }

    /**
     * @brief getSubscriber
     * @return
     */
    Subscriber<T> * getSubscriber() {
        return &_subscriber;
    }

    template<class S>
    /**
     * @brief getProcessForwarder creates a listener forwarder to transform or process an event on arrival and for further publication.
     * @param transformFunction
     * @return
     */
    std::shared_ptr<EventTransformForwarder<S, T>>
    getProcessForwarder(const std::function<void(const S &, T &)> & transformFunction) {
        return std::shared_ptr<EventTransformForwarder<S, T>>(new EventTransformForwarder<S, T>(
                                                                  transformFunction,
                                                                  getPublisher()));
    }

    /**
     * @brief _eventEmitter
     */
    EventEmitter _eventEmitter;
    /**
     * @brief _eventName
     */
    std::string _eventName;

protected:
    Publisher<T> * _publisher;
    InMemoryQueuePoller * _poller;
    EventEmitterSubscriber<T> _subscriber;
};
}
}

#endif // BASIC_MIDDLEWARE_PROVIDER_H
