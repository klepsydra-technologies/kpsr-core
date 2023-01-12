/*
 * Copyright 2023 Klepsydra Technologies AG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef IN_MEMORY_MIDDLEWARE_PROVIDER_H
#define IN_MEMORY_MIDDLEWARE_PROVIDER_H

#include <klepsydra/core/event_emitter_factory.h>
#include <klepsydra/core/event_emitter_subscriber.h>
#include <klepsydra/core/event_transform_forwarder.h>

#include <klepsydra/mem_core/in_memory_queue_poller.h>

namespace kpsr {
namespace mem {

template<class T>
/**
 * @brief The InMemoryMiddlewareProvider class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-mem-composition
 *
 * @details This class is an abstract class that provides the API for
 * the wizard that creates the internal queue, publishers and subscriber in
 * combination with some memory management functions.
 */
class InMemoryMiddlewareProvider
{
public:
    /**
     * @brief InMemoryMiddlewareProvider
     * @param container
     * @param eventName
     * @param poolSize
     */
    InMemoryMiddlewareProvider(Container *container,
                               std::string eventName,
                               EventEmitterType eventEmitterType = EventEmitterType::SAFE)
        : _eventEmitter(
              EventEmitterFactory::createEventEmitter<std::shared_ptr<const T>>(eventEmitterType))
        , _eventName(eventName)
        , _publisher(nullptr)
        , _poller(nullptr)
        , _subscriber(container, _eventEmitter, eventName)
    {}

    virtual ~InMemoryMiddlewareProvider()
    {
        delete _publisher;
        delete _poller;
    }

    /**
     * @brief start
     */
    virtual void start() { _poller->start(); }

    /**
     * @brief stop
     */
    virtual void stop() { _poller->stop(); }

    /**
     * @brief isRunning
     * @return
     */
    virtual bool isRunning() { return _poller->_running; }

    /**
     * @brief getPublisher
     * @return
     */
    Publisher<T> *getPublisher() { return _publisher; }

    /**
     * @brief getSubscriber
     * @return
     */
    Subscriber<T> *getSubscriber() { return &_subscriber; }

    template<class S>
    /**
     * @brief getProcessForwarder creates a listener forwarder to transform or process an event on arrival and for further publication.
     * @param transformFunction
     * @return
     */
    std::shared_ptr<EventTransformForwarder<S, T>> getProcessForwarder(
        const std::function<void(const S &, T &)> &transformFunction)
    {
        return std::shared_ptr<EventTransformForwarder<S, T>>(
            new EventTransformForwarder<S, T>(transformFunction, getPublisher()));
    }

    /**
     * @brief _eventEmitter
     */
    std::shared_ptr<EventEmitterInterface<std::shared_ptr<const T>>> _eventEmitter;
    /**
     * @brief _eventName
     */
    std::string _eventName;

protected:
    Publisher<T> *_publisher;
    InMemoryQueuePoller *_poller;
    EventEmitterSubscriber<T> _subscriber;
};
} // namespace mem
} // namespace kpsr

#endif // BASIC_MIDDLEWARE_PROVIDER_H
