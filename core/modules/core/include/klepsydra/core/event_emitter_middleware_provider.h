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

#ifndef EVENT_EMITTER_MIDDLEWARE_PROVIDER_H
#define EVENT_EMITTER_MIDDLEWARE_PROVIDER_H

#include <klepsydra/core/event_emitter_factory.h>
#include <klepsydra/core/event_emitter_publisher.h>
#include <klepsydra/core/event_emitter_subscriber.h>

#include <klepsydra/sdk/event_transform_forwarder.h>

namespace kpsr {
template<class T>
/*!
 * @brief The EventEmitterMiddlewareProvider class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-test
 *
 * @details This class a is facility wizard to create pub-sub pairs. It keeps tracks of them in a map. This particular implementation of a provider is intenteded for testing purposes only.
 *
 */
class EventEmitterMiddlewareProvider
{
public:
    /*!
     * \brief EventEmitterMiddlewareProvider
     * \param container
     * \param eventName
     * \param poolSize
     * \param initializerFunction
     * \param eventCloner
     */
    EventEmitterMiddlewareProvider(Container *container,
                                   std::string eventName,
                                   int poolSize,
                                   std::function<void(T &)> initializerFunction,
                                   std::function<void(const T &, T &)> eventCloner,
                                   EventEmitterType eventEmitterType = EventEmitterType::SAFE)
        : _eventEmitter(
              EventEmitterFactory::createEventEmitter<std::shared_ptr<const T>>(eventEmitterType))
        , _eventName(eventName)
        , _publisher(container, eventName, _eventEmitter, poolSize, initializerFunction, eventCloner)
        , _subscriber(container, _eventEmitter, eventName)
    {}

    /*!
     * @brief getPublisher
     * @return
     */
    Publisher<T> *getPublisher() { return &_publisher; }

    /*!
     * @brief getSubscriber
     * @return
     */
    Subscriber<T> *getSubscriber() { return &_subscriber; }

    /*!
     * @brief getProcessForwarder creates a listener forwarder to transform or process an event on arrival and for further publication.
     * @param transformFunction
     * @return
     */
    template<class S>
    std::shared_ptr<EventTransformForwarder<S, T>> getProcessForwarder(
        const std::function<void(const S &, T &)> &transformFunction)
    {
        return std::shared_ptr<EventTransformForwarder<S, T>>(
            new EventTransformForwarder<S, T>(transformFunction, getPublisher()));
    }

private:
    std::shared_ptr<EventEmitterInterface<std::shared_ptr<const T>>> _eventEmitter;
    std::string _eventName;
    EventEmitterPublisher<T> _publisher;
    EventEmitterSubscriber<T> _subscriber;
};
} // namespace kpsr

#endif // EVENT_EMITTER_MIDDLEWARE_PROVIDER_H
