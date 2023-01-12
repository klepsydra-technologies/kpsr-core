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

#ifndef EVENT_LOOP_MIDDLEWARE_PROVIDER_H
#define EVENT_LOOP_MIDDLEWARE_PROVIDER_H

#include "disruptor4cpp/disruptor4cpp.h"

#include <functional>

#include <klepsydra/core/event_emitter_interface.h>

#include "event_loop.h"
#include "event_loop_publisher.h"
#include "event_loop_subscriber.h"

#include "event_loop_function_exec_listener.h"
#include "event_loop_scheduler.h"

namespace kpsr {
namespace high_performance {
template<std::size_t BufferSize>
/**
 * @brief The EventLoopMiddlewareProvider class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-eventloop-composition
 *
 * @details Main eventloop wizard for creation of event loop and the associated pub/sub pairs. Its used is very
 * straightfoward as shown in this example:
 *
@code
    // create the provider. There can be as many as need per application.
    kpsr::high_performance::EventLoopMiddlewareProvider<4> provider(nullptr);
    provider.start();

    // retrieve a subscriber, notice the templating API;
    kpsr::Subscriber<ELTestEvent> * subscriber = provider.getSubscriber<ELTestEvent>("ELTestEvent");

    // retrieve a publisher, notice the templating API;
    kpsr::Publisher<ELTestEvent> * publisher = provider.getPublisher<ELTestEvent>("ELTestEvent", 0, nullptr, nullptr);

@endcode
 *
 * The eventloop provider includes a new API for placing function or services in the eventloop as a singlethread scheduler.
 * It is similar to the javascript event loop in this sense.
 *
 */
class EventLoopMiddlewareProvider
{
public:
    using RingBuffer = disruptor4cpp::ring_buffer<EventloopDataWrapper,
                                                  BufferSize,
                                                  disruptor4cpp::blocking_wait_strategy,
                                                  disruptor4cpp::producer_type::multi,
                                                  disruptor4cpp::sequence>;

    /**
     * @brief EventLoopMiddlewareProvider
     * @param container
     */
    EventLoopMiddlewareProvider(Container *container,
                                const std::string &name = "kpsr_EL",
                                const long timeoutUS = EVENT_LOOP_START_TIMEOUT_MICROSEC,
                                const std::vector<int> &cpuAffinity = {},
                                EventEmitterType eventEmitterType = EventEmitterType::UNSAFE_MULTI)
        : _container(container)
        , _ringBuffer()
        , _externalEventEmitter(
              EventEmitterFactory::createEventEmitter<EventloopDataWrapper>(eventEmitterType))
        , _eventLoop(_externalEventEmitter, _ringBuffer, name, timeoutUS)
        , _scheduler(nullptr)
    {}

    ~EventLoopMiddlewareProvider() {}

    template<class T>
    /**
     * @brief getPublisher retrieve an object pool based publisher associated to the event loop.
     * @param eventName
     * @param poolSize
     * @param initializerFunction
     * @param eventCloner
     * @return
     */
    Publisher<T> *getPublisher(std::string eventName,
                               int poolSize,
                               std::function<void(T &)> initializerFunction,
                               std::function<void(const T &, T &)> eventCloner)
    {
        auto search = _publisherMap.find(eventName);
        if (search != _publisherMap.end()) {
            std::shared_ptr<void> internalPointer = search->second;
            std::shared_ptr<Publisher<T>> publisher = std::static_pointer_cast<Publisher<T>>(
                internalPointer);
            return publisher.get();
        } else {
            std::shared_ptr<EventLoopPublisher<T, BufferSize>> publisher =
                std::make_shared<EventLoopPublisher<T, BufferSize>>(_container,
                                                                    _ringBuffer,
                                                                    eventName,
                                                                    poolSize,
                                                                    initializerFunction,
                                                                    eventCloner);
            std::shared_ptr<void> internalPointer = std::static_pointer_cast<void>(publisher);
            _publisherMap[eventName] = internalPointer;
            return std::static_pointer_cast<Publisher<T>>(publisher).get();
        }
    }

    template<class T>
    /**
     * @brief getSubscriber
     * @param eventName
     * @return
     */
    Subscriber<T> *getSubscriber(const std::string &eventName,
                                 EventEmitterType eventEmitterType = EventEmitterType::UNSAFE_MULTI)
    {
        auto search = _subscriberMap.find(eventName);
        if (search != _subscriberMap.end()) {
            std::shared_ptr<void> internalPointer = search->second;
            std::shared_ptr<Subscriber<T>> subscriber = std::static_pointer_cast<Subscriber<T>>(
                internalPointer);
            return subscriber.get();
        } else {
            std::shared_ptr<EventLoopSubscriber<T>> subscriber =
                std::make_shared<EventLoopSubscriber<T>>(_container,
                                                         _externalEventEmitter,
                                                         eventName,
                                                         eventEmitterType);
            std::shared_ptr<void> internalPointer = std::static_pointer_cast<void>(subscriber);
            _subscriberMap[eventName] = internalPointer;
            return std::static_pointer_cast<Subscriber<T>>(subscriber).get();
        }
    }

    /**
     * @brief place a function into the event loop. It can be placed for once or repeated execution.
     * @return
     */
    Scheduler *getScheduler(const std::string &name = "")
    {
        std::string eventName = name.empty() ? "EVENTLOOP_SCHEDULER" : name;
        if (_scheduler.get() == nullptr) {
            std::shared_ptr<EventLoopFunctionExecutorListener> subscriber =
                std::make_shared<EventLoopFunctionExecutorListener>(_container,
                                                                    _externalEventEmitter,
                                                                    eventName);
            std::shared_ptr<void> internalPointer = std::static_pointer_cast<void>(subscriber);
            _subscriberMap[eventName] = internalPointer;

            Publisher<std::function<void()>> *publisher =
                getPublisher<std::function<void()>>(eventName, 0, nullptr, nullptr);

            _scheduler = std::unique_ptr<EventLoopScheduler>(new EventLoopScheduler(publisher));
        }
        return _scheduler.get();
    }

    void start() { _eventLoop.start(); }

    void stop() { _eventLoop.stop(); }

    bool isRunning() { return _eventLoop.isStarted(); }

    void setContainer(Container *container)
    {
        if (!isRunning()) {
            _container = container;
            if (_scheduler && (_subscriberMap.size() == 1) && (_publisherMap.size() == 1)) {
                if (_container) {
                    for (auto &keyValue : _subscriberMap) {
                        std::shared_ptr<EventLoopFunctionExecutorListener> schedulerSubscriber =
                            std::static_pointer_cast<EventLoopFunctionExecutorListener>(
                                keyValue.second);
                        schedulerSubscriber->setContainer(container);
                        Publisher<std::function<void()>> *schedulerPublisher =
                            getPublisher<std::function<void()>>(keyValue.first, 0, nullptr, nullptr);
                        _container->attach(&schedulerPublisher->_publicationStats);
                    }
                }
            } else if ((_subscriberMap.size() > 0) || (_publisherMap.size() > 0)) {
                spdlog::info("Container cannot be attached to already existing subscribers or "
                             "publishers. Only subscribers/publishers declared after this call "
                             "will attach to container.");
            }
        }
    }

private:
    Container *_container;
    RingBuffer _ringBuffer;
    std::shared_ptr<EventEmitterInterface<EventloopDataWrapper>> _externalEventEmitter;
    EventLoop<BufferSize> _eventLoop;

    std::map<std::string, std::shared_ptr<void>> _publisherMap;
    std::map<std::string, std::shared_ptr<void>> _subscriberMap;
    std::unique_ptr<EventLoopScheduler> _scheduler;
};
} // namespace high_performance
} // namespace kpsr
#endif
