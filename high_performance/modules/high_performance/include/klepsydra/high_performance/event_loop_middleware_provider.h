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

#ifndef EVENT_LOOP_MIDDLEWARE_PROVIDER_H
#define EVENT_LOOP_MIDDLEWARE_PROVIDER_H

#include "disruptor4cpp/disruptor4cpp.h"

#include <functional>

#include "event_loop.h"
#include "event_loop_publisher.h"
#include "event_loop_subscriber.h"

#include "event_loop_scheduler.h"
#include "event_loop_function_exec_listener.h"

namespace kpsr
{
namespace high_performance
{
template <std::size_t BufferSize>
/**
 * @brief The EventLoopMiddlewareProvider class
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
    using RingBuffer = disruptor4cpp::ring_buffer<EventloopDataWrapper, BufferSize, disruptor4cpp::blocking_wait_strategy, disruptor4cpp::producer_type::multi, disruptor4cpp::sequence>;

    /**
     * @brief EventLoopMiddlewareProvider
     * @param container
     */
    EventLoopMiddlewareProvider(Container * container, const std::string & name = "kpsr_EL")
        : _container(container)
        , _ringBuffer()
        , _eventEmitter()
        , _eventLoop(_eventEmitter, _ringBuffer, name)
        , _scheduler(nullptr)
    {}

    ~EventLoopMiddlewareProvider() { if (_scheduler) delete _scheduler;}

    template<class T>
    /**
     * @brief getPublisher retrieve an object pool based publisher associated to the event loop.
     * @param eventName
     * @param poolSize
     * @param initializerFunction
     * @param eventCloner
     * @return
     */
    Publisher<T> * getPublisher(std::string eventName,
                                int poolSize,
                                std::function<void(T &)> initializerFunction,
                                std::function<void(const T &, T &)> eventCloner) {
        auto search = _publisherMap.find(eventName);
        if (search != _publisherMap.end()) {
            std::shared_ptr<void> internalPointer = search->second;
            std::shared_ptr<Publisher<T>> publisher = std::static_pointer_cast<Publisher<T>>(internalPointer);
            return publisher.get();
        }
        else {
            std::shared_ptr<EventLoopPublisher<T, BufferSize>> publisher = std::make_shared<
                EventLoopPublisher<T, BufferSize>>(
                    _container, _ringBuffer, eventName, poolSize, initializerFunction, eventCloner);
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
    Subscriber<T> * getSubscriber(const std::string & eventName) {
        auto search = _subscriberMap.find(eventName);
        if (search != _subscriberMap.end()) {
            std::shared_ptr<void> internalPointer = search->second;
            std::shared_ptr<Subscriber<T>> subscriber = std::static_pointer_cast<Subscriber<T>>(internalPointer);
            return subscriber.get();
        }
        else {
            std::shared_ptr<EventLoopSubscriber<T>> subscriber = std::make_shared<EventLoopSubscriber<T>>(_container, _eventEmitter, eventName);
            std::shared_ptr<void> internalPointer = std::static_pointer_cast<void>(subscriber);
            _subscriberMap[eventName] = internalPointer;
            return std::static_pointer_cast<Subscriber<T>>(subscriber).get();
        }
    }

    /**
     * @brief place a function into the event loop. It can be placed for once or repeated execution.
     * @return
     */
    Scheduler * getScheduler(const std::string & name = "") {
        std::string eventName = name.empty() ? "EVENTLOOP_SCHEDULER" : name;
        if (_scheduler == nullptr) {
            std::shared_ptr<EventLoopFunctionExecutorListener> subscriber(new EventLoopFunctionExecutorListener(_container, _eventEmitter, eventName));
            std::shared_ptr<void> internalPointer = std::static_pointer_cast<void>(subscriber);
            _subscriberMap[eventName] = internalPointer;

            Publisher<std::function<void()>> * publisher = getPublisher<std::function<void()>>(eventName, 0, nullptr, nullptr);

            _scheduler = new EventLoopScheduler(publisher);
        }
        return _scheduler;
    }

    void start() {
        _eventLoop.start();
    }

    void stop() {
        _eventLoop.stop();
    }

    bool isRunning() {
        return _eventLoop.isStarted();
    }

    void setContainer(Container * container) {
        if (!isRunning()) {
            _container = container;
            if (_scheduler && (_subscriberMap.size() == 1) && (_publisherMap.size() == 1)) {
                if (_container) {
                    for (auto& keyValue : _subscriberMap) {
                        std::shared_ptr<EventLoopFunctionExecutorListener> schedulerSubscriber = std::static_pointer_cast<EventLoopFunctionExecutorListener>(keyValue.second);
                        schedulerSubscriber->setContainer(container);
                        Publisher<std::function<void()>> *schedulerPublisher = getPublisher<std::function<void()> > (keyValue.first, 0, nullptr, nullptr);
                        _container->attach(&schedulerPublisher->_publicationStats);
                    }
                }
            } else if ((_subscriberMap.size() > 0) || (_publisherMap.size() > 0 )) {
                spdlog::info("Container cannot be attached to already existing subscribers or publishers. Only subscribers/publishers declared after this call will attach to container.");
            }
        }
    }

private:

    Container * _container;
    RingBuffer _ringBuffer;
    EventEmitter _eventEmitter;
    EventLoop<BufferSize> _eventLoop;

    std::map<std::string, std::shared_ptr<void>> _publisherMap;
    std::map<std::string, std::shared_ptr<void>> _subscriberMap;
    EventLoopScheduler * _scheduler;
};
}
}
#endif
