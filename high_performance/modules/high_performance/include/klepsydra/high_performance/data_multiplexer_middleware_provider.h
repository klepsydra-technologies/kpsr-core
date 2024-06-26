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

#ifndef DATA_MULTIPLEXER_MIDDLEWARE_PROVIDER_H
#define DATA_MULTIPLEXER_MIDDLEWARE_PROVIDER_H

#include <functional>

#include <klepsydra/sdk/event_transform_forwarder.h>

#include <klepsydra/high_performance/data_multiplexer_event_data.h>
#include <klepsydra/high_performance/data_multiplexer_publisher.h>
#include <klepsydra/high_performance/data_multiplexer_subscriber.h>
#include <klepsydra/high_performance/disruptor4cpp/disruptor4cpp.h>

namespace kpsr {
namespace high_performance {
template<typename TEvent, std::size_t BufferSize>
/**
 * @brief The DataMultiplexerMiddlewareProvider class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-high_performance-composition
 *
 * @details Klepsydra provided wizar for the creation of high_performance and associated pub/sub pairs. Its use
 * is very straight forward as presented in this example:
@code
    // Create the high_performance provider
    kpsr::high_performance::DataMultiplexerMiddlewareProvider<DataMultiplexerTestEvent, 4> provider(nullptr, "test");

    kpsr::mem::CacheListener<DataMultiplexerTestEvent> eventListener(2);
    // get subscriber
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    for (int i = 0; i < 500; i ++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        DataMultiplexerTestEvent event(i, "hola");
        // get publisher
        provider.getPublisher()->publish(event);
    }
@endcode
 *
 */
class DataMultiplexerMiddlewareProvider
{
public:
    using RingBuffer = disruptor4cpp::ring_buffer<DataMultiplexerDataWrapper<TEvent>,
                                                  BufferSize,
                                                  disruptor4cpp::blocking_wait_strategy,
                                                  disruptor4cpp::producer_type::single,
                                                  disruptor4cpp::sequence>;

    /**
     * @brief DataMultiplexerMiddlewareProvider basic constructor
     * @param container
     * @param name
     * @param eventInitializer function to initialize the events allocated in the ring buffer
     * @param eventCloner cloner function used to add new events in the ring buffer when publishing.
     * @param cpuAffinityGeneratorFunction function that returns cpu id to use based on listener name
     * @param timoutUS max timeout for listener to start
     */
    DataMultiplexerMiddlewareProvider(
        Container *container,
        const std::string &name,
        std::function<void(TEvent &)> eventInitializer = nullptr,
        std::function<void(const TEvent &, TEvent &)> eventCloner = nullptr)
        : _name(name)
        , _container(container)
        , _modelEvent(nullptr)
        , _ringBuffer([&eventInitializer](DataMultiplexerDataWrapper<TEvent> &event) {
            if (eventInitializer) {
                eventInitializer(*event.eventData);
            }
        })
        , _publisher(container, name, eventCloner, _ringBuffer)
    {}

    /**
     * @brief DataMultiplexerMiddlewareProvider
     * @param container
     * @param name
     * @param event used as event to clone the events allocated in the ring buffer.
     * @param eventCloner cloner function used to add new events in the ring buffer when publishing.
     * @param cpuAffinityGeneratorFunction function that returns cpu id to use based on listener name
     * @param timoutUS max timeout for listener to start
     */
    DataMultiplexerMiddlewareProvider(
        Container *container,
        const std::string &name,
        const TEvent &event,
        std::function<void(const TEvent &, TEvent &)> eventCloner = nullptr)
        : _name(name)
        , _container(container)
        , _modelEvent(new DataMultiplexerDataWrapper<TEvent>(std::make_shared<TEvent>(event)))
        , _ringBuffer(_modelEvent.get())
        , _publisher(container, name, eventCloner, _ringBuffer)
    {}

    ~DataMultiplexerMiddlewareProvider() {}

    /**
     * @brief getPublisher
     * @return
     */
    Publisher<TEvent> *getPublisher() { return &_publisher; }

    /**
     * @brief getSubscriber
     * @return
     */
    Subscriber<TEvent> *getSubscriber(const std::string &subscriberName)
    {
        auto subscriberIterator = _subscriberMap.find(subscriberName);
        if (subscriberIterator != _subscriberMap.end()) {
            return subscriberIterator->second.get();
        }

        std::shared_ptr<DataMultiplexerSubscriber<TEvent, BufferSize>> subscriber =
            std::make_shared<DataMultiplexerSubscriber<TEvent, BufferSize>>(
                _container, _name, subscriberName, _ringBuffer, MULTIPLEXER_START_TIMEOUT_MICROSEC);

        _subscriberMap[subscriberName] = subscriber;
        return subscriber.get();
    }

    template<typename SourceEvent>
    std::shared_ptr<EventTransformForwarder<SourceEvent, TEvent>>
    /**
     * @brief getProcessForwarder
     * @param transformFunction
     */
    getProcessForwarder(const std::function<void(const SourceEvent &, TEvent &)> &transformFunction)
    {
        return std::make_shared<EventTransformForwarder<SourceEvent, TEvent>>(transformFunction,
                                                                              getPublisher());
    }

    void setContainer(Container *container)
    {
        _container = container;
        _publisher.setContainer(_container);
        for (auto subscriberNamePairs : _subscriberMap) {
            if (subscriberNamePairs.second->batchEventProcessor->is_running()) {
                spdlog::info("Cannot attach container to Subscriber listeners which are running.");
            }
            subscriberNamePairs.second->setContainer(_container);
        }
    }

private:
    std::string _name;
    Container *_container;
    std::unique_ptr<DataMultiplexerDataWrapper<TEvent>> _modelEvent;

    RingBuffer _ringBuffer;

    DataMultiplexerPublisher<TEvent, BufferSize> _publisher;
    std::map<std::string, std::shared_ptr<DataMultiplexerSubscriber<TEvent, BufferSize>>>
        _subscriberMap;
};
} // namespace high_performance
} // namespace kpsr
#endif // DATA_MULTIPLEXER_MIDDLEWARE_PROVIDER_H
