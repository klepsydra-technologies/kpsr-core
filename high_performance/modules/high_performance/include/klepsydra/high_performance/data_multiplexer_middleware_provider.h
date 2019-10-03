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

#ifndef DATA_MULTIPLEXER_MIDDLEWARE_PROVIDER_H
#define DATA_MULTIPLEXER_MIDDLEWARE_PROVIDER_H

#include <functional>

#include <klepsydra/core/event_transform_forwarder.h>

#include <klepsydra/high_performance/disruptor4cpp/disruptor4cpp.h>
#include <klepsydra/high_performance/data_multiplexer_publisher.h>
#include <klepsydra/high_performance/data_multiplexer_subscriber.h>
#include <klepsydra/high_performance/data_multiplexer_event_data.h>

namespace kpsr
{
namespace high_performance
{
template <typename TEvent, std::size_t BufferSize>
/**
 * @brief The DataMultiplexerMiddlewareProvider class
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
    using RingBuffer = disruptor4cpp::ring_buffer<EventData<TEvent>, BufferSize, disruptor4cpp::blocking_wait_strategy, disruptor4cpp::producer_type::single, disruptor4cpp::sequence>;

    /**
     * @brief DataMultiplexerMiddlewareProvider basic constructor without any performance tuning params
     * @param container
     * @param name
     */
    DataMultiplexerMiddlewareProvider(Container * container, std::string name)
        : _ringBuffer()
        , _publisher(container, name, nullptr, _ringBuffer)
        , _subscriber(container, name, _ringBuffer)
    {}

    /**
     * @brief DataMultiplexerMiddlewareProvider
     * @param container
     * @param name
     * @param eventInitializer function to initialize the events allocated in the ring buffer
     */
    DataMultiplexerMiddlewareProvider(Container * container, std::string name,
                                std::function<void(TEvent &)> eventInitializer)
        : _ringBuffer([&eventInitializer](EventData<TEvent> & event) { eventInitializer(event.eventData); })
        , _publisher(container, name, nullptr, _ringBuffer)
        , _subscriber(container, name, _ringBuffer)
    {}

    /**
     * @brief DataMultiplexerMiddlewareProvider
     * @param container
     * @param name
     * @param event used as event to clone the events allocated in the ring buffer.
     */
    DataMultiplexerMiddlewareProvider(Container * container, std::string name, const TEvent & event)
        : _modelEvent(new EventData<TEvent>(event))
        , _ringBuffer(_modelEvent)
        , _publisher(container, name, nullptr, _ringBuffer)
        , _subscriber(container, name, _ringBuffer)
    {}

    /**
     * @brief DataMultiplexerMiddlewareProvider
     * @param container
     * @param name
     * @param eventInitializer function to initialize the events allocated in the ring buffer
     * @param eventCloner cloner function used to add new events in the ring buffer when publishing.
     */
    DataMultiplexerMiddlewareProvider(Container * container, std::string name,
                                std::function<void(TEvent &)> eventInitializer,
                                std::function<void(const TEvent &, TEvent &)> eventCloner)
        : _ringBuffer([&eventInitializer](EventData<TEvent> & event) { eventInitializer(event.eventData); })
        , _publisher(container, name, eventCloner, _ringBuffer)
        , _subscriber(container, name, _ringBuffer)
    {}

    /**
     * @brief DataMultiplexerMiddlewareProvider
     * @param container
     * @param name
     * @param event used as event to clone the events allocated in the ring buffer.
     * @param eventCloner cloner function used to add new events in the ring buffer when publishing.
     */
    DataMultiplexerMiddlewareProvider(Container * container, std::string name,
                                const TEvent & event,
                                std::function<void(const TEvent &, TEvent &)> eventCloner)
        : _modelEvent(new EventData<TEvent>(event))
        , _ringBuffer(_modelEvent)
        , _publisher(container, name, eventCloner, _ringBuffer)
        , _subscriber(container, name, _ringBuffer)
    {}

    /**
     * @brief getPublisher
     * @return
     */
    Publisher<TEvent> * getPublisher() {
        return &_publisher;
    }

    /**
     * @brief getSubscriber
     * @return
     */
    Subscriber<TEvent> * getSubscriber() {
        return &_subscriber;
    }

    template<typename SourceEvent>
    std::shared_ptr<EventTransformForwarder<SourceEvent, TEvent>>
    /**
     * @brief getProcessForwarder
     * @param transformFunction
     */
    getProcessForwarder(const std::function<void(const SourceEvent &, TEvent &)> & transformFunction) {
        return std::shared_ptr<EventTransformForwarder<SourceEvent, TEvent>>(new EventTransformForwarder<SourceEvent, TEvent>(
                                                                                 transformFunction,
                                                                                 getPublisher()));
    }
private:
    EventData<TEvent> * _modelEvent;

    RingBuffer _ringBuffer;

    DataMultiplexerPublisher<TEvent, BufferSize> _publisher;
    DataMultiplexerSubscriber<TEvent, BufferSize> _subscriber;
};
}
}
#endif // DATA_MULTIPLEXER_MIDDLEWARE_PROVIDER_H
