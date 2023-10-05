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

#ifndef DATA_MULTIPLEXER_PUBLISHER_H
#define DATA_MULTIPLEXER_PUBLISHER_H

#include <functional>

#include <spdlog/spdlog.h>

#include <klepsydra/sdk/publisher.h>
#include <klepsydra/sdk/time_utils.h>

#include <klepsydra/high_performance/data_multiplexer_event_data.h>
#include <klepsydra/high_performance/disruptor4cpp/disruptor4cpp.h>

namespace kpsr {
namespace high_performance {
template<typename TEvent, std::size_t BufferSize>
/**
 * @brief The DataMultiplexerPublisher class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-high_performance-composition
 *
 * @details This class is not actually used by the client code, but it is documented due to its close relation to the
 * provider. When the method publish is invoked, this class puts a copy of the event into the ringbuffer.
 *
 */
class DataMultiplexerPublisher : public Publisher<TEvent>
{
public:
    using RingBuffer = disruptor4cpp::ring_buffer<DataMultiplexerDataWrapper<TEvent>,
                                                  BufferSize,
                                                  disruptor4cpp::blocking_wait_strategy,
                                                  disruptor4cpp::producer_type::single,
                                                  disruptor4cpp::sequence>;

    /**
     * @brief DataMultiplexerPublisher
     * @param container
     * @param name
     * @param eventCloner optional function used for cloning event that are put in the ring buffer.
     * @param ringBuffer
     */
    DataMultiplexerPublisher(Container *container,
                             const std::string &name,
                             std::function<void(const TEvent &, TEvent &)> eventCloner,
                             RingBuffer &ringBuffer)
        : Publisher<TEvent>(container, name, "DATA_MULTIPLEXER")
        , _ringBuffer(ringBuffer)
    {
        if (eventCloner) {
            _eventCloner = [eventCloner](const TEvent &original, std::shared_ptr<TEvent> &cloned) {
                eventCloner(original, *cloned);
            };
            _eventClonerSharedPtr = [eventCloner](std::shared_ptr<const TEvent> &original,
                                                  std::shared_ptr<TEvent> &cloned) {
                eventCloner(*original, *cloned);
            };
        } else {
            _eventCloner = [](const TEvent &original, std::shared_ptr<TEvent> &cloned) {
                cloned = std::make_shared<TEvent>(original);
            };
            _eventClonerSharedPtr = [](std::shared_ptr<const TEvent> &original,
                                       std::shared_ptr<TEvent> &cloned) { *cloned = *original; };
        }

        if (container) {
            _updateTime = TimeUtils::getCurrentNanosecondsAsLlu;
        } else {
            _updateTime = []() { return 0llu; };
        }
    }

    /**
     * @brief internalPublish
     * @param event
     */
    void internalPublish(const TEvent &event)
    {
        try {
            int64_t seq = _ringBuffer.try_next();
            _eventCloner(event, _ringBuffer[seq].eventData);
            _ringBuffer[seq].enqueuedTimeInNs = _updateTime();
            _ringBuffer.publish(seq);
        } catch (disruptor4cpp::insufficient_capacity_exception &ice) {
            spdlog::info(
                "DataMultiplexerPublisher::internalPublish. no more capacity. Publisher name: {}",
                this->name);
            this->publicationStats.totalDiscardedEvents++;
        }
    }

    /**
     * @brief internalPublish
     * @param event
     */
    void internalPublish(std::shared_ptr<const TEvent> event)
    {
        try {
            int64_t seq = _ringBuffer.try_next();
            _eventClonerSharedPtr(event, _ringBuffer[seq].eventData);
            _ringBuffer[seq].enqueuedTimeInNs = _updateTime();
            _ringBuffer.publish(seq);
        } catch (disruptor4cpp::insufficient_capacity_exception &ice) {
            spdlog::info("DataMultiplexerPublisher::internalPublishSharedPtr. no more capacity. "
                         "Publisher name: {}, current discared: {}",
                         this->name,
                         this->publicationStats.totalDiscardedEvents);
            this->publicationStats.totalDiscardedEvents++;
        }
    }

    /**
     * @brief processAndPublish
     * @param process
     */
    void processAndPublish(std::function<void(TEvent &)> process)
    {
        try {
            int64_t seq = _ringBuffer.try_next();
            process(*_ringBuffer[seq].eventData);
            _ringBuffer[seq].enqueuedTimeInNs = _updateTime();
            _ringBuffer.publish(seq);
        } catch (disruptor4cpp::insufficient_capacity_exception &ice) {
            spdlog::info(
                "DataMultiplexerPublisher::processAndPublish. no more capacity. Publisher name: {}",
                this->name);
            this->publicationStats.totalDiscardedEvents++;
        }
    }

private:
    RingBuffer &_ringBuffer;
    std::function<void(const TEvent &, std::shared_ptr<TEvent> &)> _eventCloner;
    std::function<void(std::shared_ptr<const TEvent> &, std::shared_ptr<TEvent> &)>
        _eventClonerSharedPtr;
    std::function<long long unsigned int(void)> _updateTime;
};
} // namespace high_performance
} // namespace kpsr

#endif // DATA_MULTIPLEXER_PUBLISHER_H
