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

#ifndef EVENT_LOOP_PUBLISHER_H
#define EVENT_LOOP_PUBLISHER_H

#include <functional>
#include <memory>
#include <tuple>

#include <spdlog/spdlog.h>

#include <klepsydra/core/object_pool_publisher.h>

#include <klepsydra/high_performance/disruptor4cpp/disruptor4cpp.h>

#include <klepsydra/high_performance/event_loop_event_handler.h>

namespace kpsr {
namespace high_performance {
template<typename T, std::size_t BufferSize>
/**
 * @brief The EventLoopPublisher class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-eventloop-composition
 *
 * @details Although this function is not really used by the client code directly, it is documented due to its close
 * relation with the event loop. The EventLoopPublisher extends from the ObjectPoolPublisher and therefore can handle
 * pools of object in order to improve performance. The tuning can be done via the provider.
 *
 * The publisher also adds a timestamp to the event when placed in the rinbuffer, so the performance monitor can
 * check how long the message was enqueued before proceed.
 *
 */
class EventLoopPublisher : public ObjectPoolPublisher<T>
{
public:
    using RingBuffer = disruptor4cpp::ring_buffer<EventloopDataWrapper,
                                                  BufferSize,
                                                  disruptor4cpp::blocking_wait_strategy,
                                                  disruptor4cpp::producer_type::multi,
                                                  disruptor4cpp::sequence>;

    /**
     * @brief EventLoopPublisher
     * @param container
     * @param ringBuffer
     * @param eventName
     * @param poolSize
     * @param initializerFunction
     * @param eventCloner
     */
    EventLoopPublisher(Container *container,
                       RingBuffer &ringBuffer,
                       std::string eventLoopName,
                       std::string eventName,
                       int poolSize,
                       std::function<void(T &)> initializerFunction,
                       std::function<void(const T &, T &)> eventCloner,
                       bool enableUnsafeMode)
        : ObjectPoolPublisher<T>(container,
                                 eventLoopName + "_" + eventName,
                                 "EVENT_LOOP",
                                 poolSize,
                                 initializerFunction,
                                 eventCloner)
        , _discardedMessages(0)
        , _ringBuffer(ringBuffer)
        , _eventName(eventName)
        , _poolSize(poolSize)
    {
        if (container) {
            _updateTime = TimeUtils::getCurrentNanosecondsAsLlu;
        } else {
            _updateTime = []() { return 0llu; };
        }
        if (enableUnsafeMode) {
            _prePublishingStep = [this](int64_t seq) {
                this->_ringBuffer[seq].enqueuedTimeInNs = this->_updateTime();
            };
        } else {
            _prePublishingStep = [this](int64_t seq) {
                if (this->_ringBuffer[seq].eventData) {
                    spdlog::warn("Unexpected non-nullptr pointer in ringbuffer with address: {}. "
                                 "For index: {}",
                                 this->_ringBuffer[seq].eventData.get(),
                                 seq);
                    this->_ringBuffer[seq].eventData.reset();
                }
                this->_ringBuffer[seq].enqueuedTimeInNs = this->_updateTime();
            };
        }
    }

    /**
     * @brief internalPublish
     * @param event
     */
    void internalPublish(std::shared_ptr<const T> event)
    {
        try {
            int64_t seq = _ringBuffer.try_next();
            _prePublishingStep(seq);
            _ringBuffer[seq].eventData = std::static_pointer_cast<const void>(event);
            _ringBuffer[seq].eventName = _eventName;
            _ringBuffer.publish(seq);
        } catch (disruptor4cpp::insufficient_capacity_exception &ice) {
            spdlog::info("EventLoopPublisher::internalPublish. no more capacity.{}", _eventName);
            _discardedMessages++;
        }
    }

    /**
     * @brief _discardedMessages message that could not be place in the ring buffer due to reached capacity
     */
    long long unsigned int _discardedMessages;

private:
    RingBuffer &_ringBuffer;
    std::string _eventName;
    int _poolSize;
    std::function<void(int64_t seq)> _prePublishingStep;
    std::function<long long unsigned int(void)> _updateTime;
};
} // namespace high_performance
} // namespace kpsr

#endif // EVENT_LOOP_PUBLISHER_H
