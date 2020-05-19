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

#ifndef EVENT_LOOP_PUBLISHER_H
#define EVENT_LOOP_PUBLISHER_H

#include <functional>
#include <memory>
#include <tuple>

#include <spdlog/spdlog.h>


#include <klepsydra/core/object_pool_publisher.h>

#include <klepsydra/high_performance/disruptor4cpp/disruptor4cpp.h>

#include <klepsydra/high_performance/event_loop_event_handler.h>

namespace kpsr
{
namespace high_performance
{
template <typename T, std::size_t BufferSize>
/**
 * @brief The EventLoopPublisher class
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
class EventLoopPublisher : public ObjectPoolPublisher<T> {
public:
    using RingBuffer = disruptor4cpp::ring_buffer<EventloopDataWrapper, BufferSize, disruptor4cpp::blocking_wait_strategy, disruptor4cpp::producer_type::multi, disruptor4cpp::sequence>;

    /**
     * @brief EventLoopPublisher
     * @param container
     * @param ringBuffer
     * @param eventName
     * @param poolSize
     * @param initializerFunction
     * @param eventCloner
     */
    EventLoopPublisher(Container * container,
                       RingBuffer & ringBuffer,
                       std::string eventName,
                       int poolSize,
                       std::function<void(T &)> initializerFunction,
                       std::function<void(const T &, T &)> eventCloner)
        : ObjectPoolPublisher<T>(container, eventName, "EVENT_LOOP", poolSize, initializerFunction, eventCloner)
        , _discardedMessages(0)
        , _ringBuffer(ringBuffer)
        , _eventName(eventName)
        , _poolSize(poolSize)
    {
    }

    /**
     * @brief internalPublish
     * @param event
     */
    void internalPublish(std::shared_ptr<const T> event) {
        try
        {
            int64_t seq = _ringBuffer.try_next();
            if (_ringBuffer[seq].eventData) {
                _ringBuffer[seq].eventData.reset();
            }
            _ringBuffer[seq].eventData = std::static_pointer_cast<const void>(event);
            _ringBuffer[seq].eventName = _eventName;
            _ringBuffer[seq].enqueuedTimeInNs = TimeUtils::getCurrentNanosecondsAsLlu();
            _ringBuffer.publish(seq);
        }
        catch (disruptor4cpp::insufficient_capacity_exception& ice) {
            spdlog::info("EventLoopPublisher::internalPublish. no more capacity.{}", _eventName);
            _discardedMessages++;
        }
    }

    /**
     * @brief _discardedMessages message that could not be place in the ring buffer due to reached capacity
     */
    long long unsigned int _discardedMessages;

private:

    RingBuffer & _ringBuffer;
    std::string _eventName;
    int _poolSize;
};
}
}

#endif // EVENT_LOOP_PUBLISHER_H
