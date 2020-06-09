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

#ifndef DATA_MULTIPLEXER_PUBLISHER_H
#define DATA_MULTIPLEXER_PUBLISHER_H

#include <functional>

#include <spdlog/spdlog.h>


#include <klepsydra/core/publisher.h>

#include <klepsydra/high_performance/disruptor4cpp/disruptor4cpp.h>
#include <klepsydra/high_performance/data_multiplexer_event_data.h>

namespace kpsr
{
namespace high_performance
{
template <typename TEvent, std::size_t BufferSize>
/**
 * @brief The DataMultiplexerPublisher class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-high_performance-composition
 *
 * @details This class is not actually used by the client code, but it is documented due to its close relation to the
 * provider. When the method publish is invoked, this class puts a copy of the event into the ringbuffer.
 *
 */
class DataMultiplexerPublisher : public Publisher<TEvent> {
public:
    using RingBuffer = disruptor4cpp::ring_buffer<EventData<TEvent>, BufferSize, disruptor4cpp::blocking_wait_strategy, disruptor4cpp::producer_type::single, disruptor4cpp::sequence>;

    /**
     * @brief DataMultiplexerPublisher
     * @param container
     * @param name
     * @param eventCloner optional function used for cloning event that are put in the ring buffer.
     * @param ringBuffer
     */
    DataMultiplexerPublisher(Container * container, const std::string & name,
                       std::function<void(const TEvent &, TEvent &)> eventCloner,
                       RingBuffer & ringBuffer)
        : Publisher<TEvent>(container, name, "DATA_MULTIPLEXER")
        , _ringBuffer(ringBuffer)
        , _eventCloner(eventCloner)
    {}

    /**
     * @brief internalPublish
     * @param event
     */
    void internalPublish(const TEvent & event) {
        try
        {
            int64_t seq = _ringBuffer.try_next();
            if (_eventCloner == nullptr) {
                _ringBuffer[seq].eventData = event;
            } else {
                _eventCloner(event, _ringBuffer[seq].eventData);
            }
            _ringBuffer[seq].enqueuedTimeInNs = TimeUtils::getCurrentNanosecondsAsLlu();
            _ringBuffer.publish(seq);
        }
        catch (disruptor4cpp::insufficient_capacity_exception& ice) {
            spdlog::info("DataMultiplexerPublisher::internalPublish. no more capacity.");
            this->_publicationStats._totalDiscardedEvents++;
        }
    }

    /**
     * @brief internalPublish
     * @param event
     */
    void internalPublish(std::shared_ptr<const TEvent> event) {
        internalPublish(*event.get());
    }

    /**
     * @brief processAndPublish
     * @param process
     */
    void processAndPublish(std::function<void(TEvent &)> process) {
        try
        {
            int64_t seq = _ringBuffer.try_next();
            process(_ringBuffer[seq].eventData);
            _ringBuffer[seq].enqueuedTimeInNs = TimeUtils::getCurrentNanosecondsAsLlu();
            _ringBuffer.publish(seq);
        }
        catch (disruptor4cpp::insufficient_capacity_exception& ice) {
            spdlog::info("DataMultiplexerPublisher::internalPublish. no more capacity.");
            this->_publicationStats._totalDiscardedEvents++;
        }
    }

private:
    RingBuffer & _ringBuffer;
    std::function<void(const TEvent &, TEvent &)> _eventCloner = nullptr;
};
}
}

#endif // DATA_MULTIPLEXER_PUBLISHER_H
