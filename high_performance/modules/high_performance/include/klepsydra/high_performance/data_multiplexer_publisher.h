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

#ifndef DATA_MULTIPLEXER_PUBLISHER_H
#define DATA_MULTIPLEXER_PUBLISHER_H

#include <functional>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

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
    DataMultiplexerPublisher(Container * container, std::string name,
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
