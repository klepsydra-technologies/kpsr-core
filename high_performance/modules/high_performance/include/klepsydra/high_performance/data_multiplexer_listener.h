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

#ifndef DATA_MULTIPLEXER_LISTENER_H
#define DATA_MULTIPLEXER_LISTENER_H

#include <functional>
#include <memory>

#include <klepsydra/core/subscription_stats.h>

#include <klepsydra/high_performance/data_multiplexer_event_handler.h>
#include <klepsydra/high_performance/data_multiplexer_event_data.h>

namespace kpsr
{
namespace high_performance
{
template <typename TEvent, std::size_t BufferSize>
/**
 * @brief The DataMultiplexerListener class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-high_performance-internal
 *
 * @details DataMultiplexer batch processor wrapper and thread creator.
 */
class DataMultiplexerListener
{
public:
    using RingBuffer = disruptor4cpp::ring_buffer<EventData<TEvent>, BufferSize, disruptor4cpp::blocking_wait_strategy, disruptor4cpp::producer_type::single, disruptor4cpp::sequence>;
    using BatchProcessor = disruptor4cpp::batch_event_processor<RingBuffer>;

    DataMultiplexerListener(const std::function<void(TEvent)> & listener,
                      RingBuffer & ringBuffer,
                      std::shared_ptr<SubscriptionStats> listenerStat)
        : _ringBuffer(ringBuffer)
        , _eventHandler(listener, listenerStat){

        auto barrier = _ringBuffer.new_barrier();
        batchEventProcessor = std::unique_ptr<BatchProcessor>(new BatchProcessor(_ringBuffer, std::move(barrier), _eventHandler));
    }

    void start() {
        batchProcessorThread = std::thread([this] {
            std::vector<disruptor4cpp::sequence * > sequences_to_add;
            sequences_to_add.resize(1);
            sequences_to_add[0] = &batchEventProcessor.get()->get_sequence();
            this->_ringBuffer.add_gating_sequences(sequences_to_add);
            this->batchEventProcessor->run();
        });
    }

    std::unique_ptr<
    BatchProcessor> batchEventProcessor;
    std::thread batchProcessorThread;

private:
    RingBuffer & _ringBuffer;
    DataMultiplexerEventHandler<TEvent> _eventHandler;
};
}
}

#endif // DATA_MULTIPLEXER_LISTENER_H
