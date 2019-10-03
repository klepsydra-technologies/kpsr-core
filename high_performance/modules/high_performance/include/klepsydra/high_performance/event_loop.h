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

#ifndef EVENT_LOOP_H
#define EVENT_LOOP_H

#include <mutex>
#include <chrono>
#include <map>
#include <memory>
#include <iostream>
#include <atomic>

#include <klepsydra/core/event_emitter_subscriber.h>

#include <klepsydra/high_performance/disruptor4cpp/disruptor4cpp.h>
#include <klepsydra/high_performance/event_loop_event_handler.h>

namespace kpsr
{
namespace high_performance
{
template <std::size_t BufferSize>
/**
 * @brief The EventLoop class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-eventloop-composition
 *
 * @details This class is not really used by the client code, but it is documented due it its importance in the
 * event loop implementation in Klepsydra. This class basically combines the event emmiter with the high_performance pattern
 * into an event loop with similar behaviour as other event loops implementations.
 * It is as single consumer and multiple publisher ring buffer wrapper with an event emitter API.
 *
 */
class EventLoop {
public:
    using RingBuffer = disruptor4cpp::ring_buffer<EventloopDataWrapper, BufferSize, disruptor4cpp::blocking_wait_strategy, disruptor4cpp::producer_type::multi, disruptor4cpp::sequence>;
    using BatchProcessor = disruptor4cpp::batch_event_processor<RingBuffer>;

    /**
     * @brief EventLoop
     * @param eventEmitter
     * @param ringBuffer
     */
    EventLoop(kpsr::EventEmitter & eventEmitter, RingBuffer & ringBuffer)
        : _ringBuffer(ringBuffer)
        , _eventHandler(eventEmitter)
        , _isStarted(false)
    {
        auto barrier = _ringBuffer.new_barrier();
        batchEventProcessor = std::unique_ptr<BatchProcessor>(new BatchProcessor(_ringBuffer, std::move(barrier), _eventHandler));
    }

    /**
     * @brief start start consumer thread
     */
    void start() {
        if (_isStarted) {
            return;
        }
        batchProcessorThread = std::thread([this] {
            std::vector<disruptor4cpp::sequence * > sequences_to_add;
            sequences_to_add.resize(1);
            sequences_to_add[0] = &batchEventProcessor.get()->get_sequence();
            this->_ringBuffer.add_gating_sequences(sequences_to_add);
            this->batchEventProcessor->run();
        });
        _isStarted = true;
    }

    /**
     * @brief stop stop consumer thread.
     */
    void stop() {
        if (!_isStarted) {
            return;
        }
        this->batchEventProcessor->halt();
        _isStarted = false;
        if (this->batchProcessorThread.joinable())
        {
            this->batchProcessorThread.join();
        }
    }

    bool isStarted() {
        return _isStarted;
    }

private:
    RingBuffer & _ringBuffer;
    EventLoopEventHandler _eventHandler;
    std::unique_ptr<BatchProcessor> batchEventProcessor;
    std::thread batchProcessorThread;
    std::atomic_bool _isStarted;
};
}
}

#endif // EVENT_LOOP_H
