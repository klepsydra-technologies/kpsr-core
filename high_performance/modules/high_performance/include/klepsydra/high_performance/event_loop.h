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

#ifndef EVENT_LOOP_H
#define EVENT_LOOP_H

#include <mutex>
#include <chrono>
#include <map>
#include <memory>
#include <iostream>
#include <atomic>
#include <future>

#include <klepsydra/core/event_emitter_subscriber.h>

#include <klepsydra/high_performance/disruptor4cpp/disruptor4cpp.h>
#include <klepsydra/high_performance/event_loop_event_handler.h>

namespace kpsr
{
namespace high_performance
{

static const long EVENT_LOOP_START_TIMEOUT_MILLISEC = 100;
static const char * EVENT_LOOP_START_MESSAGE = "About to run batchEventProcessor";
    
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
    EventLoop(kpsr::EventEmitter & eventEmitter, RingBuffer & ringBuffer, const std::string & name,
              long timeoutMS = EVENT_LOOP_START_TIMEOUT_MILLISEC)
        : _name(name)
        , _threadName(std::to_string(BufferSize) + "_" + name)
        , _ringBuffer(ringBuffer)
        , _eventHandler(eventEmitter)
        , _isStarted(false)
        , _eventLoopTask([this] {
                                std::vector<disruptor4cpp::sequence * > sequences_to_add;
                                sequences_to_add.resize(1);
                                sequences_to_add[0] = &batchEventProcessor->get_sequence();
                                this->_ringBuffer.add_gating_sequences(sequences_to_add);
                                spdlog::info(EVENT_LOOP_START_MESSAGE);
                                this->batchEventProcessor->run();
                            })
        , _batchProcessTask(_eventLoopTask)
        , _batchProcessorThreadFuture(_batchProcessTask.get_future())
        , _timeoutUs(timeoutMS*1000)
    {
        auto barrier = _ringBuffer.new_barrier();
        batchEventProcessor = std::unique_ptr<BatchProcessor>(new BatchProcessor(_ringBuffer, std::move(barrier), _eventHandler));
    }

    /**
     * @brief start start consumer thread
     */
    void start() {
        if (isStarted()) {
            return;
        }
        _isStarted.store(true, std::memory_order_release);
        batchProcessorThread = std::thread(std::move(_batchProcessTask));
        long counterUs = 0;
        while(!this->batchEventProcessor->is_running()) {
            if (counterUs > _timeoutUs) {
                throw std::runtime_error("Could not start the event loop");
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            counterUs += 100;
        }
    }

    /**
     * @brief stop stop consumer thread.
     */
    void stop() {
        if (!isStarted()) {
            return;
        }
        _isStarted.store(false, std::memory_order_release);
        this->batchEventProcessor->halt();
        spdlog::info("Halting the batchEventProcessor");
        if (this->batchProcessorThread.joinable())
        {
            this->batchProcessorThread.join();
        }
        // make _batchProcessTask reusable
        _batchProcessTask = std::packaged_task<void()>(_eventLoopTask);
        _batchProcessorThreadFuture = _batchProcessTask.get_future();
    }

    bool isStarted() {
        return _isStarted.load(std::memory_order_acquire);
    }

    bool isRunning() const {
        return this->batchEventProcessor->is_running();
    }
private:
    std::string _name;
    std::string _threadName;
    RingBuffer & _ringBuffer;
    EventLoopEventHandler _eventHandler;
    std::unique_ptr<BatchProcessor> batchEventProcessor;
    std::thread batchProcessorThread;
    std::atomic_bool _isStarted;
    std::function<void()> _eventLoopTask;
    std::packaged_task<void()> _batchProcessTask;
    std::future<void> _batchProcessorThreadFuture;
    long _timeoutUs;
};
}
}

#endif // EVENT_LOOP_H
