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

#ifndef EVENT_LOOP_H
#define EVENT_LOOP_H

#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>

#include <klepsydra/core/event_emitter_subscriber.h>

#include <klepsydra/high_performance/disruptor4cpp/disruptor4cpp.h>
#include <klepsydra/high_performance/event_loop_event_handler.h>

namespace kpsr {
namespace high_performance {

static const long EVENT_LOOP_START_TIMEOUT_MICROSEC = 100 * 1000;
static const char *EVENT_LOOP_START_MESSAGE = "About to run batchEventProcessor";

template<std::size_t BufferSize>
/**
 * @brief The EventLoop class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-eventloop-composition
 *
 * @details This class is not really used by the client code, but it is documented due it its importance in the
 * event loop implementation in Klepsydra. This class basically combines the event emitter with the high_performance pattern
 * into an event loop with similar behaviour as other event loops implementations.
 * It is as single consumer and multiple publisher ring buffer wrapper with an event emitter API.
 *
 */
class EventLoop
{
public:
    using RingBuffer = disruptor4cpp::ring_buffer<EventloopDataWrapper,
                                                  BufferSize,
                                                  disruptor4cpp::blocking_wait_strategy,
                                                  disruptor4cpp::producer_type::multi,
                                                  disruptor4cpp::sequence>;
    using BatchProcessor = disruptor4cpp::batch_event_processor<RingBuffer>;

    /**
     * @brief EventLoop
     * @param eventEmitter
     * @param ringBuffer
     */
    EventLoop(std::shared_ptr<EventEmitterInterface<EventloopDataWrapper>> &externalEventEmitter,
              RingBuffer &ringBuffer,
              const std::string &name,
              const long timeoutUS = EVENT_LOOP_START_TIMEOUT_MICROSEC)
        : _name(name)
        , _threadName(std::to_string(BufferSize) + "_" + name)
        , _ringBuffer(ringBuffer)
        , _eventHandler(externalEventEmitter)
        , _isStarted(false)
        , _eventLoopTask([this] {
            spdlog::trace("kpsr::high_perf::EventLoop::_eventLoopTask begin");
            std::vector<disruptor4cpp::sequence *> sequences_to_add;
            sequences_to_add.resize(1);
            sequences_to_add[0] = &batchEventProcessor->get_sequence();
            this->_ringBuffer.add_gating_sequences(sequences_to_add);
            spdlog::debug(EVENT_LOOP_START_MESSAGE);
            this->batchEventProcessor->run();
            spdlog::trace("kpsr::high_perf::EventLoop::_eventLoopTask end.");
        })
        , _batchProcessTask(_eventLoopTask)
        , _batchProcessorThreadFuture(_batchProcessTask.get_future())
        , _timeoutUs(timeoutUS)
    {
        auto barrier = _ringBuffer.new_barrier();
        batchEventProcessor = std::unique_ptr<BatchProcessor>(
            new BatchProcessor(_ringBuffer, std::move(barrier), _eventHandler, name));
    }

    /**
     * @brief start start consumer thread
     */
    void start()
    {
        if (isStarted()) {
            return;
        }
        batchProcessorThread = std::thread(std::move(_batchProcessTask));
        spdlog::debug(
            "kpsr::high_perf::EventLoop::start. Starting the event loop here with threadname {}",
            _threadName);
        _isStarted.store(true, std::memory_order_release);
        long counterUs = 0;
        while (!this->batchEventProcessor->is_running()) {
            if (counterUs > _timeoutUs) {
                throw std::runtime_error("Could not start the event loop");
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            counterUs += 100;
            spdlog::trace("kpsr::high_perf::EventLoop::start. Waiting... {}", counterUs);
        }
    }

    /**
     * @brief stop stop consumer thread.
     */
    void stop()
    {
        if (!isStarted()) {
            return;
        }
        _isStarted.store(false, std::memory_order_release);
        this->batchEventProcessor->halt();
        spdlog::debug("kpsr::high_perf::EventLoop::stop. Halting the batchEventProcessor");
        if (this->batchProcessorThread.joinable()) {
            this->batchProcessorThread.join();
        }
        spdlog::debug("kpsr::high_perf::EventLoop::stop. Eventloop {} stopped.", this->_name);
        // make _batchProcessTask reusable
        _batchProcessTask = std::packaged_task<void()>(_eventLoopTask);
        _batchProcessorThreadFuture = _batchProcessTask.get_future();
    }

    bool isStarted() { return _isStarted.load(std::memory_order_acquire); }

    bool isRunning() const { return this->batchEventProcessor->is_running(); }

    std::string getName() { return _name; }

private:
    std::string _name;
    std::string _threadName;
    RingBuffer &_ringBuffer;
    EventLoopEventHandler _eventHandler;
    std::unique_ptr<BatchProcessor> batchEventProcessor;
    std::atomic_bool _isStarted;
    std::function<void()> _eventLoopTask;
    std::packaged_task<void()> _batchProcessTask;
    std::thread batchProcessorThread;
    std::future<void> _batchProcessorThreadFuture;
    const long _timeoutUs;
};
} // namespace high_performance
} // namespace kpsr

#endif // EVENT_LOOP_H
