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

#ifndef DATA_MULTIPLEXER_SUBSCRIBER_H
#define DATA_MULTIPLEXER_SUBSCRIBER_H

#include <functional>
#include <future>
#include <memory>

#include <spdlog/spdlog.h>

#include <klepsydra/core/event_emitter_subscriber.h>

#include <klepsydra/high_performance/data_multiplexer_event_data.h>
#include <klepsydra/high_performance/data_multiplexer_event_handler.h>
#include <klepsydra/high_performance/disruptor4cpp/disruptor4cpp.h>

namespace kpsr {
namespace high_performance {
static const long MULTIPLEXER_START_TIMEOUT_MICROSEC = 100 * 1000;
template<typename TEvent, std::size_t BufferSize>

/**
 * @brief The DataMultiplexerSubscriber class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-high_performance-composition
 *
 * @details This class is not actually used by the client code, but it is documented due to its close relation to the
 * provider. This class has the same API as most subscribers, but it is not based in the event emitter as the most
 * of them. Very important to notice is that in the high_performance, only the last event is invoked. This means that
 * faster listeners will process all messages, while slower listener will only process the later messages while
 * discarding any older ones.
 *
 */
class DataMultiplexerSubscriber : public EventEmitterSubscriber<TEvent>
{
public:
    using RingBuffer = disruptor4cpp::ring_buffer<DataMultiplexerDataWrapper<TEvent>,
                                                  BufferSize,
                                                  disruptor4cpp::blocking_wait_strategy,
                                                  disruptor4cpp::producer_type::single,
                                                  disruptor4cpp::sequence>;
    using BatchProcessor = disruptor4cpp::batch_event_processor<RingBuffer>;

    /**
     * @brief DataMultiplexerSubscriber
     * @param container
     * @param name
     * @param ringBuffer
     * @param cpuAffinityGeneratorFunction function that returns cpu id to use based on listener name
     * @param timoutUS max timeout for listener to start
     */
    DataMultiplexerSubscriber(Container *container,
                              const std::string &dataMuxName,
                              const std::string &subscriberName,
                              RingBuffer &ringBuffer,
                              const long timeoutUS = MULTIPLEXER_START_TIMEOUT_MICROSEC)
        : EventEmitterSubscriber<TEvent>(container,
                                         EventEmitterType::SAFE,
                                         dataMuxName + subscriberName)
        , _ringBuffer(ringBuffer)
        , _eventHandler(this->name, this->_eventEmitter)
        , _started(false)
        , _batchProcessorTask([this] {
            std::vector<disruptor4cpp::sequence *> sequences_to_add;
            sequences_to_add.resize(1);
            sequences_to_add[0] = &batchEventProcessor.get()->get_sequence();
            this->_ringBuffer.add_gating_sequences(sequences_to_add);
            this->batchEventProcessor->run();
        })
        , _batchProcessorThread()
        , _batchProcessorThreadFuture(_batchProcessorTask.get_future())
        , _timeoutUs(timeoutUS)
    {
        auto barrier = _ringBuffer.new_barrier();
        batchEventProcessor = std::unique_ptr<BatchProcessor>(
            new BatchProcessor(_ringBuffer, std::move(barrier), _eventHandler, this->name));

        _batchProcessorThread = std::thread(std::move(_batchProcessorTask));
        _started = true;
        spdlog::debug("{}. DataMultiplexerSubscriber with name {} has been started.",
                      __PRETTY_FUNCTION__,
                      this->name);
        long counterUs = 0;
        while (!this->batchEventProcessor->is_running()) {
            if (counterUs > _timeoutUs) {
                throw std::runtime_error("Could not start the DataMultiplexerListener");
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            counterUs += 100;
        }
    }

    virtual ~DataMultiplexerSubscriber()
    {
        // TODO Move this to stop method and/or ensure all listeners are removed.
        if (!_started) {
            return;
        }
        while (!batchEventProcessor->is_running()) {
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
        batchEventProcessor->halt();
        if (_batchProcessorThread.joinable()) {
            _batchProcessorThread.join();
        }
    }

    std::unique_ptr<BatchProcessor> batchEventProcessor;

private:
    RingBuffer &_ringBuffer;
    DataMultiplexerEventHandler<TEvent> _eventHandler;
    std::atomic<bool> _started;
    std::packaged_task<void()> _batchProcessorTask;
    std::thread _batchProcessorThread;
    std::future<void> _batchProcessorThreadFuture;
    const long _timeoutUs;
};
} // namespace high_performance
} // namespace kpsr

#endif // DATA_MULTIPLEXER_SUBSCRIBER_H
