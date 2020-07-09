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

#ifndef DATA_MULTIPLEXER_LISTENER_H
#define DATA_MULTIPLEXER_LISTENER_H

#include <functional>
#include <memory>
#include <future>

#include <klepsydra/core/subscription_stats.h>

#include <klepsydra/high_performance/data_multiplexer_event_handler.h>
#include <klepsydra/high_performance/data_multiplexer_event_data.h>

namespace kpsr
{
namespace high_performance
{

static const long MULTIPLEXER_START_TIMEOUT_MILLISEC = 100;

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
                            const std::shared_ptr<SubscriptionStats>& listenerStat,
                            long timeoutMS = MULTIPLEXER_START_TIMEOUT_MILLISEC)
        : _ringBuffer(ringBuffer)
        , _eventHandler(listener, listenerStat)
        , _name(listenerStat->_name)
        , _started(false)
        , _batchProcessorTask([this] {
                                  std::vector<disruptor4cpp::sequence * > sequences_to_add;
                                  sequences_to_add.resize(1);
                                  sequences_to_add[0] = &batchEventProcessor.get()->get_sequence();
                                  this->_ringBuffer.add_gating_sequences(sequences_to_add);
                                  this->batchEventProcessor->run();
                              })
        , _batchProcessorThread()
        , _batchProcessorThreadFuture(_batchProcessorTask.get_future())
        , _timeoutUs(timeoutMS*1000) {
        auto barrier = _ringBuffer.new_barrier();
        batchEventProcessor = std::unique_ptr<BatchProcessor>(new BatchProcessor(_ringBuffer, std::move(barrier), _eventHandler));
    }

    void start() {
        if (_started) {
            return;
        }
        _batchProcessorThread = std::thread(std::move(_batchProcessorTask));
        _started = true;
        long counterUs = 0;
        while(!this->batchEventProcessor->is_running()) {
            if (counterUs > _timeoutUs) {
                throw std::runtime_error("Could not start the DataMultiplexerListener");
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            counterUs += 100;
        }
    }

    void stop() {
        if (!_started) {
            return;
        }
        batchEventProcessor->halt();
        if (_batchProcessorThread.joinable())
        {
            _batchProcessorThread.join();
        }
    }

    std::unique_ptr<BatchProcessor> batchEventProcessor;

private:
    RingBuffer & _ringBuffer;
    DataMultiplexerEventHandler<TEvent> _eventHandler;
    std::string _name;
    std::atomic<bool> _started;
    std::packaged_task<void()> _batchProcessorTask;
    std::thread _batchProcessorThread;
    std::future<void> _batchProcessorThreadFuture;
    long _timeoutUs;
};
}
}

#endif // DATA_MULTIPLEXER_LISTENER_H
