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

/*
Copyright (c) 2015, Alex Man-fui Lee
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of disruptor4cpp nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef DISRUPTOR4CPP_BATCH_EVENT_PROCESSOR_H_
#define DISRUPTOR4CPP_BATCH_EVENT_PROCESSOR_H_

#include <atomic>
#include <cstdint>
#include <memory>
#include <stdexcept>

#include <klepsydra/high_performance/disruptor4cpp/event_handler.h>
#include <klepsydra/high_performance/disruptor4cpp/exceptions/alert_exception.h>
#include <klepsydra/high_performance/disruptor4cpp/exceptions/timeout_exception.h>
#include <klepsydra/high_performance/disruptor4cpp/sequence.h>

#include <spdlog/spdlog.h>

namespace disruptor4cpp {
template<typename TRingBuffer>
class batch_event_processor
{
public:
    batch_event_processor(TRingBuffer &ring_buffer,
                          typename TRingBuffer::sequence_barrier_type &sequence_barrier,
                          event_handler<typename TRingBuffer::event_type> &evt_handler,
                          std::string name = "")
        : sequence_()
        , ring_buffer_(ring_buffer)
        , sequence_barrier_(sequence_barrier)
        , event_handler_(evt_handler)
        , running_(false)
        , name_(name)
    {}

    batch_event_processor(
        TRingBuffer &ring_buffer,
        std::unique_ptr<typename TRingBuffer::sequence_barrier_type> sequence_barrier_ptr,
        event_handler<typename TRingBuffer::event_type> &evt_handler,
        std::string name = "")
        : sequence_()
        , ring_buffer_(ring_buffer)
        , sequence_barrier_(*sequence_barrier_ptr)
        , event_handler_(evt_handler)
        , sequence_barrier_ptr_(std::move(sequence_barrier_ptr))
        , running_(false)
        , name_(name)
    {}

    typename TRingBuffer::sequence_type &get_sequence() { return sequence_; }

    void halt()
    {
        running_.store(false, std::memory_order_release);
        sequence_barrier_.alert();
    }

    bool is_running() const { return running_.load(std::memory_order_acquire); }

    void run()
    {
        bool expected_running_state = false;
        if (!running_.compare_exchange_strong(expected_running_state, true))
            throw std::runtime_error("Thread is already running");

        sequence_barrier_.clear_alert();
        notify_start();

        typename TRingBuffer::event_type *event = nullptr;
        int64_t next_sequence = sequence_.get() + 1;
        try {
            while (true) {
                try {
                    const int64_t available_sequence = sequence_barrier_.wait_for(next_sequence);
                    while (next_sequence <= available_sequence) {
                        event = &ring_buffer_[next_sequence];
                        event_handler_.on_event(*event,
                                                next_sequence,
                                                next_sequence == available_sequence);
                        next_sequence++;
                    }
                    sequence_.set(available_sequence);
                } catch (timeout_exception &timeout_ex) {
                    spdlog::info(
                        "kpsr::high_perf::batch_event_processor. timeout_exception {} on {}.",
                        timeout_ex.what(),
                        name_);
                    notify_timeout(sequence_.get());
                } catch (alert_exception &alert_ex) {
                    if (!running_.load(std::memory_order_acquire)) {
                        spdlog::debug("kpsr::high_perf::batch_event_processor. alert_ex {} on {}.",
                                      alert_ex.what(),
                                      name_);
                        break;
                    }
                } catch (std::exception &ex) {
                    spdlog::info("kpsr::high_perf::batch_event_processor. exception {} on {}.",
                                 ex.what(),
                                 name_);
                    event_handler_.on_event_exception(ex, next_sequence, event);
                    sequence_.set(next_sequence);
                    next_sequence++;
                }
            }
        } catch (...) {
            spdlog::debug("kpsr::high_perf::batch_event_processor. notify_shutdown on {}.", name_);
            notify_shutdown();
            running_.store(false, std::memory_order_release);
            throw;
        }
        notify_shutdown();
        running_.store(false, std::memory_order_release);
    }

private:
    void notify_timeout(int64_t available_sequence)
    {
        try {
            event_handler_.on_timeout(available_sequence);
        } catch (std::exception &ex) {
            event_handler_.on_event_exception(ex, available_sequence, nullptr);
        }
    }

    void notify_start()
    {
        try {
            event_handler_.on_start();
        } catch (std::exception &ex) {
            event_handler_.on_start_exception(ex);
        }
    }

    void notify_shutdown()
    {
        try {
            event_handler_.on_shutdown();
        } catch (std::exception &ex) {
            event_handler_.on_shutdown_exception(ex);
        }
    }

    typename TRingBuffer::sequence_type sequence_;
    TRingBuffer &ring_buffer_;
    typename TRingBuffer::sequence_barrier_type &sequence_barrier_;
    event_handler<typename TRingBuffer::event_type> &event_handler_;
    std::unique_ptr<typename TRingBuffer::sequence_barrier_type> sequence_barrier_ptr_;
    std::atomic<bool> running_;
    std::string name_;
};
} // namespace disruptor4cpp

#endif
