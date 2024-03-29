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

#ifndef DISRUPTOR4CPP_NO_OP_EVENT_PROCESSOR_H_
#define DISRUPTOR4CPP_NO_OP_EVENT_PROCESSOR_H_

#include <atomic>
#include <cstdint>
#include <memory>
#include <stdexcept>

#include <klepsydra/high_performance/disruptor4cpp/event_handler.h>
#include <klepsydra/high_performance/disruptor4cpp/exceptions/alert_exception.h>
#include <klepsydra/high_performance/disruptor4cpp/exceptions/timeout_exception.h>
#include <klepsydra/high_performance/disruptor4cpp/sequence.h>

namespace disruptor4cpp {
template<typename TRingBuffer>
class no_op_event_processor
{
public:
    no_op_event_processor(TRingBuffer &ring_buffer,
                          typename TRingBuffer::sequence_barrier_type &sequence_barrier)
        : sequence_()
        , ring_buffer_(ring_buffer)
        , sequence_barrier_(sequence_barrier)
        , running_(false)
    {}

    no_op_event_processor(
        TRingBuffer &ring_buffer,
        std::unique_ptr<typename TRingBuffer::sequence_barrier_type> sequence_barrier_ptr)
        : sequence_()
        , ring_buffer_(ring_buffer)
        , sequence_barrier_(*sequence_barrier_ptr)
        , sequence_barrier_ptr_(std::move(sequence_barrier_ptr))
        , running_(false)
    {}

    typename TRingBuffer::sequence_type &get_sequence() { return sequence_; }

    void halt() { running_.store(false, std::memory_order_release); }

    bool is_running() const { return running_.load(std::memory_order_acquire); }

    void run()
    {
        bool expected_running_state = false;
        if (!running_.compare_exchange_strong(expected_running_state, true))
            throw std::runtime_error("Thread is already running");

        // Implemenent with the same logic in batch event processor without notification.
        // Different from the java version.
        sequence_barrier_.clear_alert();
        int64_t next_sequence = sequence_.get() + 1;
        try {
            while (true) {
                try {
                    const int64_t available_sequence = sequence_barrier_.wait_for(next_sequence);
                    while (next_sequence <= available_sequence) {
                        next_sequence++;
                    }
                    sequence_.set(available_sequence);
                } catch (timeout_exception &timeout_ex) {
                } catch (alert_exception &alert_ex) {
                    if (!running_.load(std::memory_order_acquire))
                        break;
                } catch (std::exception &ex) {
                    sequence_.set(next_sequence);
                    next_sequence++;
                }
            }
        } catch (...) {
            running_.store(false, std::memory_order_release);
            throw;
        }
        running_.store(false, std::memory_order_release);
    }

private:
    typename TRingBuffer::sequence_type sequence_;
    TRingBuffer &ring_buffer_;
    typename TRingBuffer::sequence_barrier_type &sequence_barrier_;
    std::unique_ptr<typename TRingBuffer::sequence_barrier_type> sequence_barrier_ptr_;
    std::atomic<bool> running_;
};
} // namespace disruptor4cpp

#endif
