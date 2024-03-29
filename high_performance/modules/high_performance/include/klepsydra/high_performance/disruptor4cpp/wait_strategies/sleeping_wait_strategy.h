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

#ifndef DISRUPTOR4CPP_WAIT_STRATEGIES_SLEEPING_WAIT_STRATEGY_H_
#define DISRUPTOR4CPP_WAIT_STRATEGIES_SLEEPING_WAIT_STRATEGY_H_

#include <chrono>
#include <cstdint>
#include <thread>

#include <klepsydra/high_performance/disruptor4cpp/fixed_sequence_group.h>

namespace disruptor4cpp {
template<int Retries = 200>
class sleeping_wait_strategy
{
public:
    sleeping_wait_strategy() = default;
    ~sleeping_wait_strategy() = default;

    template<typename TSequenceBarrier, typename TSequence>
    int64_t wait_for(int64_t seq,
                     const TSequence &cursor_sequence,
                     const fixed_sequence_group<TSequence> &dependent_sequence,
                     const TSequenceBarrier &seq_barrier)
    {
        int64_t available_sequence = 0;
        int counter = Retries;

        while ((available_sequence = dependent_sequence.get()) < seq) {
            counter = apply_wait_method(seq_barrier, counter);
        }
        return available_sequence;
    }

    void signal_all_when_blocking() {}

private:
    sleeping_wait_strategy(const sleeping_wait_strategy &) = delete;
    sleeping_wait_strategy &operator=(const sleeping_wait_strategy &) = delete;
    sleeping_wait_strategy(sleeping_wait_strategy &&) = delete;
    sleeping_wait_strategy &operator=(sleeping_wait_strategy &&) = delete;

    template<typename TSequenceBarrier>
    int apply_wait_method(const TSequenceBarrier &seq_barrier, int counter)
    {
        seq_barrier.check_alert();
        if (counter > 100)
            --counter;
        else if (counter > 0) {
            --counter;
            std::this_thread::yield();
        } else
            std::this_thread::sleep_for(std::chrono::nanoseconds(1));
        return counter;
    }
};
} // namespace disruptor4cpp

#endif
