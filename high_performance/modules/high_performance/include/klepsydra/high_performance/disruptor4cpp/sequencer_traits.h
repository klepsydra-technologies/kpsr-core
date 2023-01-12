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

#ifndef DISRUPTOR4CPP_SEQUENCER_TRAITS_H_
#define DISRUPTOR4CPP_SEQUENCER_TRAITS_H_

#include <cstddef>

#include <klepsydra/high_performance/disruptor4cpp/multi_producer_sequencer.h>
#include <klepsydra/high_performance/disruptor4cpp/producer_type.h>
#include <klepsydra/high_performance/disruptor4cpp/sequence_barrier.h>
#include <klepsydra/high_performance/disruptor4cpp/single_producer_sequencer.h>

namespace disruptor4cpp {
template<std::size_t BufferSize, typename TWaitStrategy, typename TSequence, producer_type ProducerType>
struct sequencer_traits;

template<std::size_t BufferSize, typename TWaitStrategy, typename TSequence>
struct sequencer_traits<BufferSize, TWaitStrategy, TSequence, producer_type::single>
{
    typedef TWaitStrategy wait_strategy_type;
    typedef TSequence sequence_type;
    typedef single_producer_sequencer<BufferSize, TWaitStrategy, TSequence> sequencer_type;
    typedef sequence_barrier<sequencer_type> sequence_barrier_type;

    static constexpr std::size_t BUFFER_SIZE = BufferSize;
    static constexpr producer_type PRODUCER_TYPE = producer_type::single;
};

template<std::size_t BufferSize, typename TWaitStrategy, typename TSequence>
struct sequencer_traits<BufferSize, TWaitStrategy, TSequence, producer_type::multi>
{
    typedef TWaitStrategy wait_strategy_type;
    typedef TSequence sequence_type;
    typedef multi_producer_sequencer<BufferSize, TWaitStrategy, TSequence> sequencer_type;
    typedef sequence_barrier<sequencer_type> sequence_barrier_type;

    static constexpr std::size_t BUFFER_SIZE = BufferSize;
    static constexpr producer_type PRODUCER_TYPE = producer_type::multi;
};
} // namespace disruptor4cpp

#endif
