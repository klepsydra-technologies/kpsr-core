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

#ifndef DISRUPTOR4CPP_RING_BUFFER_H_
#define DISRUPTOR4CPP_RING_BUFFER_H_

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <functional>

#include <klepsydra/high_performance/disruptor4cpp/producer_type.h>
#include <klepsydra/high_performance/disruptor4cpp/sequencer_traits.h>
#include <klepsydra/high_performance/disruptor4cpp/utils/cache_line_storage.h>

namespace disruptor4cpp
{
template <typename TEvent, std::size_t BufferSize,
          typename TWaitStrategy, producer_type ProducerType, typename TSequence = sequence>
class ring_buffer :
        public sequencer_traits<BufferSize, TWaitStrategy, TSequence, ProducerType>::sequencer_type
{
public:
    static_assert(std::is_default_constructible<TEvent>::value, "Event type must be default constructible");
    static_assert((BufferSize & (~BufferSize + 1)) == BufferSize, "Ring buffer size must be a power of 2");

    typedef TEvent event_type;
    typedef TWaitStrategy wait_strategy_type;
    typedef TSequence sequence_type;
    typedef typename sequencer_traits<BufferSize, TWaitStrategy, TSequence, ProducerType>::sequence_barrier_type sequence_barrier_type;

    static constexpr std::size_t BUFFER_SIZE = BufferSize;
    static constexpr producer_type PRODUCER_TYPE = ProducerType;

    ring_buffer() = default;

    explicit ring_buffer(const TEvent * value)
    {
        for (int i = 0; i < static_cast<int>(BufferSize); i++)
        {
            if (value) {
                events_[i].data = *value;
            }
        }
    }

    explicit ring_buffer(std::function<void(TEvent &)> initializer)
    {
        for (int i = 0; i < static_cast<int>(BufferSize); i++)
        {
            initializer(events_[i].data);
        }
    }

    ~ring_buffer() = default;

    TEvent& operator[](int64_t seq)
    {
        return events_[seq & (static_cast<int64_t>(BufferSize) - 1)].data;
    }

    const TEvent& operator[](int64_t seq) const
    {
        return events_[seq & (static_cast<int64_t>(BufferSize) - 1)].data;
    }

private:
    ring_buffer(const ring_buffer&) = delete;
    ring_buffer& operator=(const ring_buffer&) = delete;
    ring_buffer(ring_buffer&&) = delete;
    ring_buffer& operator=(ring_buffer&&) = delete;

    cache_line_storage<TEvent> events_[BufferSize];
};
}

#endif
