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

#ifndef DISRUPTOR4CPP_TEST_DUMMY_SEQUENCE_BARRIER_H_
#define DISRUPTOR4CPP_TEST_DUMMY_SEQUENCE_BARRIER_H_

#include <cstdint>

namespace disruptor4cpp {
namespace test {
class dummy_sequence_barrier
{
public:
    dummy_sequence_barrier() = default;
    ~dummy_sequence_barrier() = default;

    int64_t wait_for(int64_t seq) { return 0; }

    int64_t get_cursor() const { return 0; }

    bool is_alerted() const { return false; }

    void alert() {}

    void clear_alert() {}

    void check_alert() const {}

private:
    dummy_sequence_barrier(const dummy_sequence_barrier &) = delete;
    dummy_sequence_barrier &operator=(const dummy_sequence_barrier &) = delete;
    dummy_sequence_barrier(dummy_sequence_barrier &&) = delete;
    dummy_sequence_barrier &operator=(dummy_sequence_barrier &&) = delete;
};
} // namespace test
} // namespace disruptor4cpp

#endif
