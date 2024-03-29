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

#include <chrono>

#include <gtest/gtest.h>

#include "wait_strategy_test_util.h"
#include <disruptor4cpp/disruptor4cpp.h>

namespace disruptor4cpp {
namespace test {
TEST(phased_backoff_wait_strategy_test, should_handle_immediate_sequence_change)
{
    phased_backoff_wait_strategy<1000000, 1000000, blocking_wait_strategy> wait_strategy_with_lock;
    wait_strategy_test_util::assert_wait_for_with_delay_of(std::chrono::milliseconds(0),
                                                           wait_strategy_with_lock);
    phased_backoff_wait_strategy<1000000, 1000000, sleeping_wait_strategy<0>>
        wait_strategy_with_sleep;
    wait_strategy_test_util::assert_wait_for_with_delay_of(std::chrono::milliseconds(0),
                                                           wait_strategy_with_sleep);
}

TEST(phased_backoff_wait_strategy_test, should_handle_sequence_change_with_one_millisecond_delay)
{
    phased_backoff_wait_strategy<1000000, 1000000, blocking_wait_strategy> wait_strategy_with_lock;
    wait_strategy_test_util::assert_wait_for_with_delay_of(std::chrono::milliseconds(1),
                                                           wait_strategy_with_lock);
    phased_backoff_wait_strategy<1000000, 1000000, sleeping_wait_strategy<0>>
        wait_strategy_with_sleep;
    wait_strategy_test_util::assert_wait_for_with_delay_of(std::chrono::milliseconds(1),
                                                           wait_strategy_with_sleep);
}

TEST(phased_backoff_wait_strategy_test, should_handle_sequence_change_with_two_millisecond_delay)
{
    phased_backoff_wait_strategy<1000000, 1000000, blocking_wait_strategy> wait_strategy_with_lock;
    wait_strategy_test_util::assert_wait_for_with_delay_of(std::chrono::milliseconds(2),
                                                           wait_strategy_with_lock);
    phased_backoff_wait_strategy<1000000, 1000000, sleeping_wait_strategy<0>>
        wait_strategy_with_sleep;
    wait_strategy_test_util::assert_wait_for_with_delay_of(std::chrono::milliseconds(2),
                                                           wait_strategy_with_sleep);
}

TEST(phased_backoff_wait_strategy_test, should_handle_sequence_change_with_ten_millisecond_delay)
{
    phased_backoff_wait_strategy<1000000, 1000000, blocking_wait_strategy> wait_strategy_with_lock;
    wait_strategy_test_util::assert_wait_for_with_delay_of(std::chrono::milliseconds(10),
                                                           wait_strategy_with_lock);
    phased_backoff_wait_strategy<1000000, 1000000, sleeping_wait_strategy<0>>
        wait_strategy_with_sleep;
    wait_strategy_test_util::assert_wait_for_with_delay_of(std::chrono::milliseconds(10),
                                                           wait_strategy_with_sleep);
}
} // namespace test
} // namespace disruptor4cpp
