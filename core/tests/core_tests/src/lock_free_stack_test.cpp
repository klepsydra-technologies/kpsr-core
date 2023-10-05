// Copyright 2023 Klepsydra Technologies AG
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>

#include "gtest/gtest.h"

#include <spdlog/spdlog.h>

#include <klepsydra/core/lock_free_stack.h>

TEST(LockFreeStackTest, allElemUsed)
{
    int capacity = 4;

    kpsr::LockFreeStack<long> stack(capacity, nullptr);

    std::vector<long> v(capacity);

    for (auto it = v.begin(); it != v.end(); ++it) {
        ASSERT_TRUE(stack.pop(*it));
    }

    long new_elem;
    ASSERT_FALSE(stack.pop(new_elem));
}

TEST(LockFreeStackTest, allElemFree)
{
    kpsr::LockFreeStack<long> stack(4, nullptr);

    long new_elem = 0;
    ASSERT_FALSE(stack.push(new_elem));
}

TEST(LockFreeStackTest, initializer)
{
    int capacity = 10;
    long cnt = capacity;

    std::function<void(long &)> elementInitializer = [&](long &data) { data = --cnt; };

    kpsr::LockFreeStack<long> stack(capacity, elementInitializer);

    std::vector<long> v(capacity);

    long i = capacity;
    for (std::vector<long>::iterator it = v.begin(); it != v.end(); ++it) {
        ASSERT_TRUE(stack.pop(*it));
        ASSERT_EQ(*it, --i);
    }
}

// This test is intended to check if the objects are retrieved from the pool
//    in the reverse order as they had been previously returned to the pool
TEST(LockFreeStackTest, objectAddressOrder)
{
    std::function<void(uintptr_t &)> elementInitializer = [&](uintptr_t &data) {
        data = reinterpret_cast<uintptr_t>(&data);
        spdlog::debug("LockFreeStack initFunction: init stack object at address 0x{0:x}", data);
    };

    kpsr::LockFreeStack<uintptr_t> stack(4, elementInitializer);

    uintptr_t addr0, addr1, addr2, addr3;

    ASSERT_TRUE(stack.pop(addr0));
    spdlog::debug("Got object at address 0x{0:x}", addr0);
    ASSERT_TRUE(stack.pop(addr1));
    spdlog::debug("Got object at address 0x{0:x}", addr1);
    ASSERT_TRUE(stack.pop(addr2));
    spdlog::debug("Got object at address 0x{0:x}", addr2);
    ASSERT_TRUE(stack.pop(addr3));
    spdlog::debug("Got object at address 0x{0:x}", addr3);

    ASSERT_TRUE(stack.push(addr0));
    spdlog::debug("Released object (had 0x{0:x})", addr0);
    ASSERT_TRUE(stack.push(addr1));
    spdlog::debug("Released object (had 0x{0:x})", addr1);

    uintptr_t addr4_1;
    ASSERT_TRUE(stack.pop(addr4_1));
    ASSERT_EQ(addr1, addr4_1);
    spdlog::debug("Got object at address 0x{0:x}", addr4_1);

    uintptr_t addr5_0;
    ASSERT_TRUE(stack.pop(addr5_0));
    ASSERT_EQ(addr0, addr5_0);
    spdlog::debug("Got object at address 0x{0:x}", addr5_0);

    ASSERT_TRUE(stack.push(addr3));
    spdlog::debug("Released object (had 0x{0:x})", addr3);

    ASSERT_TRUE(stack.push(addr2));
    spdlog::debug("Released object (had 0x{0:x})", addr2);

    uintptr_t addr6_2;
    ASSERT_TRUE(stack.pop(addr6_2));
    ASSERT_EQ(addr2, addr6_2);
    spdlog::debug("Got object at address 0x{0:x}", addr6_2);

    uintptr_t addr7_3;
    ASSERT_TRUE(stack.pop(addr7_3));
    ASSERT_EQ(addr3, addr7_3);
    spdlog::debug("Got object at address 0x{0:x}", addr7_3);
}
