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

#include <math.h>
#include <stdio.h>
#include <thread>
#include <unistd.h>

#include <fstream>
#include <sstream>

#include "gtest/gtest.h"

#include <klepsydra/serialization/void_caster_mapper.h>

struct Event
{
    int id;
    float fValue;
    double dValue;
};

TEST(SerializationTests, VoidCasterMapperTest)
{
    std::vector<unsigned char> message;
    Event event;
    event.id = 1;
    event.fValue = 2.0;
    event.dValue = 3.0;

    kpsr::Mapper<Event, std::vector<unsigned char>> voidCasterMapper;
    voidCasterMapper.toMiddleware(event, message);

    Event fmEvent;
    voidCasterMapper.fromMiddleware(message, fmEvent);
    ASSERT_EQ(event.id, fmEvent.id);
    ASSERT_EQ(event.fValue, fmEvent.fValue);
    ASSERT_EQ(event.dValue, fmEvent.dValue);
}
