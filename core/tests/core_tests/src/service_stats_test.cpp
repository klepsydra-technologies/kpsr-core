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

#include "gtest/gtest.h"
#include <klepsydra/core/container.h>
#include <klepsydra/core/service.h>

class dummyService : public kpsr::Service
{
public:
    explicit dummyService(const std::string &name)
        : kpsr::Service(nullptr, name)
    {}

protected:
    virtual void execute() {}
    virtual void start() {}
    virtual void stop() {}
};

TEST(ServiceStatsTest, BasicTests)
{
    {
        ASSERT_NO_FATAL_FAILURE(kpsr::ServiceStats("dummyService"));
    }
    kpsr::ServiceStats dummyServiceStats("dummyService");

    ASSERT_EQ(0, dummyServiceStats.getTotalRunningTimeMs());

    ASSERT_NO_FATAL_FAILURE(dummyServiceStats.stopTimeWatch());
    ASSERT_NO_FATAL_FAILURE(dummyServiceStats.startTimeWatch());
    ASSERT_NO_FATAL_FAILURE(dummyServiceStats.startTimeWatch());
    ASSERT_NO_FATAL_FAILURE(dummyServiceStats.stopTimeWatch());
    ASSERT_LE(0, dummyServiceStats.getTotalRunningTimeMs());

    ASSERT_EQ(0, dummyServiceStats.getMillisecondsSinceStart());

    ASSERT_NO_FATAL_FAILURE(dummyServiceStats.stop());
    ASSERT_NO_FATAL_FAILURE(dummyServiceStats.start());
    ASSERT_LE(0, dummyServiceStats.getMillisecondsSinceStart());
    ASSERT_NO_FATAL_FAILURE(dummyServiceStats.stop());
}
