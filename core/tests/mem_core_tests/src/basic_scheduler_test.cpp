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
#include <chrono>
#include <sstream>
#include <stdio.h>

#include <klepsydra/core/core_container.h>
#include <klepsydra/sdk/environment.h>
#include <klepsydra/sdk/service.h>

#include <klepsydra/mem_core/basic_scheduler.h>

std::chrono::time_point<std::chrono::system_clock> executeTime;
int count;
void task()
{
    executeTime = std::chrono::system_clock::now();
    count++;
}

void long_task()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

class TestService : public kpsr::Service
{
public:
    explicit TestService(kpsr::Container *container, kpsr::Environment *environment)
        : Service(container, environment, "testService")
        , count(0)
        , executeTime(std::chrono::system_clock::now())
    {}

    int count;
    std::chrono::time_point<std::chrono::system_clock> executeTime;

protected:
    void start() override{};

    void stop() override{};

    void execute() override
    {
        count++;
        executeTime = std::chrono::system_clock::now();
    }
};

TEST(BasicSchedulerTest, schedulerTestNominal)
{
    count = 0;
    int microSecondsToWait = 20000;

    kpsr::mem::BasicScheduler scheduler;
    std::string taskName = "test";
    auto startTime = std::chrono::system_clock::now();
    scheduler.startScheduledTask(taskName, microSecondsToWait, false, task);

    auto sleepFor = 10 * microSecondsToWait;
    std::this_thread::sleep_for(std::chrono::microseconds(sleepFor));
    scheduler.stopScheduledTask(taskName);
    std::chrono::duration<long int, std::nano> diff = executeTime - startTime;

    ASSERT_LT(microSecondsToWait, diff.count() / 1000);
    ASSERT_GT(sleepFor, diff.count() / 1000);
    ASSERT_EQ(count, 1);
}

TEST(BasicSchedulerTest, schedulerTestRepeat)
{
    count = 0;
    int microSecondsToWait = 20000;

    kpsr::mem::BasicScheduler scheduler;
    std::string taskName = "test";
    auto startTime = std::chrono::system_clock::now();
    scheduler.startScheduledTask(taskName, microSecondsToWait, true, task);

    auto sleepFor = 10 * microSecondsToWait;
    std::this_thread::sleep_for(std::chrono::microseconds(sleepFor));
    scheduler.stopScheduledTask(taskName);
    std::chrono::duration<long int, std::nano> diff = executeTime - startTime;

    ASSERT_LT(sleepFor, diff.count() / 1000);
    ASSERT_GT(count, 0);
}

TEST(BasicSchedulerTest, schedulerTestServiceNominal)
{
    TestService candidate(nullptr, nullptr);

    int microSecondsToWait = 1000;

    kpsr::mem::BasicScheduler scheduler;
    candidate.startup();
    auto startTime = std::chrono::system_clock::now();
    scheduler.startScheduledService(microSecondsToWait, false, &candidate);

    auto sleepFor = 10 * microSecondsToWait;
    std::this_thread::sleep_for(std::chrono::microseconds(sleepFor));
    scheduler.stopScheduledService(&candidate);
    std::chrono::duration<long int, std::nano> diff = candidate.executeTime - startTime;
    candidate.shutdown();
    ASSERT_LT(microSecondsToWait, diff.count() / 1000);
    ASSERT_GT(sleepFor, diff.count() / 1000);
    ASSERT_EQ(candidate.count, 1);
}

TEST(BasicSchedulerTest, schedulerTestServiceWithContainerNominal)
{
    kpsr::CoreContainer testContainer(nullptr, "testContainer");
    TestService candidate(&testContainer, nullptr);

    int microSecondsToWait = 1000;

    kpsr::mem::BasicScheduler scheduler;
    candidate.startup();
    auto startTime = std::chrono::system_clock::now();
    scheduler.startScheduledService(microSecondsToWait, false, &candidate);

    auto sleepFor = 10 * microSecondsToWait;
    std::this_thread::sleep_for(std::chrono::microseconds(sleepFor));
    scheduler.stopScheduledService(&candidate);
    std::chrono::duration<long int, std::nano> diff = candidate.executeTime - startTime;
    candidate.shutdown();
    ASSERT_LT(microSecondsToWait, diff.count() / 1000);
    ASSERT_GT(sleepFor, diff.count() / 1000);
    ASSERT_EQ(candidate.count, 1);
}

TEST(BasicSchedulerTest, schedulerTestServiceNoStart)
{
    TestService candidate(nullptr, nullptr);

    int microSecondsToWait = 1000;

    kpsr::mem::BasicScheduler scheduler;
    auto startTime = std::chrono::system_clock::now();
    scheduler.startScheduledService(microSecondsToWait, false, &candidate);

    auto sleepFor = 10 * microSecondsToWait;
    std::this_thread::sleep_for(std::chrono::microseconds(sleepFor));
    scheduler.stopScheduledService(&candidate);

    ASSERT_LE(candidate.executeTime, startTime);
    ASSERT_EQ(candidate.count, 0);
}

TEST(BasicSchedulerTest, schedulerTestServiceRepeat)
{
    TestService candidate(nullptr, nullptr);

    int microSecondsToWait = 1000;

    kpsr::mem::BasicScheduler scheduler;
    candidate.startup();
    auto startTime = std::chrono::system_clock::now();
    scheduler.startScheduledService(microSecondsToWait, true, &candidate);

    auto sleepFor = 10 * microSecondsToWait;
    std::this_thread::sleep_for(std::chrono::microseconds(sleepFor));
    scheduler.stopScheduledService(&candidate);
    std::chrono::duration<long int, std::nano> diff = candidate.executeTime - startTime;
    candidate.shutdown();
    ASSERT_LT(sleepFor, diff.count() / 1000);
    ASSERT_GT(candidate.count, 0);
}

TEST(BasicSchedulerTest, schedulerTestServiceRepeatNoStart)
{
    TestService candidate(nullptr, nullptr);

    int microSecondsToWait = 1000;

    kpsr::mem::BasicScheduler scheduler;
    auto startTime = std::chrono::system_clock::now();
    scheduler.startScheduledService(microSecondsToWait, true, &candidate);

    auto sleepFor = 10 * microSecondsToWait;
    std::this_thread::sleep_for(std::chrono::microseconds(sleepFor));
    scheduler.stopScheduledService(&candidate);

    ASSERT_LT(candidate.executeTime, startTime);
    ASSERT_EQ(candidate.count, 0);
}
