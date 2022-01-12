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

#include "gtest/gtest.h"
#include <chrono>
#include <sstream>
#include <stdio.h>

#include <klepsydra/core/environment.h>
#include <klepsydra/core/service.h>

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
    explicit TestService(kpsr::Environment *environment)
        : Service(environment, "testService")
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
    auto ptr = std::function<void()>(task);
    int microSecondsToWait = 20000;

    kpsr::mem::BasicScheduler scheduler;
    std::string taskName = "test";
    auto startTime = std::chrono::system_clock::now();
    scheduler.startScheduledTask(taskName, microSecondsToWait, false, ptr);

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
    auto ptr = std::function<void()>(task);
    int microSecondsToWait = 20000;

    kpsr::mem::BasicScheduler scheduler;
    std::string taskName = "test";
    auto startTime = std::chrono::system_clock::now();
    scheduler.startScheduledTask(taskName, microSecondsToWait, true, ptr);

    auto sleepFor = 10 * microSecondsToWait;
    std::this_thread::sleep_for(std::chrono::microseconds(sleepFor));
    scheduler.stopScheduledTask(taskName);
    std::chrono::duration<long int, std::nano> diff = executeTime - startTime;

    ASSERT_LT(sleepFor, diff.count() / 1000);
    ASSERT_GT(count, 0);
}

TEST(BasicSchedulerTest, schedulerTestServiceNominal)
{
    TestService candidate(nullptr);

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
    TestService candidate(nullptr);

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
    TestService candidate(nullptr);

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
