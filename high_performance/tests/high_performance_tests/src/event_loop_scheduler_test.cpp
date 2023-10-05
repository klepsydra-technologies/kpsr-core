/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2031  Klepsydra Technologies AG
*                            All Rights Reserved.
*
*  This file is subject to the terms and conditions defined in
*  file 'LICENSE.md', which is part of this source code package.
*
*  NOTICE:  All information contained herein is, and remains the property of Klepsydra
*  Technologies AG and its suppliers, if any. The intellectual and technical concepts
*  contained herein are proprietary to Klepsydra Technologies AG and its suppliers and
*  may be covered by Swiss and Foreign Patents, patents in process, and are protected by
*  trade secret or copyright law. Dissemination of this information or reproduction of
*  this material is strictly forbidden unless prior written permission is obtained from
*  Klepsydra Technologies AG.
*
****************************************************************************/

#include <chrono>

#include <spdlog/spdlog.h>

#include <klepsydra/core/core_container.h>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>

#include "gtest/gtest.h"

struct Task
{
    Task()
        : count(0)
        , executeTime(std::chrono::system_clock::now())
    {}

    void task()
    {
        executeTime = std::chrono::system_clock::now();
        count++;
    }

    void long_task() { std::this_thread::sleep_for(std::chrono::milliseconds(1000)); }

    int count;
    std::chrono::time_point<std::chrono::system_clock> executeTime;
};

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

TEST(EventLoopSchedulerTest, schedulerTestNominal)
{
    Task t;
    int microSecondsToWait = 20000;

    kpsr::high_performance::EventLoopMiddlewareProvider<4> provider(nullptr);
    provider.start();

    std::string taskName = "test";
    auto startTime = std::chrono::system_clock::now();
    provider.getScheduler()->startScheduledTask(taskName,
                                                microSecondsToWait,
                                                false,
                                                std::bind(&Task::task, &t));

    auto sleepFor = 10 * microSecondsToWait;
    std::this_thread::sleep_for(std::chrono::microseconds(sleepFor));
    provider.getScheduler()->stopScheduledTask(taskName);
    std::chrono::duration<long int, std::nano> diff = t.executeTime - startTime;
    provider.stop();

    ASSERT_LT(microSecondsToWait, diff.count() / 1000);
    ASSERT_GT(sleepFor, diff.count() / 1000);
    ASSERT_EQ(t.count, 1);
}

TEST(EventLoopSchedulerTest, schedulerTestRepeat)
{
    Task t;
    int microSecondsToWait = 20000;

    kpsr::high_performance::EventLoopMiddlewareProvider<4> provider(nullptr);
    provider.start();

    std::string taskName = "test";
    auto startTime = std::chrono::system_clock::now();
    provider.getScheduler()->startScheduledTask(taskName,
                                                microSecondsToWait,
                                                true,
                                                std::bind(&Task::task, &t));

    auto sleepFor = 10 * microSecondsToWait;
    std::this_thread::sleep_for(std::chrono::microseconds(sleepFor));
    provider.getScheduler()->stopScheduledTask(taskName);
    std::chrono::duration<long int, std::nano> diff = t.executeTime - startTime;
    provider.stop();

    ASSERT_GT(diff.count() / 1000, 9 * microSecondsToWait);
    ASSERT_GE(t.count, 9);
}

TEST(EventLoopSchedulerTest, schedulerTestServiceNominal)
{
    TestService candidate(nullptr, nullptr);

    int microSecondsToWait = 1000;

    kpsr::high_performance::EventLoopMiddlewareProvider<4> provider(nullptr);
    provider.start();
    candidate.startup();
    auto startTime = std::chrono::system_clock::now();
    provider.getScheduler()->startScheduledService(microSecondsToWait, false, &candidate);

    auto sleepFor = 10 * microSecondsToWait;
    std::this_thread::sleep_for(std::chrono::microseconds(sleepFor));
    provider.getScheduler()->stopScheduledService(&candidate);
    std::chrono::duration<long int, std::nano> diff = candidate.executeTime - startTime;
    candidate.shutdown();
    provider.stop();

    ASSERT_LT(microSecondsToWait, diff.count() / 1000);
    ASSERT_GT(sleepFor, diff.count() / 1000);
    ASSERT_EQ(candidate.count, 1);
}

TEST(EventLoopSchedulerTest, schedulerTestServiceWithContainerNominal)
{
    kpsr::CoreContainer testContainer(nullptr, "testContainer");
    TestService candidate(&testContainer, nullptr);

    int microSecondsToWait = 1000;

    kpsr::high_performance::EventLoopMiddlewareProvider<4> provider(nullptr);
    provider.start();
    candidate.startup();
    auto startTime = std::chrono::system_clock::now();
    provider.getScheduler()->startScheduledService(microSecondsToWait, false, &candidate);

    auto sleepFor = 10 * microSecondsToWait;
    std::this_thread::sleep_for(std::chrono::microseconds(sleepFor));
    provider.getScheduler()->stopScheduledService(&candidate);
    std::chrono::duration<long int, std::nano> diff = candidate.executeTime - startTime;
    candidate.shutdown();
    provider.stop();

    ASSERT_LT(microSecondsToWait, diff.count() / 1000);
    ASSERT_GT(sleepFor, diff.count() / 1000);
    ASSERT_EQ(candidate.count, 1);
}

TEST(EventLoopSchedulerTest, schedulerTestServiceNoStart)
{
    TestService candidate(nullptr, nullptr);

    int microSecondsToWait = 1000;

    kpsr::high_performance::EventLoopMiddlewareProvider<4> provider(nullptr);
    provider.start();
    auto startTime = std::chrono::system_clock::now();
    provider.getScheduler()->startScheduledService(microSecondsToWait, false, &candidate);

    auto sleepFor = 10 * microSecondsToWait;
    std::this_thread::sleep_for(std::chrono::microseconds(sleepFor));
    provider.getScheduler()->stopScheduledService(&candidate);
    provider.stop();

    ASSERT_LE(candidate.executeTime, startTime);
    ASSERT_EQ(candidate.count, 0);
}

TEST(EventLoopSchedulerTest, schedulerTestServiceRepeat)
{
    TestService candidate(nullptr, nullptr);

    int microSecondsToWait = 1000;

    kpsr::high_performance::EventLoopMiddlewareProvider<4> provider(nullptr);
    provider.start();
    candidate.startup();
    auto startTime = std::chrono::system_clock::now();
    provider.getScheduler()->startScheduledService(microSecondsToWait, true, &candidate);

    auto sleepFor = 10 * microSecondsToWait;
    std::this_thread::sleep_for(std::chrono::microseconds(sleepFor));
    provider.getScheduler()->stopScheduledService(&candidate);
    std::chrono::duration<long int, std::nano> diff = candidate.executeTime - startTime;
    provider.stop();
    candidate.shutdown();

    ASSERT_GT(diff.count() / 1000, 9 * microSecondsToWait);
    ASSERT_GE(candidate.count, 9);
}

TEST(EventLoopSchedulerTest, schedulerTestServiceRepeatNoStart)
{
    TestService candidate(nullptr, nullptr);

    int microSecondsToWait = 1000;

    kpsr::high_performance::EventLoopMiddlewareProvider<4> provider(nullptr);
    provider.start();
    auto startTime = std::chrono::system_clock::now();
    provider.getScheduler()->startScheduledService(microSecondsToWait, true, &candidate);

    auto sleepFor = 10 * microSecondsToWait;
    std::this_thread::sleep_for(std::chrono::microseconds(sleepFor));
    provider.getScheduler()->stopScheduledService(&candidate);
    provider.stop();

    ASSERT_LT(candidate.executeTime, startTime);
    ASSERT_EQ(candidate.count, 0);
}
