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

#include <math.h>
#include <stdio.h>
#include <thread>
#include <unistd.h>

#include <fstream>
#include <sstream>

#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <klepsydra/core/cache_listener.h>

#include <klepsydra/high_performance/data_multiplexer_middleware_provider.h>
#include <klepsydra/mem_core/mem_env.h>

#include "gtest/gtest.h"

class DataMultiplexerTestEvent
{
public:
    static std::atomic_int constructorInvokations;
    static std::atomic_int emptyConstructorInvokations;
    static std::atomic_int copyInvokations;

    DataMultiplexerTestEvent(int id, const std::string &message)
        : _id(id)
        , _message(message)
    {
        DataMultiplexerTestEvent::constructorInvokations++;
    }

    DataMultiplexerTestEvent()
    {
        spdlog::debug("new empty invocation!!!");
        DataMultiplexerTestEvent::emptyConstructorInvokations++;
    }

    DataMultiplexerTestEvent(const DataMultiplexerTestEvent &that)
        : _id(that._id)
        , _message(that._message)
    {
        DataMultiplexerTestEvent::copyInvokations++;
    }

    int _id;
    std::string _message;
};

class DataMultiplexerNewTestEvent
{
public:
    DataMultiplexerNewTestEvent(const std::string &label, std::vector<double> values)
        : _label(label)
        , _values(values)
    {}

    DataMultiplexerNewTestEvent() {}

    std::string _label;
    std::vector<double> _values;
};

std::atomic_int DataMultiplexerTestEvent::constructorInvokations(0);
std::atomic_int DataMultiplexerTestEvent::emptyConstructorInvokations(0);
std::atomic_int DataMultiplexerTestEvent::copyInvokations(0);

class DataMultiplexerMiddlewareTest : public ::testing::Test
{
protected:
    DataMultiplexerMiddlewareTest()
    {
        DataMultiplexerTestEvent::constructorInvokations = 0;
        DataMultiplexerTestEvent::emptyConstructorInvokations = 0;
        DataMultiplexerTestEvent::copyInvokations = 0;
    }
};

TEST_F(DataMultiplexerMiddlewareTest, NominalCase)
{
    kpsr::high_performance::DataMultiplexerMiddlewareProvider<DataMultiplexerTestEvent, 4>
        provider(nullptr, "test");
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 0);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 0);

    auto publisher = provider.getPublisher();
    auto subscriber = provider.getSubscriber();
    kpsr::mem::TestCacheListener<DataMultiplexerTestEvent> eventListener(-1);
    subscriber->registerListener("cacheListener", eventListener.cacheListenerFunction);
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 0);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 0);

    for (int i = 0; i < 10; i++) {
        std::this_thread::sleep_for(std::chrono::microseconds(20));
        DataMultiplexerTestEvent event(i, "hola");
        publisher->publish(event);
    }
    std::shared_ptr<kpsr::SubscriptionStats> subscriptionStats = subscriber->getSubscriptionStats(
        "cacheListener");

    for (int i = 0; i < 10; i++) {
        if (publisher->_publicationStats._totalDiscardedEvents +
                subscriptionStats->_totalProcessed + subscriptionStats->_totalDiscardedEvents >=
            10) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    subscriber->removeListener("cacheListener");

    int discardedMessages = publisher->_publicationStats._totalDiscardedEvents;

    spdlog::info("discardedMessages:{}", discardedMessages);
    spdlog::info("subscriptionStats->_totalProcessed:{}", subscriptionStats->_totalProcessed);
    spdlog::info("subscriptionStats->_totalDiscardedEvents:{}",
                 subscriptionStats->_totalDiscardedEvents);

    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 10);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations +
                  (subscriptionStats->_totalDiscardedEvents + discardedMessages) * 2,
              20);

    ASSERT_EQ(9, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hola", eventListener.getLastReceivedEvent()->_message);
    ASSERT_EQ(subscriptionStats->_totalProcessed + subscriptionStats->_totalDiscardedEvents,
              publisher->_publicationStats._totalProcessed - discardedMessages);
}

TEST_F(DataMultiplexerMiddlewareTest, SingleSlowConsumer)
{
    kpsr::high_performance::DataMultiplexerMiddlewareProvider<DataMultiplexerTestEvent, 4>
        provider(nullptr, "test");
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 0);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 0);

    auto high_performancePublisher = provider.getPublisher();
    auto subscriber = provider.getSubscriber();
    kpsr::mem::TestCacheListener<DataMultiplexerTestEvent> eventListener(2);
    subscriber->registerListener("cacheListener", eventListener.cacheListenerFunction);
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 0);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 0);

    for (int i = 0; i < 500; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        DataMultiplexerTestEvent event(i, "hola");
        high_performancePublisher->publish(event);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    std::shared_ptr<kpsr::SubscriptionStats> subscriptionStats = subscriber->getSubscriptionStats(
        "cacheListener");
    subscriber->removeListener("cacheListener");

    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 500);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    int discardedMessages = high_performancePublisher->_publicationStats._totalDiscardedEvents;
    ASSERT_EQ(subscriptionStats->_totalProcessed + discardedMessages +
                  subscriptionStats->_totalDiscardedEvents,
              500);

    auto lastSentId = subscriptionStats->_totalProcessed + discardedMessages +
                      subscriptionStats->_totalDiscardedEvents - 1;
    ASSERT_EQ(lastSentId, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hola", eventListener.getLastReceivedEvent()->_message);
    ASSERT_EQ(subscriptionStats->_totalProcessed, eventListener.counter);
}

TEST_F(DataMultiplexerMiddlewareTest, TwoConsumer)
{
    DataMultiplexerTestEvent event(0, "hola");
    kpsr::high_performance::DataMultiplexerMiddlewareProvider<DataMultiplexerTestEvent, 4>
        provider(nullptr, "test", event);
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 1);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 2);

    auto high_performancePublisher = provider.getPublisher();
    auto subscriber = provider.getSubscriber();
    int slowListenerSleepTime = 2;
    kpsr::mem::TestCacheListener<DataMultiplexerTestEvent> slowListener(slowListenerSleepTime);
    subscriber->registerListener("slowListener", slowListener.cacheListenerFunction);

    kpsr::mem::TestCacheListener<DataMultiplexerTestEvent> fastListener(-1);
    subscriber->registerListener("fastListener", fastListener.cacheListenerFunction);
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 1);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 2);

    int numIterations = 500;
    for (int i = 0; i < numIterations; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        DataMultiplexerTestEvent event(i, "hola");
        high_performancePublisher->publish(event);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(
        2 *
        slowListenerSleepTime)); // Give enough time for last publish event to reach the slow listener.
    std::shared_ptr<kpsr::SubscriptionStats> slowSubscriptionStats =
        subscriber->getSubscriptionStats("slowListener");
    std::shared_ptr<kpsr::SubscriptionStats> fastSubscriptionStats =
        subscriber->getSubscriptionStats("fastListener");
    subscriber->removeListener("slowListener");
    subscriber->removeListener("fastListener");

    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, numIterations + 1);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);

    int publishedMessages = high_performancePublisher->_publicationStats._totalProcessed;
    int discardedMessages = high_performancePublisher->_publicationStats._totalDiscardedEvents;
    spdlog::info("publishedMessages:{}", publishedMessages);
    spdlog::info("discardedMessages:{}", discardedMessages);
    spdlog::info("fastListener.counter:{}", fastListener.counter);
    spdlog::info("fastSubscriptionStats->_totalProcessed:{}",
                 fastSubscriptionStats->_totalProcessed);
    spdlog::info("fastSubscriptionStats->_totalDiscardedEvents:{}",
                 fastSubscriptionStats->_totalDiscardedEvents);
    spdlog::info("slowListener.counter:{}", slowListener.counter);
    spdlog::info("slowSubscriptionStats->_totalProcessed:{}",
                 slowSubscriptionStats->_totalProcessed);
    spdlog::info("slowSubscriptionStats->_totalDiscardedEvents:{}",
                 slowSubscriptionStats->_totalDiscardedEvents);

    auto fastSubscriberEvents = fastSubscriptionStats->_totalProcessed +
                                fastSubscriptionStats->_totalDiscardedEvents;
    auto slowSubscriberEvents = slowSubscriptionStats->_totalProcessed +
                                slowSubscriptionStats->_totalDiscardedEvents;

    ASSERT_EQ(fastSubscriptionStats->_totalProcessed, fastListener.counter);
    ASSERT_EQ(slowSubscriptionStats->_totalProcessed, slowListener.counter);
    ASSERT_GT(fastSubscriptionStats->_totalProcessed, slowSubscriptionStats->_totalProcessed);
    ASSERT_LE(fastSubscriberEvents, publishedMessages - discardedMessages);
    ASSERT_LE(slowSubscriberEvents, publishedMessages - discardedMessages);

    ASSERT_GE(publishedMessages - discardedMessages, fastListener.counter);
    ASSERT_GE(publishedMessages - discardedMessages, slowListener.counter);

    ASSERT_EQ(fastSubscriptionStats->_totalProcessed + slowSubscriptionStats->_totalProcessed,
              fastListener.counter + slowListener.counter);
}

TEST_F(DataMultiplexerMiddlewareTest, SetContainerSuccessTest)
{
    kpsr::high_performance::DataMultiplexerMiddlewareProvider<int, 4> provider(nullptr,
                                                                               "testProvider");

    kpsr::mem::MemEnv environment;
    kpsr::Container testContainer(&environment, "testContainer");

    provider.setContainer(&testContainer);
    auto dummySubscriberTest = provider.getSubscriber();
    ASSERT_EQ(&testContainer, dummySubscriberTest->_container);
}

TEST_F(DataMultiplexerMiddlewareTest, SetContainerAfterSubscriberStartTest)
{
    kpsr::high_performance::DataMultiplexerMiddlewareProvider<int, 4> provider(nullptr,
                                                                               "testProvider");
    kpsr::mem::MemEnv environment;
    kpsr::Container testContainer(&environment, "testContainer");
    auto dummySubscriberTest = provider.getSubscriber();
    dummySubscriberTest->registerListener("dummy", [](const int &event) {
        std::cout << "Got event : " << event << std::endl;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::stringstream programLogStream;
    auto ostream_sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(programLogStream);
    auto logger = std::make_shared<spdlog::logger>("my_logger", ostream_sink);
    spdlog::register_logger(logger);
    spdlog::set_default_logger(logger);
    provider.setContainer(&testContainer);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    spdlog::drop("my_logger");
    dummySubscriberTest->removeListener("dummy");
    ASSERT_EQ(&testContainer, dummySubscriberTest->_container);
    std::string spdlogString = programLogStream.str();
    ASSERT_NE(spdlogString.size(), 0);
    auto console = spdlog::stdout_color_mt("default");
    spdlog::set_default_logger(console);
}
