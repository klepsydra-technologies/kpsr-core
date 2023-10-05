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

#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <klepsydra/core/cache_listener.h>
#include <klepsydra/core/core_container.h>

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

using TestTuple =
    std::tuple<kpsr::Container *,
               std::function<void(DataMultiplexerTestEvent &)>,
               std::function<void(const DataMultiplexerTestEvent &, DataMultiplexerTestEvent &)>>;

class DataMultiplexerMiddlewareConstructorTest : public DataMultiplexerMiddlewareTest,
                                                 public ::testing::WithParamInterface<TestTuple>
{
protected:
    virtual void SetUp() { std::tie(testContainer, eventInitializer, eventCloner) = GetParam(); }
    kpsr::Container *testContainer;
    std::function<void(DataMultiplexerTestEvent &)> eventInitializer;
    std::function<void(const DataMultiplexerTestEvent &, DataMultiplexerTestEvent &)> eventCloner;
};

kpsr::mem::MemEnv testEnvironment;
kpsr::CoreContainer testContainer(&testEnvironment, "TestEnvironment");

INSTANTIATE_TEST_SUITE_P(
    DataMuxVariations,
    DataMultiplexerMiddlewareConstructorTest,
    ::testing::Combine(::testing::Values(nullptr, &testContainer),
                       ::testing::Values(nullptr,
                                         [](DataMultiplexerTestEvent &event) {
                                             event._id = -1;
                                             event._message = "";
                                             return;
                                         }),
                       ::testing::Values(nullptr,
                                         [](const DataMultiplexerTestEvent &source,
                                            DataMultiplexerTestEvent &dest) {
                                             dest._id = source._id;
                                             dest._message = source._message;
                                         })));

TEST_P(DataMultiplexerMiddlewareConstructorTest, NominalTest)
{
    auto macroWrapper = [this]() {
        kpsr::high_performance::DataMultiplexerMiddlewareProvider<DataMultiplexerTestEvent, 4>
            provider(this->testContainer, "test", this->eventInitializer, this->eventCloner);
    };
    ASSERT_NO_THROW(macroWrapper());
}

TEST_P(DataMultiplexerMiddlewareConstructorTest, NominalCase)
{
    kpsr::high_performance::DataMultiplexerMiddlewareProvider<DataMultiplexerTestEvent, 4>
        provider(this->testContainer, "test", eventInitializer, eventCloner);
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 0);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 0);

    auto publisher = provider.getPublisher();
    auto subscriber = provider.getSubscriber("testSubscriber");

    {
        ASSERT_NO_THROW(provider.getSubscriber("testSubscriber"));
    }
    {
        auto subscriber2 = provider.getSubscriber("testSubscriber");
        ASSERT_EQ(subscriber2, subscriber);
    }
    kpsr::mem::TestCacheListener<DataMultiplexerTestEvent> eventListener(-1);
    subscriber->registerListener("cacheListener", eventListener.cacheListenerFunction);
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 0);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 0);

    const int numPublish = 10;

    for (int i = 0; i < numPublish; i++) {
        std::this_thread::sleep_for(std::chrono::microseconds(20));
        DataMultiplexerTestEvent event(i, "hola");
        publisher->publish(event);
    }
    std::shared_ptr<kpsr::SubscriptionStats> subscriptionStats = subscriber->getSubscriptionStats(
        "cacheListener");

    auto &publisherStats = publisher->publicationStats;

    for (int i = 0; i < numPublish; i++) {
        if (publisherStats.totalDiscardedEvents + subscriptionStats->totalProcessed +
                subscriptionStats->totalDiscardedEvents >=
            numPublish) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    subscriber->removeListener("cacheListener");

    int discardedMessages = publisherStats.totalDiscardedEvents;

    spdlog::info("publisher discardedMessages:{}", discardedMessages);
    spdlog::info("subscriptionStats->totalProcessed:{}", subscriptionStats->totalProcessed);
    spdlog::info("subscriptionStats->totalDiscardedEvents:{}",
                 subscriptionStats->totalDiscardedEvents);
    spdlog::info("DataMultiplexerTestEvent::copyInvokations {}",
                 DataMultiplexerTestEvent::copyInvokations);

    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, numPublish);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations,
              (subscriptionStats->totalProcessed) +
                  (eventCloner == nullptr) * (publisherStats.totalProcessed - discardedMessages));

    ASSERT_EQ(subscriptionStats->totalProcessed + subscriptionStats->totalDiscardedEvents,
              publisher->publicationStats.totalProcessed - discardedMessages);
    ASSERT_EQ(subscriptionStats->totalProcessed, eventListener.counter);
}

TEST_P(DataMultiplexerMiddlewareConstructorTest, NominalCaseSharedPtr)
{
    kpsr::high_performance::DataMultiplexerMiddlewareProvider<DataMultiplexerTestEvent, 4>
        provider(this->testContainer, "test", eventInitializer, eventCloner);
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 0);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 0);

    auto publisher = provider.getPublisher();
    auto subscriber = provider.getSubscriber("testSubscriber");

    {
        ASSERT_NO_THROW(provider.getSubscriber("testSubscriber"));
    }
    {
        auto subscriber2 = provider.getSubscriber("testSubscriber");
        ASSERT_EQ(subscriber2, subscriber);
    }
    kpsr::mem::TestCacheListener<DataMultiplexerTestEvent> eventListener(-1);
    subscriber->registerListener("cacheListener", eventListener.cacheListenerFunction);
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 0);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 0);

    const int numPublish = 10;

    for (int i = 0; i < numPublish; i++) {
        std::this_thread::sleep_for(std::chrono::microseconds(20));
        auto event = std::make_shared<DataMultiplexerTestEvent>(i, "hola");
        publisher->publish(event);
    }
    std::shared_ptr<kpsr::SubscriptionStats> subscriptionStats = subscriber->getSubscriptionStats(
        "cacheListener");

    auto &publisherStats = publisher->publicationStats;

    for (int i = 0; i < numPublish; i++) {
        if (publisherStats.totalDiscardedEvents + subscriptionStats->totalProcessed +
                subscriptionStats->totalDiscardedEvents >=
            numPublish) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    subscriber->removeListener("cacheListener");

    int discardedMessages = publisherStats.totalDiscardedEvents;

    spdlog::info("publisher discardedMessages:{}", discardedMessages);
    spdlog::info("subscriptionStats->totalProcessed:{}", subscriptionStats->totalProcessed);
    spdlog::info("subscriptionStats->totalDiscardedEvents:{}",
                 subscriptionStats->totalDiscardedEvents);
    spdlog::info("DataMultiplexerTestEvent::copyInvokations {}",
                 DataMultiplexerTestEvent::copyInvokations);

    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, numPublish);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, subscriptionStats->totalProcessed);

    ASSERT_EQ(subscriptionStats->totalProcessed + subscriptionStats->totalDiscardedEvents,
              publisher->publicationStats.totalProcessed - discardedMessages);
    ASSERT_EQ(subscriptionStats->totalProcessed, eventListener.counter);
}

TEST_P(DataMultiplexerMiddlewareConstructorTest, SingleSlowConsumer)
{
    kpsr::high_performance::DataMultiplexerMiddlewareProvider<DataMultiplexerTestEvent, 4>
        provider(this->testContainer, "test", eventInitializer, eventCloner);
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 0);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 0);

    auto high_performancePublisher = provider.getPublisher();
    std::shared_ptr<kpsr::SubscriptionStats> subscriptionStats;
    const int numPublish = 10;
    int slowConsumerTimeMs = 3;
    kpsr::mem::TestCacheListener<DataMultiplexerTestEvent> eventListener(2);
    {
        auto subscriber = provider.getSubscriber("testSubscriber");
        subscriber->registerListener("cacheListener", eventListener.cacheListenerFunction);
        ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 0);
        ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
        ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 0);

        for (int i = 0; i < numPublish; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            DataMultiplexerTestEvent event(i, "hola");
            high_performancePublisher->publish(event);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(slowConsumerTimeMs));
        subscriptionStats = subscriber->getSubscriptionStats("cacheListener");
        subscriber->removeListener("cacheListener");
    }
    // give sufficient time for potential consumers to finish
    std::this_thread::sleep_for(std::chrono::milliseconds(slowConsumerTimeMs));
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, numPublish);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    int discardedMessages = high_performancePublisher->publicationStats.totalDiscardedEvents;
    int totalPublisherProcessedMessages = high_performancePublisher->publicationStats.totalProcessed;

    spdlog::debug("discardedMessages:{}", discardedMessages);
    spdlog::debug("subscriptionStats->totalProcessed:{}", subscriptionStats->totalProcessed);
    spdlog::debug("subscriptionStats->totalDiscardedEvents:{}",
                  subscriptionStats->totalDiscardedEvents);
    spdlog::debug("high_performancePublisher->_publicationStats.totalProcessed = {}",
                  totalPublisherProcessedMessages);

    EXPECT_EQ(DataMultiplexerTestEvent::copyInvokations,
              (subscriptionStats->totalProcessed) +
                  (eventCloner == nullptr) * (totalPublisherProcessedMessages - discardedMessages));

    EXPECT_EQ(subscriptionStats->totalProcessed + discardedMessages +
                  subscriptionStats->totalDiscardedEvents,
              totalPublisherProcessedMessages);

    EXPECT_EQ(subscriptionStats->totalProcessed, eventListener.counter);
}

TEST_F(DataMultiplexerMiddlewareTest, TwoConsumer)
{
    DataMultiplexerTestEvent event(0, "hola");
    kpsr::high_performance::DataMultiplexerMiddlewareProvider<DataMultiplexerTestEvent, 4>
        provider(nullptr, "test", event);
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 1);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 1);

    auto high_performancePublisher = provider.getPublisher();
    auto fastSubscriber = provider.getSubscriber("fast_subscriber");
    auto slowSubscriber = provider.getSubscriber("slow_subscriber");
    int slowListenerSleepTime = 2;
    kpsr::mem::TestCacheListener<DataMultiplexerTestEvent> slowListener(slowListenerSleepTime);
    slowSubscriber->registerListener("slowListener", slowListener.cacheListenerFunction);

    kpsr::mem::TestCacheListener<DataMultiplexerTestEvent> fastListener(-1);
    fastSubscriber->registerListener("fastListener", fastListener.cacheListenerFunction);
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 1);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 1);

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
        slowSubscriber->getSubscriptionStats("slowListener");
    std::shared_ptr<kpsr::SubscriptionStats> fastSubscriptionStats =
        fastSubscriber->getSubscriptionStats("fastListener");
    slowSubscriber->removeListener("slowListener");
    fastSubscriber->removeListener("fastListener");

    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, numIterations + 1);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);

    int publishedMessages = high_performancePublisher->publicationStats.totalProcessed;
    int discardedMessages = high_performancePublisher->publicationStats.totalDiscardedEvents;

    auto fastSubscriberEvents = fastSubscriptionStats->totalProcessed +
                                fastSubscriptionStats->totalDiscardedEvents;
    auto slowSubscriberEvents = slowSubscriptionStats->totalProcessed +
                                slowSubscriptionStats->totalDiscardedEvents;
    // Fast subscriber processes more data than slow
    ASSERT_GE(fastSubscriptionStats->totalProcessed, slowSubscriptionStats->totalProcessed);
    // Events sent by publisher are all received by subscribers

    ASSERT_EQ(fastSubscriberEvents, publishedMessages - discardedMessages);
    ASSERT_EQ(slowSubscriberEvents, publishedMessages - discardedMessages);

    ASSERT_EQ(publishedMessages - discardedMessages,
              fastListener.counter + fastSubscriptionStats->totalDiscardedEvents);
    ASSERT_EQ(publishedMessages - discardedMessages,
              slowListener.counter + slowSubscriptionStats->totalDiscardedEvents);

    ASSERT_EQ(fastSubscriptionStats->totalProcessed + slowSubscriptionStats->totalProcessed,
              fastListener.counter + slowListener.counter);
}

TEST_F(DataMultiplexerMiddlewareTest, SingleConsumerTwoListeners)
{
    DataMultiplexerTestEvent event(0, "hola");
    kpsr::high_performance::DataMultiplexerMiddlewareProvider<DataMultiplexerTestEvent, 4>
        provider(nullptr, "test", event);
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 1);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 1);

    auto high_performancePublisher = provider.getPublisher();
    auto subscriber = provider.getSubscriber("fast_subscriber");
    int slowListenerSleepTime = 2;

    std::thread::id slowListenerThreadId;
    std::thread::id fastListenerThreadId;

    auto slowListener = [&](std::shared_ptr<const DataMultiplexerTestEvent> event) {
        slowListenerThreadId = std::this_thread::get_id();
        std::this_thread::sleep_for(std::chrono::milliseconds(slowListenerSleepTime));
    };
    subscriber->registerSharedPtrListener("slowListener", slowListener);

    auto fastListener =
        [&fastListenerThreadId](std::shared_ptr<const DataMultiplexerTestEvent> event) {
            fastListenerThreadId = std::this_thread::get_id();
        };
    subscriber->registerSharedPtrListener("fastListener", fastListener);

    int numIterations = 10;
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

    ASSERT_EQ(slowListenerThreadId, fastListenerThreadId);
    ASSERT_NE(slowListenerThreadId, std::this_thread::get_id());
}

TEST_F(DataMultiplexerMiddlewareTest, TwoConsumersListeners)
{
    DataMultiplexerTestEvent event(0, "hola");
    kpsr::high_performance::DataMultiplexerMiddlewareProvider<DataMultiplexerTestEvent, 4>
        provider(nullptr, "test", event);
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 1);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 1);

    auto high_performancePublisher = provider.getPublisher();
    auto fastSubscriber = provider.getSubscriber("fast_subscriber");
    auto slowSubscriber = provider.getSubscriber("slow_subscriber");
    int slowListenerSleepTime = 2;

    std::thread::id slowListenerThreadId;
    std::thread::id fastListenerThreadId;

    auto slowListener = [&](std::shared_ptr<const DataMultiplexerTestEvent> event) {
        slowListenerThreadId = std::this_thread::get_id();
        std::this_thread::sleep_for(std::chrono::milliseconds(slowListenerSleepTime));
    };
    slowSubscriber->registerSharedPtrListener("slowListener", slowListener);

    auto fastListener =
        [&fastListenerThreadId](std::shared_ptr<const DataMultiplexerTestEvent> event) {
            fastListenerThreadId = std::this_thread::get_id();
        };
    fastSubscriber->registerSharedPtrListener("fastListener", fastListener);

    int numIterations = 10;
    for (int i = 0; i < numIterations; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        DataMultiplexerTestEvent event(i, "hola");
        high_performancePublisher->publish(event);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(
        2 *
        slowListenerSleepTime)); // Give enough time for last publish event to reach the slow listener.
    slowSubscriber->removeListener("slowListener");
    fastSubscriber->removeListener("fastListener");

    ASSERT_NE(slowListenerThreadId, fastListenerThreadId);
    ASSERT_NE(slowListenerThreadId, std::this_thread::get_id());
    ASSERT_NE(fastListenerThreadId, std::this_thread::get_id());
}

TEST_P(DataMultiplexerMiddlewareConstructorTest, SetContainerSuccessTest)
{
    kpsr::high_performance::DataMultiplexerMiddlewareProvider<DataMultiplexerTestEvent, 4>
        provider(nullptr, "test", eventInitializer, eventCloner);

    kpsr::mem::MemEnv environment;
    kpsr::CoreContainer testContainer(&environment, "testContainer");
    provider.setContainer(&testContainer);

    auto dummySubscriberTest = provider.getSubscriber("test_subscriber");
    ASSERT_EQ(&testContainer, dummySubscriberTest->container);
    provider.setContainer(nullptr);
}

TEST_P(DataMultiplexerMiddlewareConstructorTest, SetContainerAfterSubscriberStartTest)
{
    kpsr::high_performance::DataMultiplexerMiddlewareProvider<DataMultiplexerTestEvent, 4>
        provider(nullptr, "test", eventInitializer, eventCloner);
    kpsr::mem::MemEnv environment;
    kpsr::CoreContainer testContainer(&environment, "testContainer");
    auto dummySubscriberTest = provider.getSubscriber("test_subscriber");
    dummySubscriberTest->registerListener("dummy", [](const DataMultiplexerTestEvent &event) {
        std::cout << "Got event : " << std::endl;
    });

    std::stringstream programLogStream;
    auto ostream_sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(programLogStream);
    auto logger = std::make_shared<spdlog::logger>("my_logger", ostream_sink);
    spdlog::register_logger(logger);
    spdlog::set_default_logger(logger);

    provider.setContainer(&testContainer);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    dummySubscriberTest->removeListener("dummy");
    ASSERT_EQ(&testContainer, dummySubscriberTest->container);
    std::string spdlogString = programLogStream.str();
    ASSERT_NE(spdlogString.size(), 0);

    auto console = spdlog::stdout_color_mt("default");
    spdlog::set_default_logger(console);
    spdlog::drop("my_logger");
    provider.setContainer(nullptr);
}
