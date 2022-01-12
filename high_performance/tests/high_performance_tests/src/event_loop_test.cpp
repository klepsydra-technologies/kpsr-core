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

#include <functional>
#include <memory>
#include <sstream>
#include <vector>

#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <klepsydra/core/cache_listener.h>
#include <klepsydra/core/container.h>
#include <klepsydra/core/smart_object_pool.h>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>
#include <klepsydra/mem_core/mem_env.h>

#include "gtest/gtest.h"

class DummyContainer : public kpsr::Container
{
public:
    DummyContainer()
        : kpsr::Container(nullptr, "dummy")
    {}
};

class SensorData
{
public:
    int id;
    std::vector<double> data;

    SensorData(int id, std::vector<double> data)
        : id(id)
        , data(data)
    {
        spdlog::info("SensorData::SensorData. Main");
    }

    SensorData() { spdlog::info("SensorData::SensorData. Empty"); }

    SensorData(const SensorData &that)
        : id(that.id)
        , data(that.data)
    {
        spdlog::info("SensorData::SensorData. Copy");
    }
};

class ELTestEvent
{
public:
    static int constructorInvokations;
    static int emptyConstructorInvokations;
    static int copyInvokations;

    ELTestEvent(int id, const std::string &message)
        : _id(id)
        , _message(message)
    {
        ELTestEvent::constructorInvokations++;
    }

    ELTestEvent(const ELTestEvent &that)
        : _id(that._id)
        , _message(that._message)
    {
        ELTestEvent::copyInvokations++;
    }

    ELTestEvent() { ELTestEvent::emptyConstructorInvokations++; }

    int _id;
    std::string _message;
};

class ELTestNewEvent
{
public:
    ELTestNewEvent(const std::string &label, std::vector<double> values)
        : _label(label)
        , _values(values)
    {}

    ELTestNewEvent() {}

    std::string _label;
    std::vector<double> _values;
};

int ELTestEvent::constructorInvokations = 0;
int ELTestEvent::emptyConstructorInvokations = 0;
int ELTestEvent::copyInvokations = 0;

class EventLoopTest : public ::testing::Test
{
protected:
    EventLoopTest()
    {
        ELTestEvent::constructorInvokations = 0;
        ELTestEvent::emptyConstructorInvokations = 0;
        ELTestEvent::copyInvokations = 0;
    }
};

TEST_F(EventLoopTest, SharePointerCasting)
{
    std::shared_ptr<void> ringBufferPointer;
    {
        std::shared_ptr<SensorData> sensorDataPointer(new SensorData(1, {1, 2, 3}));
        ringBufferPointer = std::static_pointer_cast<void>(sensorDataPointer);
    }
    std::shared_ptr<SensorData> newSensorDataPointer = std::static_pointer_cast<SensorData>(
        ringBufferPointer);

    ASSERT_EQ(1, newSensorDataPointer->id);
    ASSERT_EQ(std::vector<double>({1, 2, 3}), newSensorDataPointer->data);
}

TEST_F(EventLoopTest, SharePointerCastingWithPool)
{
    kpsr::SmartObjectPool<SensorData> pool("EventLoopTest", 4);
    for (int i = 0; i < 10; i++) {
        std::shared_ptr<void> ringBufferPointer;
        {
            std::shared_ptr<SensorData> sensorDataPointer = std::move(pool.acquire());
            ringBufferPointer = std::static_pointer_cast<void>(sensorDataPointer);
        }
        std::shared_ptr<SensorData> newSensorDataPointer = std::static_pointer_cast<SensorData>(
            ringBufferPointer);
    }
}

TEST_F(EventLoopTest, FullSingleEventEmitterTopicWitOuthPool)
{
    DummyContainer container;
    kpsr::high_performance::EventLoopMiddlewareProvider<4>
        provider(&container,
                 "test_event_loop",
                 kpsr::high_performance::EVENT_LOOP_START_TIMEOUT_MICROSEC,
                 {},
                 kpsr::EventEmitterType::SAFE);
    provider.start();

    kpsr::mem::TestCacheListener<ELTestEvent> eventListener(-1);

    kpsr::Subscriber<ELTestEvent> *subscriber = provider.getSubscriber<ELTestEvent>("ELTestEvent");
    subscriber->registerListener("cacheListener", eventListener.cacheListenerFunction);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::copyInvokations, 0);

    kpsr::Publisher<ELTestEvent> *publisher = provider.getPublisher<ELTestEvent>("ELTestEvent",
                                                                                 0,
                                                                                 nullptr,
                                                                                 nullptr);
    for (int i = 0; i < 10; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ELTestEvent event(i, "hola");
        publisher->publish(event);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    provider.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_EQ(9, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hola", eventListener.getLastReceivedEvent()->_message);
    ASSERT_EQ(subscriber->getSubscriptionStats("cacheListener")->totalProcessed, 10);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 10);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::copyInvokations, 20);
}

TEST_F(EventLoopTest, FullSingleEventEmitterTopicWithPool)
{
    kpsr::high_performance::EventLoopMiddlewareProvider<4>
        provider(nullptr,
                 "test_event_loop",
                 kpsr::high_performance::EVENT_LOOP_START_TIMEOUT_MICROSEC,
                 {},
                 kpsr::EventEmitterType::SAFE);
    provider.start();

    kpsr::mem::TestCacheListener<ELTestEvent> eventListener(-1);

    kpsr::Subscriber<ELTestEvent> *subscriber = provider.getSubscriber<ELTestEvent>("ELTestEvent");
    subscriber->registerListener("cacheListener", eventListener.cacheListenerFunction);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::copyInvokations, 0);

    kpsr::Publisher<ELTestEvent> *publisher = provider.getPublisher<ELTestEvent>("ELTestEvent",
                                                                                 4,
                                                                                 nullptr,
                                                                                 nullptr);
    for (int i = 0; i < 10; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ELTestEvent event(i, "hola");
        publisher->publish(event);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    provider.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_EQ(9, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hola", eventListener.getLastReceivedEvent()->_message);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 10);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(ELTestEvent::copyInvokations, eventListener.counter);
}

TEST_F(EventLoopTest, FullSingleEventEmitterTwoTopicsWithOutPool)
{
    DummyContainer container;
    kpsr::high_performance::EventLoopMiddlewareProvider<4>
        provider(&container,
                 "test_event_loop",
                 kpsr::high_performance::EVENT_LOOP_START_TIMEOUT_MICROSEC,
                 {},
                 kpsr::EventEmitterType::SAFE);
    provider.start();

    kpsr::mem::TestCacheListener<ELTestEvent> eventListener(-1);

    kpsr::Subscriber<ELTestEvent> *subscriber = provider.getSubscriber<ELTestEvent>("ELTestEvent");
    subscriber->registerListener("eventListener", eventListener.cacheListenerFunction);

    kpsr::high_performance::EventLoopPublisher<ELTestEvent, 4> *publisher =
        (kpsr::high_performance::EventLoopPublisher<ELTestEvent, 4> *)
            provider.getPublisher<ELTestEvent>("ELTestEvent", 0, nullptr, nullptr);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::copyInvokations, 0);

    kpsr::mem::TestCacheListener<ELTestNewEvent> newEventListener(-1);

    kpsr::Subscriber<ELTestNewEvent> *newSubscriber = provider.getSubscriber<ELTestNewEvent>(
        "ELTestNewEvent");
    newSubscriber->registerListener("newEventListener", newEventListener.cacheListenerFunction);

    kpsr::high_performance::EventLoopPublisher<ELTestNewEvent, 4> *newPublisher =
        (kpsr::high_performance::EventLoopPublisher<ELTestNewEvent, 4> *)
            provider.getPublisher<ELTestNewEvent>("ELTestNewEvent", 0, nullptr, nullptr);
    const int t1Iterations = 100;
    const int t2Iterations = 200;
    std::thread t1([&publisher] {
        for (int i = 0; i < t1Iterations; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            ELTestEvent event(i, "hola");
            publisher->publish(event);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    });

    std::thread t2([&newPublisher] {
        for (int i = 0; i < t2Iterations; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            ELTestNewEvent event("hola", {(double) i});
            newPublisher->publish(event);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    });

    t1.join();
    t2.join();

    // Wait for all events to be processed.
    const int ATTEMPTS = 10;
    int attemptNo = 0;
    while ((eventListener.counter < t1Iterations) && (newEventListener.counter < t2Iterations) &&
           (attemptNo < ATTEMPTS)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        attemptNo++;
    }

    auto eventListenerStats = subscriber->getSubscriptionStats("eventListener")->totalProcessed;
    auto publisherDiscarded = publisher->_discardedMessages;
    auto newEventListenerStats = newSubscriber->getSubscriptionStats("newEventListener")
                                     ->totalProcessed;
    auto newPublisherDiscarded = newPublisher->_discardedMessages;
    auto totalMessages = eventListenerStats + publisherDiscarded;
    auto totalNewMessages = newEventListenerStats + newPublisherDiscarded;
    subscriber->removeListener("eventListener");
    newSubscriber->removeListener("newEventListener");
    provider.stop();

    ASSERT_EQ(t1Iterations - 1, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hola", eventListener.getLastReceivedEvent()->_message);

    ASSERT_EQ(t2Iterations - 1, newEventListener.getLastReceivedEvent()->_values[0]);

    ASSERT_EQ(totalMessages, t1Iterations);
    ASSERT_EQ(totalNewMessages, t2Iterations);

    ASSERT_EQ(ELTestEvent::constructorInvokations, t1Iterations);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::copyInvokations, t1Iterations + eventListenerStats);
}

TEST_F(EventLoopTest, FullSingleEventEmitterTwoTopicsWithPool)
{
    DummyContainer container;
    kpsr::high_performance::EventLoopMiddlewareProvider<4>
        provider(&container,
                 "test_event_loop",
                 kpsr::high_performance::EVENT_LOOP_START_TIMEOUT_MICROSEC,
                 {},
                 kpsr::EventEmitterType::SAFE);
    provider.start();

    kpsr::mem::TestCacheListener<ELTestEvent> eventListener(10);

    kpsr::Subscriber<ELTestEvent> *subscriber = provider.getSubscriber<ELTestEvent>("ELTestEvent");
    subscriber->registerListener("eventListener", eventListener.cacheListenerFunction);

    kpsr::high_performance::EventLoopPublisher<ELTestEvent, 4> *publisher =
        (kpsr::high_performance::EventLoopPublisher<ELTestEvent, 4> *)
            provider.getPublisher<ELTestEvent>("ELTestEvent", 6, nullptr, nullptr);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 6);
    ASSERT_EQ(ELTestEvent::copyInvokations, 0);

    kpsr::mem::TestCacheListener<ELTestNewEvent> newEventListener(1);

    kpsr::Subscriber<ELTestNewEvent> *newSubscriber = provider.getSubscriber<ELTestNewEvent>(
        "ELTestNewEvent");
    newSubscriber->registerListener("newEventListener", newEventListener.cacheListenerFunction);

    kpsr::high_performance::EventLoopPublisher<ELTestNewEvent, 4> *newPublisher =
        (kpsr::high_performance::EventLoopPublisher<ELTestNewEvent, 4> *)
            provider.getPublisher<ELTestNewEvent>("ELTestNewEvent", 6, nullptr, nullptr);
    const int t1Iterations = 100;
    const int t2Iterations = 200;
    std::thread t1([&publisher] {
        for (int i = 0; i < t1Iterations; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            ELTestEvent event(i, "hola");
            publisher->publish(event);
        }
    });

    std::thread t2([&newPublisher] {
        for (int i = 0; i < t2Iterations; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            ELTestNewEvent event("hola", {(double) i});
            newPublisher->publish(event);
        }
    });

    auto stats = subscriber->getSubscriptionStats("eventListener");
    auto newStats = newSubscriber->getSubscriptionStats("newEventListener");

    t1.join();
    t2.join();
    spdlog::info("EventLoopTest::SingleEventEmitterTwoTopicsWithPool. Finishing.1");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    provider.stop();
    spdlog::info("EventLoopTest::SingleEventEmitterTwoTopicsWithPool. Finishing.2");

    if (publisher->_discardedMessages == 0) {
        ASSERT_EQ(t1Iterations - 1, eventListener.getLastReceivedEvent()->_id);
    }
    ASSERT_EQ("hola", eventListener.getLastReceivedEvent()->_message);

    if (newPublisher->_discardedMessages == 0) {
        ASSERT_EQ(t2Iterations - 1, newEventListener.getLastReceivedEvent()->_values[0]);
    }

    long totalMessages = stats->totalProcessed + publisher->_discardedMessages;
    ASSERT_EQ(totalMessages, t1Iterations);
    long totalNewMessages = newStats->totalProcessed + newPublisher->_discardedMessages;
    ASSERT_EQ(totalNewMessages, t2Iterations);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 100);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 6);
    ASSERT_EQ(ELTestEvent::copyInvokations, eventListener.counter);
}

TEST_F(EventLoopTest, FullSetContainerSuccessTest)
{
    kpsr::high_performance::EventLoopMiddlewareProvider<4>
        provider(nullptr,
                 "test_event_loop",
                 kpsr::high_performance::EVENT_LOOP_START_TIMEOUT_MICROSEC,
                 {},
                 kpsr::EventEmitterType::SAFE);

    kpsr::mem::MemEnv environment;
    kpsr::Container testContainer(&environment, "testContainer");

    provider.setContainer(&testContainer);
    auto dummySubscriberTest = provider.getSubscriber<int>("testSubscriber");
    ASSERT_EQ(&testContainer, dummySubscriberTest->_container);
}

TEST_F(EventLoopTest, FullSetContainerAfterStartTest)
{
    kpsr::high_performance::EventLoopMiddlewareProvider<4>
        provider(nullptr,
                 "test_event_loop",
                 kpsr::high_performance::EVENT_LOOP_START_TIMEOUT_MICROSEC,
                 {},
                 kpsr::EventEmitterType::SAFE);
    provider.start();
    kpsr::mem::MemEnv environment;
    kpsr::Container testContainer(&environment, "testContainer");

    provider.setContainer(&testContainer);
    auto dummySubscriberTest = provider.getSubscriber<int>("testSubscriber");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_EQ(nullptr, dummySubscriberTest->_container);
    provider.stop();
}

TEST_F(EventLoopTest, FullSetContainerAfterSubscribersTest)
{
    kpsr::high_performance::EventLoopMiddlewareProvider<4> provider(nullptr);

    kpsr::mem::MemEnv environment;
    kpsr::Container testContainer(&environment, "testContainer");
    auto dummySubscriberTest = provider.getSubscriber<int>("testSubscriber");

    std::stringstream programLogStream;
    auto ostream_sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(programLogStream);
    auto logger = std::make_shared<spdlog::logger>("my_logger", ostream_sink);
    spdlog::register_logger(logger);
    spdlog::set_default_logger(logger);
    provider.setContainer(&testContainer);

    ASSERT_NE(&testContainer, dummySubscriberTest->_container);
    std::string spdlogString = programLogStream.str();

    ASSERT_NE(spdlogString.size(), 0);
    auto console = spdlog::stdout_color_mt("default");
    spdlog::set_default_logger(console);
    spdlog::drop("my_logger");
}

TEST_F(EventLoopTest, FullSetContainerAfterPublisherTest)
{
    kpsr::high_performance::EventLoopMiddlewareProvider<4>
        provider(nullptr,
                 "test_event_loop",
                 kpsr::high_performance::EVENT_LOOP_START_TIMEOUT_MICROSEC,
                 {},
                 kpsr::EventEmitterType::SAFE);

    kpsr::mem::MemEnv environment;
    kpsr::Container testContainer(&environment, "testContainer");
    auto dummyPublisherTest
        __attribute__((unused)) = provider.getPublisher<int>("testSubscriber", 0, nullptr, nullptr);

    std::stringstream programLogStream;
    auto ostream_sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(programLogStream);
    auto logger = std::make_shared<spdlog::logger>("my_logger", ostream_sink);
    spdlog::register_logger(logger);
    spdlog::set_default_logger(logger);

    provider.setContainer(&testContainer);
    std::string spdlogString = programLogStream.str();

    ASSERT_NE(spdlogString.size(), 0);

    auto console = spdlog::stdout_color_mt("default");
    spdlog::set_default_logger(console);
    spdlog::drop("my_logger");
}

TEST_F(EventLoopTest, FullSetContainerWithScheduler)
{
    kpsr::high_performance::EventLoopMiddlewareProvider<4>
        provider(nullptr,
                 "test_event_loop",
                 kpsr::high_performance::EVENT_LOOP_START_TIMEOUT_MICROSEC,
                 {},
                 kpsr::EventEmitterType::SAFE);

    kpsr::mem::MemEnv environment;
    kpsr::Container testContainer(&environment, "testContainer");
    auto dummyScheduler __attribute__((unused)) = provider.getScheduler("testScheduler");

    std::stringstream programLogStream;
    auto ostream_sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(programLogStream);
    auto logger = std::make_shared<spdlog::logger>("my_logger", ostream_sink);
    spdlog::register_logger(logger);
    spdlog::set_default_logger(logger);
    provider.setContainer(&testContainer);

    std::string spdlogString = programLogStream.str();
    ASSERT_EQ(spdlogString.size(), 0);
    auto console = spdlog::stdout_color_mt("default");
    spdlog::set_default_logger(console);
    spdlog::drop("my_logger");
}

TEST_F(EventLoopTest, FixedSingleEventEmitterTopicWitOuthPool)
{
    DummyContainer container;
    kpsr::high_performance::EventLoopMiddlewareProvider<4>
        provider(&container,
                 "test_event_loop",
                 kpsr::high_performance::EVENT_LOOP_START_TIMEOUT_MICROSEC,
                 {},
                 kpsr::EventEmitterType::UNSAFE_MULTI);
    provider.start();

    kpsr::mem::TestCacheListener<ELTestEvent> eventListener(-1);

    kpsr::Subscriber<ELTestEvent> *subscriber =
        provider.getSubscriber<ELTestEvent>("ELTestEvent", kpsr::EventEmitterType::UNSAFE_MULTI);
    subscriber->registerListener("cacheListener", eventListener.cacheListenerFunction);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::copyInvokations, 0);

    kpsr::Publisher<ELTestEvent> *publisher = provider.getPublisher<ELTestEvent>("ELTestEvent",
                                                                                 0,
                                                                                 nullptr,
                                                                                 nullptr);
    for (int i = 0; i < 10; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ELTestEvent event(i, "hola");
        publisher->publish(event);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    provider.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_EQ(9, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hola", eventListener.getLastReceivedEvent()->_message);
    ASSERT_EQ(subscriber->getSubscriptionStats("cacheListener")->totalProcessed, 10);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 10);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::copyInvokations, 20);
}

TEST_F(EventLoopTest, FixedSingleEventEmitterTopicWithPool)
{
    DummyContainer container;
    kpsr::high_performance::EventLoopMiddlewareProvider<4>
        provider(&container,
                 "test_event_loop",
                 kpsr::high_performance::EVENT_LOOP_START_TIMEOUT_MICROSEC,
                 {},
                 kpsr::EventEmitterType::UNSAFE_MULTI);
    provider.start();

    kpsr::mem::TestCacheListener<ELTestEvent> eventListener(-1);

    kpsr::Subscriber<ELTestEvent> *subscriber =
        provider.getSubscriber<ELTestEvent>("ELTestEvent", kpsr::EventEmitterType::UNSAFE_MULTI);
    subscriber->registerListener("cacheListener", eventListener.cacheListenerFunction);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::copyInvokations, 0);

    kpsr::Publisher<ELTestEvent> *publisher = provider.getPublisher<ELTestEvent>("ELTestEvent",
                                                                                 4,
                                                                                 nullptr,
                                                                                 nullptr);
    for (int i = 0; i < 10; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        ELTestEvent event(i, "hola");
        publisher->publish(event);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    provider.stop();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    ASSERT_EQ(9, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hola", eventListener.getLastReceivedEvent()->_message);
    ASSERT_EQ(subscriber->getSubscriptionStats("cacheListener")->totalProcessed, 10);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 10);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(ELTestEvent::copyInvokations, eventListener.counter);
}

TEST_F(EventLoopTest, FixedSingleEventEmitterTwoTopicsWithOutPool)
{
    DummyContainer container;
    kpsr::high_performance::EventLoopMiddlewareProvider<4>
        provider(&container,
                 "test_event_loop",
                 kpsr::high_performance::EVENT_LOOP_START_TIMEOUT_MICROSEC,
                 {},
                 kpsr::EventEmitterType::UNSAFE_MULTI);
    provider.start();

    kpsr::mem::TestCacheListener<ELTestEvent> eventListener(-1);

    kpsr::Subscriber<ELTestEvent> *subscriber =
        provider.getSubscriber<ELTestEvent>("ELTestEvent", kpsr::EventEmitterType::UNSAFE_MULTI);
    subscriber->registerListener("eventListener", eventListener.cacheListenerFunction);

    kpsr::high_performance::EventLoopPublisher<ELTestEvent, 4> *publisher =
        (kpsr::high_performance::EventLoopPublisher<ELTestEvent, 4> *)
            provider.getPublisher<ELTestEvent>("ELTestEvent", 0, nullptr, nullptr);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::copyInvokations, 0);

    kpsr::mem::TestCacheListener<ELTestNewEvent> newEventListener(-1);

    kpsr::Subscriber<ELTestNewEvent> *newSubscriber =
        provider.getSubscriber<ELTestNewEvent>("ELTestNewEvent",
                                               kpsr::EventEmitterType::UNSAFE_MULTI);
    newSubscriber->registerListener("newEventListener", newEventListener.cacheListenerFunction);

    kpsr::high_performance::EventLoopPublisher<ELTestNewEvent, 4> *newPublisher =
        (kpsr::high_performance::EventLoopPublisher<ELTestNewEvent, 4> *)
            provider.getPublisher<ELTestNewEvent>("ELTestNewEvent", 0, nullptr, nullptr);
    const int t1Iterations = 100;
    const int t2Iterations = 200;
    std::thread t1([&publisher] {
        for (int i = 0; i < t1Iterations; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            ELTestEvent event(i, "hola");
            publisher->publish(event);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    });

    std::thread t2([&newPublisher] {
        for (int i = 0; i < t2Iterations; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            ELTestNewEvent event("hola", {(double) i});
            newPublisher->publish(event);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    });

    t1.join();
    t2.join();

    // Wait for all events to be processed.
    const int ATTEMPTS = 10;
    int attemptNo = 0;
    while ((eventListener.counter < t1Iterations) && (newEventListener.counter < t2Iterations) &&
           (attemptNo < ATTEMPTS)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        attemptNo++;
    }

    auto eventListenerStats = subscriber->getSubscriptionStats("eventListener")->totalProcessed;
    auto publisherDiscarded = publisher->_discardedMessages;
    auto newEventListenerStats = newSubscriber->getSubscriptionStats("newEventListener")
                                     ->totalProcessed;
    auto newPublisherDiscarded = newPublisher->_discardedMessages;
    auto totalMessages = eventListenerStats + publisherDiscarded;
    auto totalNewMessages = newEventListenerStats + newPublisherDiscarded;
    subscriber->removeListener("eventListener");
    newSubscriber->removeListener("newEventListener");
    provider.stop();

    ASSERT_EQ(t1Iterations - 1, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hola", eventListener.getLastReceivedEvent()->_message);

    ASSERT_EQ(t2Iterations - 1, newEventListener.getLastReceivedEvent()->_values[0]);

    ASSERT_EQ(totalMessages, t1Iterations);
    ASSERT_EQ(totalNewMessages, t2Iterations);

    ASSERT_EQ(ELTestEvent::constructorInvokations, t1Iterations);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::copyInvokations, t1Iterations + eventListenerStats);
}

TEST_F(EventLoopTest, FixedSingleEventEmitterTwoTopicsWithPool)
{
    DummyContainer container;
    kpsr::high_performance::EventLoopMiddlewareProvider<4>
        provider(&container,
                 "test_event_loop",
                 kpsr::high_performance::EVENT_LOOP_START_TIMEOUT_MICROSEC,
                 {},
                 kpsr::EventEmitterType::UNSAFE_MULTI);
    provider.start();

    kpsr::mem::TestCacheListener<ELTestEvent> eventListener(10);

    kpsr::Subscriber<ELTestEvent> *subscriber =
        provider.getSubscriber<ELTestEvent>("ELTestEvent", kpsr::EventEmitterType::UNSAFE_MULTI);
    subscriber->registerListener("eventListener", eventListener.cacheListenerFunction);

    kpsr::high_performance::EventLoopPublisher<ELTestEvent, 4> *publisher =
        (kpsr::high_performance::EventLoopPublisher<ELTestEvent, 4> *)
            provider.getPublisher<ELTestEvent>("ELTestEvent", 6, nullptr, nullptr);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 6);
    ASSERT_EQ(ELTestEvent::copyInvokations, 0);

    kpsr::mem::TestCacheListener<ELTestNewEvent> newEventListener(1);

    kpsr::Subscriber<ELTestNewEvent> *newSubscriber =
        provider.getSubscriber<ELTestNewEvent>("ELTestNewEvent",
                                               kpsr::EventEmitterType::UNSAFE_MULTI);
    newSubscriber->registerListener("newEventListener", newEventListener.cacheListenerFunction);

    kpsr::high_performance::EventLoopPublisher<ELTestNewEvent, 4> *newPublisher =
        (kpsr::high_performance::EventLoopPublisher<ELTestNewEvent, 4> *)
            provider.getPublisher<ELTestNewEvent>("ELTestNewEvent", 6, nullptr, nullptr);
    const int t1Iterations = 100;
    const int t2Iterations = 200;
    std::thread t1([&publisher] {
        for (int i = 0; i < t1Iterations; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            ELTestEvent event(i, "hola");
            publisher->publish(event);
        }
    });

    std::thread t2([&newPublisher] {
        for (int i = 0; i < t2Iterations; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            ELTestNewEvent event("hola", {(double) i});
            newPublisher->publish(event);
        }
    });

    auto stats = subscriber->getSubscriptionStats("eventListener");
    auto newStats = newSubscriber->getSubscriptionStats("newEventListener");

    t1.join();
    t2.join();
    spdlog::info("EventLoopTest::SingleEventEmitterTwoTopicsWithPool. Finishing.1");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    provider.stop();
    spdlog::info("EventLoopTest::SingleEventEmitterTwoTopicsWithPool. Finishing.2");

    if (publisher->_discardedMessages == 0) {
        ASSERT_EQ(t1Iterations - 1, eventListener.getLastReceivedEvent()->_id);
    }
    ASSERT_EQ("hola", eventListener.getLastReceivedEvent()->_message);

    if (newPublisher->_discardedMessages == 0) {
        ASSERT_EQ(t2Iterations - 1, newEventListener.getLastReceivedEvent()->_values[0]);
    }

    long totalMessages = stats->totalProcessed + publisher->_discardedMessages;
    ASSERT_EQ(totalMessages, t1Iterations);
    long totalNewMessages = newStats->totalProcessed + newPublisher->_discardedMessages;
    ASSERT_EQ(totalNewMessages, t2Iterations);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 100);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 6);
    ASSERT_EQ(ELTestEvent::copyInvokations, eventListener.counter);
}

TEST_F(EventLoopTest, FixedSetContainerSuccessTest)
{
    kpsr::high_performance::EventLoopMiddlewareProvider<4>
        provider(nullptr,
                 "test_event_loop",
                 kpsr::high_performance::EVENT_LOOP_START_TIMEOUT_MICROSEC,
                 {},
                 kpsr::EventEmitterType::UNSAFE_MULTI);

    kpsr::mem::MemEnv environment;
    kpsr::Container testContainer(&environment, "testContainer");

    provider.setContainer(&testContainer);
    auto dummySubscriberTest = provider.getSubscriber<int>("testSubscriber",
                                                           kpsr::EventEmitterType::UNSAFE_MULTI);
    ASSERT_EQ(&testContainer, dummySubscriberTest->_container);
}

TEST_F(EventLoopTest, FixedSetContainerAfterStartTest)
{
    kpsr::high_performance::EventLoopMiddlewareProvider<4>
        provider(nullptr,
                 "test_event_loop",
                 kpsr::high_performance::EVENT_LOOP_START_TIMEOUT_MICROSEC,
                 {},
                 kpsr::EventEmitterType::UNSAFE_MULTI);
    provider.start();
    kpsr::mem::MemEnv environment;
    kpsr::Container testContainer(&environment, "testContainer");

    provider.setContainer(&testContainer);
    auto dummySubscriberTest = provider.getSubscriber<int>("testSubscriber",
                                                           kpsr::EventEmitterType::UNSAFE_MULTI);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_EQ(nullptr, dummySubscriberTest->_container);
    provider.stop();
}

TEST_F(EventLoopTest, FixedSetContainerAfterSubscribersTest)
{
    kpsr::high_performance::EventLoopMiddlewareProvider<4> provider(nullptr);

    kpsr::mem::MemEnv environment;
    kpsr::Container testContainer(&environment, "testContainer");
    auto dummySubscriberTest = provider.getSubscriber<int>("testSubscriber",
                                                           kpsr::EventEmitterType::UNSAFE_MULTI);

    std::stringstream programLogStream;
    auto ostream_sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(programLogStream);
    auto logger = std::make_shared<spdlog::logger>("my_logger", ostream_sink);
    spdlog::register_logger(logger);
    spdlog::set_default_logger(logger);
    provider.setContainer(&testContainer);

    ASSERT_NE(&testContainer, dummySubscriberTest->_container);
    std::string spdlogString = programLogStream.str();

    ASSERT_NE(spdlogString.size(), 0);
    auto console = spdlog::stdout_color_mt("default");
    spdlog::set_default_logger(console);
    spdlog::drop("my_logger");
}

TEST_F(EventLoopTest, FixedSetContainerAfterPublisherTest)
{
    kpsr::high_performance::EventLoopMiddlewareProvider<4>
        provider(nullptr,
                 "test_event_loop",
                 kpsr::high_performance::EVENT_LOOP_START_TIMEOUT_MICROSEC,
                 {},
                 kpsr::EventEmitterType::UNSAFE_MULTI);

    kpsr::mem::MemEnv environment;
    kpsr::Container testContainer(&environment, "testContainer");
    auto dummyPublisherTest
        __attribute__((unused)) = provider.getPublisher<int>("testSubscriber", 0, nullptr, nullptr);

    std::stringstream programLogStream;
    auto ostream_sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(programLogStream);
    auto logger = std::make_shared<spdlog::logger>("my_logger", ostream_sink);
    spdlog::register_logger(logger);
    spdlog::set_default_logger(logger);

    provider.setContainer(&testContainer);
    std::string spdlogString = programLogStream.str();

    ASSERT_NE(spdlogString.size(), 0);

    auto console = spdlog::stdout_color_mt("default");
    spdlog::set_default_logger(console);
    spdlog::drop("my_logger");
}

TEST_F(EventLoopTest, FixedSetContainerWithScheduler)
{
    kpsr::high_performance::EventLoopMiddlewareProvider<4>
        provider(nullptr,
                 "test_event_loop",
                 kpsr::high_performance::EVENT_LOOP_START_TIMEOUT_MICROSEC,
                 {},
                 kpsr::EventEmitterType::UNSAFE_MULTI);

    kpsr::mem::MemEnv environment;
    kpsr::Container testContainer(&environment, "testContainer");
    auto dummyScheduler __attribute__((unused)) = provider.getScheduler("testScheduler");

    std::stringstream programLogStream;
    auto ostream_sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(programLogStream);
    auto logger = std::make_shared<spdlog::logger>("my_logger", ostream_sink);
    spdlog::register_logger(logger);
    spdlog::set_default_logger(logger);
    provider.setContainer(&testContainer);

    std::string spdlogString = programLogStream.str();
    ASSERT_EQ(spdlogString.size(), 0);
    auto console = spdlog::stdout_color_mt("default");
    spdlog::set_default_logger(console);
    spdlog::drop("my_logger");
}

TEST_F(EventLoopTest, StartStopTest)
{
    kpsr::high_performance::EventLoopMiddlewareProvider<256>::RingBuffer _ringBuffer;
    std::shared_ptr<kpsr::EventEmitterInterface<kpsr::high_performance::EventloopDataWrapper>>
        _eventEmitter = kpsr::EventEmitterFactory::createEventEmitter<
            kpsr::high_performance::EventloopDataWrapper>(kpsr::EventEmitterType::SAFE);
    std::string name = "kpsr_EL";

    kpsr::high_performance::EventLoop<256> eventLoop(_eventEmitter, _ringBuffer, name);

    eventLoop.start();
    ASSERT_TRUE(eventLoop.isStarted());
    ASSERT_TRUE(eventLoop.isRunning());
    eventLoop.stop();
    ASSERT_FALSE(eventLoop.isStarted());
}

TEST_F(EventLoopTest, StartStopFastTest)
{
    kpsr::high_performance::EventLoopMiddlewareProvider<256>::RingBuffer _ringBuffer;
    std::shared_ptr<kpsr::EventEmitterInterface<kpsr::high_performance::EventloopDataWrapper>>
        _eventEmitter = kpsr::EventEmitterFactory::createEventEmitter<
            kpsr::high_performance::EventloopDataWrapper>(kpsr::EventEmitterType::SAFE);
    std::string name = "kpsr_EL";

    kpsr::high_performance::EventLoop<256> eventLoop(_eventEmitter, _ringBuffer, name);

    eventLoop.start();
    eventLoop.stop();
    ASSERT_FALSE(eventLoop.isStarted());
    ASSERT_FALSE(eventLoop.isRunning());
}

TEST_F(EventLoopTest, StartTwiceTest)
{
    kpsr::high_performance::EventLoopMiddlewareProvider<256>::RingBuffer _ringBuffer;
    std::shared_ptr<kpsr::EventEmitterInterface<kpsr::high_performance::EventloopDataWrapper>>
        _eventEmitter = kpsr::EventEmitterFactory::createEventEmitter<
            kpsr::high_performance::EventloopDataWrapper>(kpsr::EventEmitterType::SAFE);
    std::string name = "kpsr_EL";
    std::string messageToCheck(kpsr::high_performance::EVENT_LOOP_START_MESSAGE);
    kpsr::high_performance::EventLoop<256> eventLoop(_eventEmitter, _ringBuffer, name);

    std::stringstream programLogStream;
    auto ostream_sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(programLogStream);
    auto logger = std::make_shared<spdlog::logger>("my_logger", ostream_sink);
    logger->set_pattern("%v");
    logger->set_level(spdlog::level::debug);

    spdlog::register_logger(logger);
    spdlog::set_default_logger(logger);
    ASSERT_NO_THROW(eventLoop.start());
    ASSERT_NO_THROW(eventLoop.start());
    std::string spdlogString = programLogStream.str();
    eventLoop.stop();
    auto console = spdlog::stdout_color_mt("default");
    spdlog::set_default_logger(console);
    spdlog::drop("my_logger");

    auto found = spdlogString.find(messageToCheck);
    ASSERT_NE(found, std::string::npos);
    ASSERT_EQ(spdlogString.find(messageToCheck, ++found), std::string::npos);
}

TEST_F(EventLoopTest, StartStopTwiceTest)
{
    kpsr::high_performance::EventLoopMiddlewareProvider<256>::RingBuffer _ringBuffer;
    std::shared_ptr<kpsr::EventEmitterInterface<kpsr::high_performance::EventloopDataWrapper>>
        _eventEmitter = kpsr::EventEmitterFactory::createEventEmitter<
            kpsr::high_performance::EventloopDataWrapper>(kpsr::EventEmitterType::SAFE);
    std::string name = "kpsr_EL";
    std::string messageToCheck(kpsr::high_performance::EVENT_LOOP_START_MESSAGE);
    messageToCheck += "\n";
    kpsr::high_performance::EventLoop<256> eventLoop(_eventEmitter, _ringBuffer, name);

    ASSERT_NO_THROW(eventLoop.start());
    eventLoop.stop();
    std::stringstream programLogStream;
    auto ostream_sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(programLogStream);
    auto logger = std::make_shared<spdlog::logger>("my_logger", ostream_sink);
    logger->set_pattern("%v");
    logger->set_level(spdlog::level::debug);
    spdlog::register_logger(logger);
    spdlog::set_default_logger(logger);
    ASSERT_NO_THROW(eventLoop.start());
    ASSERT_TRUE(eventLoop.isStarted());
    std::string spdlogString = programLogStream.str();
    auto console = spdlog::stdout_color_mt("default");
    spdlog::set_default_logger(console);
    spdlog::drop("my_logger");
    eventLoop.stop();

    auto found = spdlogString.find(messageToCheck);
    ASSERT_NE(found, std::string::npos);
}
