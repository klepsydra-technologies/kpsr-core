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

#include <stdio.h>
#include <thread>
#include <unistd.h>
#include <math.h>

#include <sstream>
#include <fstream>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <klepsydra/core/cache_listener.h>

#include <klepsydra/mem_core/mem_env.h>
#include <klepsydra/high_performance/data_multiplexer_middleware_provider.h>

#include "gtest/gtest.h"

class DataMultiplexerTestEvent {
public:

    static std::atomic_int constructorInvokations;
    static std::atomic_int emptyConstructorInvokations;
    static std::atomic_int copyInvokations;

    DataMultiplexerTestEvent(int id, const std::string & message)
        : _id(id)
        , _message(message) {
        DataMultiplexerTestEvent::constructorInvokations++;
    }

    DataMultiplexerTestEvent() {
        spdlog::info("new empty invocation!!!");
        DataMultiplexerTestEvent::emptyConstructorInvokations++;
    }

    DataMultiplexerTestEvent(const DataMultiplexerTestEvent & that)
        : _id(that._id)
        , _message(that._message) {
        DataMultiplexerTestEvent::copyInvokations++;
    }

    int _id;
    std::string _message;
};

class DataMultiplexerNewTestEvent {
public:

    DataMultiplexerNewTestEvent(const std::string & label, std::vector<double> values)
        : _label(label)
        , _values(values) {
    }

    DataMultiplexerNewTestEvent() {
    }

    std::string _label;
    std::vector<double> _values;
};

std::atomic_int DataMultiplexerTestEvent::constructorInvokations(0);
std::atomic_int DataMultiplexerTestEvent::emptyConstructorInvokations(0);
std::atomic_int DataMultiplexerTestEvent::copyInvokations(0);

TEST(DataMultiplexerMiddlewareTest, NominalCase) {
    kpsr::high_performance::DataMultiplexerMiddlewareProvider<DataMultiplexerTestEvent, 4> provider(nullptr, "test");
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 0);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 0);

    kpsr::mem::TestCacheListener<DataMultiplexerTestEvent> eventListener(-1);
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 0);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 0);

    for (int i = 0; i < 10; i ++) {
        std::this_thread::sleep_for(std::chrono::microseconds(20));
        DataMultiplexerTestEvent event(i, "hola");
        provider.getPublisher()->publish(event);
    }
    std::shared_ptr<kpsr::SubscriptionStats> subscriptionStats = provider.getSubscriber()->getSubscriptionStats("cacheListener");

    for (int i = 0; i < 10; i ++) {
       if (provider.getPublisher()->_publicationStats._totalDiscardedEvents +
           subscriptionStats->_totalProcessed + subscriptionStats->_totalDiscardedEvents >= 10) {
           break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    provider.getSubscriber()->removeListener("cacheListener");

    int discardedMessages = provider.getPublisher()->_publicationStats._totalDiscardedEvents;

    spdlog::info("discardedMessages:{}", discardedMessages);
    spdlog::info("subscriptionStats->_totalProcessed:{}", subscriptionStats->_totalProcessed);
    spdlog::info("subscriptionStats->_totalDiscardedEvents:{}", subscriptionStats->_totalDiscardedEvents);

    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 10);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations + subscriptionStats->_totalDiscardedEvents*2, 20);

    ASSERT_EQ(9, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hola", eventListener.getLastReceivedEvent()->_message);
    ASSERT_EQ(subscriptionStats->_totalProcessed + subscriptionStats->_totalDiscardedEvents, 10);
}

TEST(DataMultiplexerMiddlewareTest, SingleSlowConsumer) {
    DataMultiplexerTestEvent::constructorInvokations = 0;
    DataMultiplexerTestEvent::emptyConstructorInvokations = 0;
    DataMultiplexerTestEvent::copyInvokations = 0;
    kpsr::high_performance::DataMultiplexerMiddlewareProvider<DataMultiplexerTestEvent, 4> provider(nullptr, "test");
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 0);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 0);

    kpsr::mem::TestCacheListener<DataMultiplexerTestEvent> eventListener(2);
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 0);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 0);

    for (int i = 0; i < 500; i ++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        DataMultiplexerTestEvent event(i, "hola");
        provider.getPublisher()->publish(event);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(3));
    std::shared_ptr<kpsr::SubscriptionStats> subscriptionStats = provider.getSubscriber()->getSubscriptionStats("cacheListener");
    provider.getSubscriber()->removeListener("cacheListener");

    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 500);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    kpsr::high_performance::DataMultiplexerPublisher<DataMultiplexerTestEvent, 4> * high_performancePublisher =
            ((kpsr::high_performance::DataMultiplexerPublisher<DataMultiplexerTestEvent, 4> *) provider.getPublisher());
    int discardedMessages = high_performancePublisher->_publicationStats._totalDiscardedEvents;
    ASSERT_EQ(eventListener.counter + discardedMessages + subscriptionStats->_totalDiscardedEvents, 500);

    ASSERT_EQ(499, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hola", eventListener.getLastReceivedEvent()->_message);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->_totalProcessed, eventListener.counter);
}

TEST(DataMultiplexerMiddlewareTest, TwoConsumer) {
    DataMultiplexerTestEvent::constructorInvokations = 0;
    DataMultiplexerTestEvent::emptyConstructorInvokations = 0;
    DataMultiplexerTestEvent::copyInvokations = 0;
    DataMultiplexerTestEvent event(0, "hola");
    kpsr::high_performance::DataMultiplexerMiddlewareProvider<DataMultiplexerTestEvent, 4> provider(nullptr, "test", event);
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 1);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 2);

    kpsr::mem::TestCacheListener<DataMultiplexerTestEvent> slowListener(2);
    provider.getSubscriber()->registerListener("slowListener", slowListener.cacheListenerFunction);

    kpsr::mem::TestCacheListener<DataMultiplexerTestEvent> fastListener(-1);
    provider.getSubscriber()->registerListener("fastListener", fastListener.cacheListenerFunction);
    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 1);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(DataMultiplexerTestEvent::copyInvokations, 2);

    for (int i = 0; i < 500; i ++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        DataMultiplexerTestEvent event(i, "hola");
        provider.getPublisher()->publish(event);
    }
    while (fastListener.getLastReceivedEvent()->_id < 499) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    while (slowListener.getLastReceivedEvent()->_id < 499) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::shared_ptr<kpsr::SubscriptionStats> slowSubscriptionStats = provider.getSubscriber()->getSubscriptionStats("slowListener");
    std::shared_ptr<kpsr::SubscriptionStats> fastSubscriptionStats = provider.getSubscriber()->getSubscriptionStats("fastListener");
    provider.getSubscriber()->removeListener("slowListener");
    provider.getSubscriber()->removeListener("fastListener");

    ASSERT_EQ(DataMultiplexerTestEvent::constructorInvokations, 501);
    ASSERT_EQ(DataMultiplexerTestEvent::emptyConstructorInvokations, 4);
    kpsr::high_performance::DataMultiplexerPublisher<DataMultiplexerTestEvent, 4> * high_performancePublisher =
            ((kpsr::high_performance::DataMultiplexerPublisher<DataMultiplexerTestEvent, 4> *) provider.getPublisher());

    int discardedMessages = high_performancePublisher->_publicationStats._totalDiscardedEvents;
    spdlog::info("discardedMessages:{}", discardedMessages);
    spdlog::info("fastListener.counter:{}", fastListener.counter);
    spdlog::info("fastSubscriptionStats->_totalProcessed:{}", fastSubscriptionStats->_totalProcessed);
    spdlog::info("fastSubscriptionStats->_totalDiscardedEvents:{}", fastSubscriptionStats->_totalDiscardedEvents);
    spdlog::info("slowListener.counter:{}", slowListener.counter);
    spdlog::info("slowSubscriptionStats->_totalProcessed:{}", slowSubscriptionStats->_totalProcessed);
    spdlog::info("slowSubscriptionStats->_totalDiscardedEvents:{}", slowSubscriptionStats->_totalDiscardedEvents);

    ASSERT_GT(fastSubscriptionStats->_totalProcessed,  slowSubscriptionStats->_totalProcessed);
    ASSERT_EQ(fastListener.counter + slowListener.counter + discardedMessages + discardedMessages +
              fastSubscriptionStats->_totalDiscardedEvents + slowSubscriptionStats->_totalDiscardedEvents, 1000);

    ASSERT_EQ(499, fastListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hola", fastListener.getLastReceivedEvent()->_message);

    ASSERT_EQ(499, slowListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hola", slowListener.getLastReceivedEvent()->_message);
    ASSERT_EQ(fastSubscriptionStats->_totalProcessed + slowSubscriptionStats->_totalProcessed, fastListener.counter + slowListener.counter);
}

TEST(DataMultiplexerMiddlewareTest, SetContainerSuccessTest) {

    kpsr::high_performance::DataMultiplexerMiddlewareProvider<int, 4> provider(nullptr, "testProvider");

    kpsr::mem::MemEnv environment;
    kpsr::Container testContainer(&environment, "testContainer");

    provider.setContainer(&testContainer);
    auto dummySubscriberTest = provider.getSubscriber();
    ASSERT_EQ(&testContainer, dummySubscriberTest->_container);
}

TEST(DataMultiplexerMiddlewareTest, SetContainerAfterSubscriberStartTest) {

    kpsr::high_performance::DataMultiplexerMiddlewareProvider<int, 4> provider(nullptr, "testProvider");
    kpsr::mem::MemEnv environment;
    kpsr::Container testContainer(&environment, "testContainer");
    auto dummySubscriberTest = provider.getSubscriber();
    dummySubscriberTest->registerListener("dummy", [](const int& event) {std::cout << "Got event : " << event << std::endl;});
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::stringstream programLogStream;
    auto ostream_sink = std::make_shared<spdlog::sinks::ostream_sink_mt> (programLogStream);
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
