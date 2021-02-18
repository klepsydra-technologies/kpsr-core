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

#include "gtest/gtest.h"

#include <klepsydra/mem_core/basic_middleware_provider.h>
#include <klepsydra/core/cache_listener.h>

class SQTestEvent {
public:

    static std::atomic_int constructorInvokations;
    static std::atomic_int emptyConstructorInvokations;
    static std::atomic_int copyInvokations;

    SQTestEvent(int id, const std::string & message)
        : _id(id)
        , _message(message) {
        SQTestEvent::constructorInvokations++;
    }

    SQTestEvent() {
        SQTestEvent::emptyConstructorInvokations++;
    }

    SQTestEvent(const SQTestEvent & that)
        : _id(that._id)
        , _message(that._message) {
        SQTestEvent::copyInvokations++;
    }

    int _id;
    std::string _message;
};

class SQTestNewEvent {
public:

    static std::atomic_int constructorInvokations;
    static std::atomic_int emptyConstructorInvokations;
    static std::atomic_int copyInvokations;

    SQTestNewEvent(const std::string & label, std::vector<double> values)
        : _label(label)
        , _values(values) {
        SQTestNewEvent::constructorInvokations++;
    }

    SQTestNewEvent() {
        SQTestNewEvent::emptyConstructorInvokations++;
    }

    SQTestNewEvent(const SQTestNewEvent & that)
        : _label(that._label)
        , _values(that._values) {
        SQTestNewEvent::copyInvokations++;
    }

    std::string _label;
    std::vector<double> _values;
};

std::atomic_int SQTestEvent::constructorInvokations(0);
std::atomic_int SQTestEvent::emptyConstructorInvokations(0);
std::atomic_int SQTestEvent::copyInvokations(0);

std::atomic_int SQTestNewEvent::constructorInvokations(0);
std::atomic_int SQTestNewEvent::emptyConstructorInvokations(0);
std::atomic_int SQTestNewEvent::copyInvokations(0);

TEST(BasicEventEmitterTest, SingleEventEmitterTopic) {

    kpsr::mem::BasicMiddlewareProvider<SQTestEvent> provider(nullptr, "event", 4, 0, nullptr, nullptr, false);
    provider.start();
    kpsr::mem::TestCacheListener<SQTestEvent> eventListener(-1);
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    std::thread t2([&provider]{
        SQTestEvent event1(1, "hello");
        provider.getPublisher()->publish(event1);
        SQTestEvent event2(2, "hallo");
        provider.getPublisher()->publish(event2);
        event2._id = 3;
        event2._message = "hola";
        provider.getPublisher()->publish(event2);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    });

    t2.join();
    provider.stop();
    auto lastReceivedEvent = eventListener.getLastReceivedEvent();
    ASSERT_NE(lastReceivedEvent.get(), nullptr);
    ASSERT_EQ(3, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hola", eventListener.getLastReceivedEvent()->_message);

    ASSERT_EQ(eventListener.counter, 3);
    ASSERT_EQ(SQTestEvent::emptyConstructorInvokations,0);
    ASSERT_EQ(SQTestEvent::constructorInvokations, 2);
    ASSERT_EQ(SQTestEvent::copyInvokations, 6);
}

TEST(BasicEventEmitterTest, WithObjectPoolNoFailures) {
    SQTestEvent::emptyConstructorInvokations = 0;
    SQTestEvent::constructorInvokations = 0;
    SQTestEvent::copyInvokations = 0;

    kpsr::mem::BasicMiddlewareProvider<SQTestEvent> provider(nullptr, "event", 4, 4, nullptr, nullptr, false);
    ASSERT_EQ(SQTestEvent::emptyConstructorInvokations, 4);

    provider.start();

    kpsr::mem::TestCacheListener<SQTestEvent> eventListener(-1);
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    std::thread t2([&provider]{
        for (int i = 0; i < 10; i++) {
            SQTestEvent event1(i, "hello");
            provider.getPublisher()->publish(event1);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    });

    t2.join();
    provider.stop();

    auto lastReceivedEvent = eventListener.getLastReceivedEvent();
    ASSERT_EQ(9, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hello", eventListener.getLastReceivedEvent()->_message);

    ASSERT_EQ(eventListener.counter, 10);
    ASSERT_EQ(SQTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(SQTestEvent::constructorInvokations, 10);
    ASSERT_EQ(SQTestEvent::copyInvokations, 10);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->_totalProcessed, 10);
}

TEST(BasicEventEmitterTest, WithObjectPoolWithFailuresBlocking) {
    SQTestEvent::emptyConstructorInvokations = 0;
    SQTestEvent::constructorInvokations = 0;
    SQTestEvent::copyInvokations = 0;

    kpsr::mem::BasicMiddlewareProvider<SQTestEvent> provider(nullptr, "event", 4, 6, nullptr, nullptr, false);
    ASSERT_EQ(SQTestEvent::emptyConstructorInvokations, 6);

    provider.start();
    kpsr::mem::TestCacheListener<SQTestEvent> eventListener(1);
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    std::thread t2([&provider]{
        for (int i = 0; i < 300; i++) {
            SQTestEvent event1(i, "hello");
            provider.getPublisher()->publish(event1);
        }
        while (!provider._internalQueue.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    t2.join();
    provider.stop();

    auto lastReceivedEvent = eventListener.getLastReceivedEvent();
    ASSERT_NE(lastReceivedEvent.get(), nullptr);
    ASSERT_EQ(299, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hello", eventListener.getLastReceivedEvent()->_message);

    ASSERT_GE(eventListener.counter, 300);
    ASSERT_EQ(SQTestEvent::emptyConstructorInvokations, 6);
    ASSERT_EQ(SQTestEvent::constructorInvokations, 300);
    ASSERT_GE(SQTestEvent::copyInvokations, 300);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->_totalProcessed, 300);
}

TEST(BasicEventEmitterTest, WithObjectPoolWithFailuresNonBlocking) {
    SQTestEvent::emptyConstructorInvokations = 0;
    SQTestEvent::constructorInvokations = 0;
    SQTestEvent::copyInvokations = 0;

    kpsr::mem::BasicMiddlewareProvider<SQTestEvent> provider(nullptr, "event", 4, 6, nullptr, nullptr, true);
    ASSERT_EQ(SQTestEvent::emptyConstructorInvokations, 6);

    provider.start();

    kpsr::mem::TestCacheListener<SQTestEvent> eventListener(1);
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    std::thread t2([&provider]{
        for (int i = 0; i < 300; i++) {
            SQTestEvent event1(i, "hello");
            provider.getPublisher()->publish(event1);
        }
        while (!provider._internalQueue.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });

    t2.join();
    provider.stop();

    auto lastReceivedEvent = eventListener.getLastReceivedEvent();
    ASSERT_NE(lastReceivedEvent.get(), nullptr);
    ASSERT_EQ(299, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hello", eventListener.getLastReceivedEvent()->_message);

    ASSERT_GE(eventListener.counter, 4);
    ASSERT_EQ(SQTestEvent::emptyConstructorInvokations, 6);
    ASSERT_EQ(SQTestEvent::constructorInvokations, 300);
    ASSERT_GE(SQTestEvent::copyInvokations, 4);
    int totalMessages = provider.getSubscriber()->getSubscriptionStats("cacheListener")->_totalProcessed
            + ((kpsr::mem::BasicPublisher<SQTestEvent> * )provider.getPublisher())->_publicationStats._totalDiscardedEvents;
    ASSERT_EQ(totalMessages, 300);
}

TEST(BasicEventEmitterTest, TransformForwaringTestNoPool) {
    SQTestEvent::emptyConstructorInvokations = 0;
    SQTestEvent::constructorInvokations = 0;
    SQTestEvent::copyInvokations = 0;

    SQTestNewEvent::emptyConstructorInvokations = 0;
    SQTestNewEvent::constructorInvokations = 0;
    SQTestNewEvent::copyInvokations = 0;

    kpsr::mem::BasicMiddlewareProvider<SQTestEvent> provider(nullptr, "event", 4, 0, nullptr, nullptr, false);
    kpsr::mem::BasicMiddlewareProvider<SQTestNewEvent> newProvider(nullptr, "newEvent", 4, 0, nullptr, nullptr, false);

    provider.start();
    newProvider.start();

    std::function<void(const SQTestEvent &, SQTestNewEvent &)> transformFunction = [] (const SQTestEvent & src, SQTestNewEvent & dest) {
        dest._label = src._message;
        dest._values = { (double) src._id };
    };

    auto forwarder = newProvider.getProcessForwarder(transformFunction);
    provider.getSubscriber()->registerListener("forwarderListener", forwarder->forwarderListenerFunction);

    kpsr::mem::TestCacheListener<SQTestNewEvent> eventListener(-1);
    newProvider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    std::thread t2([&provider, &newProvider]{
        for (int i = 0; i < 10; i++) {
            SQTestEvent event1(i, "hello");
            provider.getPublisher()->publish(event1);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    });

    t2.join();
    provider.stop();
    newProvider.stop();

    auto lastReceivedEvent = eventListener.getLastReceivedEvent();
    ASSERT_NE(lastReceivedEvent.get(), nullptr);
    ASSERT_EQ(9, eventListener.getLastReceivedEvent()->_values[0]);
    ASSERT_EQ("hello", eventListener.getLastReceivedEvent()->_label);

    ASSERT_EQ(eventListener.counter, 10);
    ASSERT_EQ(SQTestEvent::emptyConstructorInvokations, 0);
    ASSERT_EQ(SQTestEvent::constructorInvokations, 10);
    ASSERT_EQ(SQTestEvent::copyInvokations, 10);

    ASSERT_EQ(SQTestNewEvent::emptyConstructorInvokations, 10);
    ASSERT_EQ(SQTestNewEvent::constructorInvokations, 0);
    ASSERT_EQ(SQTestNewEvent::copyInvokations, 10);
    ASSERT_EQ(newProvider.getSubscriber()->getSubscriptionStats("cacheListener")->_totalProcessed, 10);
}

TEST(BasicEventEmitterTest, TransformForwaringTestWithPool) {
    SQTestEvent::emptyConstructorInvokations = 0;
    SQTestEvent::constructorInvokations = 0;
    SQTestEvent::copyInvokations = 0;

    SQTestNewEvent::emptyConstructorInvokations = 0;
    SQTestNewEvent::constructorInvokations = 0;
    SQTestNewEvent::copyInvokations = 0;

    kpsr::mem::BasicMiddlewareProvider<SQTestEvent> provider(nullptr, "event", 4, 4, nullptr, nullptr, false);
    kpsr::mem::BasicMiddlewareProvider<SQTestNewEvent> newProvider(nullptr, "newEvent", 4, 4, nullptr, nullptr, false);

    provider.start();
    newProvider.start();

    std::function<void(const SQTestEvent &, SQTestNewEvent &)> transformFunction = [] (const SQTestEvent & src, SQTestNewEvent & dest) {
        dest._label = src._message;
        dest._values = { (double) src._id };
    };

    auto forwarder = newProvider.getProcessForwarder(transformFunction);
    provider.getSubscriber()->registerListener("forwarderListener", forwarder->forwarderListenerFunction);

    kpsr::mem::TestCacheListener<SQTestNewEvent> eventListener(-1);
    newProvider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    std::thread t2([&provider, &newProvider]{
        for (int i = 0; i < 10; i++) {
            SQTestEvent event1(i, "hello");
            provider.getPublisher()->publish(event1);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    });

    t2.join();
    provider.stop();
    newProvider.stop();

    auto lastReceivedEvent = eventListener.getLastReceivedEvent();
    ASSERT_NE(lastReceivedEvent.get(), nullptr);
    ASSERT_EQ(9, eventListener.getLastReceivedEvent()->_values[0]);
    ASSERT_EQ("hello", eventListener.getLastReceivedEvent()->_label);

    ASSERT_EQ(eventListener.counter, 10);
    ASSERT_EQ(SQTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(SQTestEvent::constructorInvokations, 10);
    ASSERT_EQ(SQTestEvent::copyInvokations, 0);

    ASSERT_EQ(SQTestNewEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(SQTestNewEvent::constructorInvokations, 0);
    ASSERT_EQ(SQTestNewEvent::copyInvokations, 10);

    ASSERT_EQ(newProvider.getSubscriber()->getSubscriptionStats("cacheListener")->_totalProcessed, 10);
}
