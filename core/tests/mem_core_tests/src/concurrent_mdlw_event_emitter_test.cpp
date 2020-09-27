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

#include <klepsydra/mem_core/concurrent_middleware_provider.h>
#include <klepsydra/core/cache_listener.h>

class ConcurrentSQTestEvent {
public:

    static std::atomic_int constructorInvokations;
    static std::atomic_int emptyConstructorInvokations;
    static std::atomic_int copyInvokations;

    ConcurrentSQTestEvent(int id, const std::string & message)
        : _id(id)
        , _message(message) {
        ConcurrentSQTestEvent::constructorInvokations++;
    }

    ConcurrentSQTestEvent() {
        ConcurrentSQTestEvent::emptyConstructorInvokations++;
    }

    ConcurrentSQTestEvent(const ConcurrentSQTestEvent & that)
        : _id(that._id)
        , _message(that._message) {
        ConcurrentSQTestEvent::copyInvokations++;
    }

    int _id;
    std::string _message;
};

class ConcurrentSQTestNewEvent {
public:

    static std::atomic_int constructorInvokations;
    static std::atomic_int emptyConstructorInvokations;
    static std::atomic_int copyInvokations;

    ConcurrentSQTestNewEvent(const std::string & label, std::vector<double> values)
        : _label(label)
        , _values(values) {
        ConcurrentSQTestNewEvent::constructorInvokations++;
    }

    ConcurrentSQTestNewEvent() {
        ConcurrentSQTestNewEvent::emptyConstructorInvokations++;
    }

    ConcurrentSQTestNewEvent(const ConcurrentSQTestNewEvent & that)
        : _label(that._label)
        , _values(that._values) {
        ConcurrentSQTestNewEvent::copyInvokations++;
    }

    std::string _label;
    std::vector<double> _values;
};

std::atomic_int ConcurrentSQTestEvent::constructorInvokations(0);
std::atomic_int ConcurrentSQTestEvent::emptyConstructorInvokations(0);
std::atomic_int ConcurrentSQTestEvent::copyInvokations(0);

std::atomic_int ConcurrentSQTestNewEvent::constructorInvokations(0);
std::atomic_int ConcurrentSQTestNewEvent::emptyConstructorInvokations(0);
std::atomic_int ConcurrentSQTestNewEvent::copyInvokations(0);

TEST(ConcurrentEventEmitterTest, basicQueueTest) {

    moodycamel::ConcurrentQueue <kpsr::mem::EventData<const int> > testQueue(10);

    int event1(1);
    kpsr::mem::EventData<const int> enqEvent;
    enqEvent.eventData = std::shared_ptr<const int>(new int(event1));

    EXPECT_TRUE(testQueue.try_enqueue(enqEvent));
    std::thread t([&testQueue, enqEvent]() {
                      // This should fail since new thread is assigned different block of memory by concurrent queue.
                      EXPECT_FALSE(testQueue.try_enqueue(enqEvent));
    });

    t.join();
}

TEST(ConcurrentEventEmitterTest, basicQueueTestWithToken) {

    moodycamel::ConcurrentQueue <kpsr::mem::EventData<const int> > testQueue(10);
    moodycamel::ProducerToken token(testQueue);
    int event1(1);
    kpsr::mem::EventData<const int> enqEvent;
    enqEvent.eventData = std::shared_ptr<const int>(new int(event1));

    EXPECT_TRUE(testQueue.try_enqueue(token, enqEvent));
    std::thread t([&testQueue, &enqEvent, &token]() {
                      // This should not fail since previous token is
                      // reused and no new block of memory is needed
                      // by concurrent queue.
                      EXPECT_TRUE(testQueue.try_enqueue(token, enqEvent));
    });

    t.join();
}

TEST(ConcurrentEventEmitterTest, basicQueueTestLargeSize) {

    moodycamel::ConcurrentQueue <kpsr::mem::EventData<const int> > testQueue(100);

    int event1(1);
    kpsr::mem::EventData<const int> enqEvent;
    enqEvent.eventData = std::shared_ptr<const int>(new int(event1));

    EXPECT_TRUE(testQueue.try_enqueue(enqEvent));
    std::thread t([&testQueue, enqEvent]() {
                      // This should not fail since new thread is
                      // assigned different block of memory by
                      // concurrent queue and the testQueue has enough
                      // capacity for at least 3 blocks.
                      EXPECT_TRUE(testQueue.try_enqueue(enqEvent));
    });

    t.join();
}

TEST(ConcurrentEventEmitterTest, SingleEventEmitterTopic) {
    ConcurrentSQTestEvent::emptyConstructorInvokations = 0;
    ConcurrentSQTestEvent::constructorInvokations = 0;
    ConcurrentSQTestEvent::copyInvokations = 0;


    kpsr::mem::ConcurrentMiddlewareProvider<ConcurrentSQTestEvent> provider(nullptr, "event", 4, 0, nullptr, nullptr, false, 1000);
    provider.start();
    kpsr::mem::TestCacheListener<ConcurrentSQTestEvent> eventListener(-1);
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    std::thread t2([&provider]{
        ConcurrentSQTestEvent event1(1, "hello");
        provider.getPublisher()->publish(event1);
        ConcurrentSQTestEvent event2(2, "hallo");
        provider.getPublisher()->publish(event2);
        event2._id = 3;
        event2._message = "hola";
        provider.getPublisher()->publish(event2);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        provider.stop();
    });

    t2.join();

    ASSERT_EQ(3, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hola", eventListener.getLastReceivedEvent()->_message);

    ASSERT_EQ(eventListener.counter, 3);
    ASSERT_EQ(ConcurrentSQTestEvent::emptyConstructorInvokations,0);
    ASSERT_EQ(ConcurrentSQTestEvent::constructorInvokations, 2);
    ASSERT_EQ(ConcurrentSQTestEvent::copyInvokations, 6);
}

TEST(ConcurrentEventEmitterTest, WithObjectPoolNoFailures) {
    ConcurrentSQTestEvent::emptyConstructorInvokations = 0;
    ConcurrentSQTestEvent::constructorInvokations = 0;
    ConcurrentSQTestEvent::copyInvokations = 0;

    const int poolSize = 4;
    const int queueSize = 4;
    const int numPublish = 10;
    kpsr::mem::ConcurrentMiddlewareProvider<ConcurrentSQTestEvent> provider(nullptr, "event", queueSize, poolSize, nullptr, nullptr, false, 1000);
    ASSERT_EQ(ConcurrentSQTestEvent::emptyConstructorInvokations, poolSize);

    provider.start();

    kpsr::mem::TestCacheListener<ConcurrentSQTestEvent> eventListener(-1);
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    std::thread t2([&provider]{
        for (int i = 0; i < numPublish; i++) {
            ConcurrentSQTestEvent event1(i, "hello");
            provider.getPublisher()->publish(event1);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        provider.stop();
    });
    
    t2.join();

    ASSERT_EQ(numPublish - 1, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hello", eventListener.getLastReceivedEvent()->_message);

    ASSERT_EQ(eventListener.counter, numPublish);
    ASSERT_EQ(ConcurrentSQTestEvent::emptyConstructorInvokations, poolSize);
    ASSERT_EQ(ConcurrentSQTestEvent::constructorInvokations, numPublish);
    ASSERT_EQ(ConcurrentSQTestEvent::copyInvokations, numPublish);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->_totalProcessed, numPublish);
}

TEST(ConcurrentEventEmitterTest, WithObjectPoolWithFailuresBlocking) {
    ConcurrentSQTestEvent::emptyConstructorInvokations = 0;
    ConcurrentSQTestEvent::constructorInvokations = 0;
    ConcurrentSQTestEvent::copyInvokations = 0;

    const int poolSize = 7;
    const int queueSize = 4;
    kpsr::mem::ConcurrentMiddlewareProvider<ConcurrentSQTestEvent> provider(nullptr, "event", queueSize, poolSize, nullptr, nullptr, false, 1000);
    ASSERT_EQ(ConcurrentSQTestEvent::emptyConstructorInvokations, poolSize);

    provider.start();
    kpsr::mem::TestCacheListener<ConcurrentSQTestEvent> eventListener(2);
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    const int numPublish = 300;
    std::thread t2([&provider]{
        for (int i = 0; i < numPublish; i++) {
            ConcurrentSQTestEvent event1(i, "hello");
            provider.getPublisher()->publish(event1);
            // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        while (!(provider._internalQueue.size_approx()==0)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        provider.stop();
    });

    t2.join();

    ASSERT_EQ(numPublish - 1, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hello", eventListener.getLastReceivedEvent()->_message);

    ASSERT_EQ(eventListener.counter, numPublish);
    ASSERT_EQ(ConcurrentSQTestEvent::emptyConstructorInvokations, poolSize);
    ASSERT_EQ(ConcurrentSQTestEvent::constructorInvokations, numPublish);
    ASSERT_GE(ConcurrentSQTestEvent::copyInvokations, numPublish);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->_totalProcessed, numPublish);
}

TEST(ConcurrentEventEmitterTest, WithObjectPoolWithFailuresNonBlocking) {
    ConcurrentSQTestEvent::emptyConstructorInvokations = 0;
    ConcurrentSQTestEvent::constructorInvokations = 0;
    ConcurrentSQTestEvent::copyInvokations = 0;
    const int poolSize = 7;
    const int queueSize = 4;
    kpsr::mem::ConcurrentMiddlewareProvider<ConcurrentSQTestEvent> provider(nullptr, "event", queueSize, poolSize, nullptr, nullptr, true, 1000);
    ASSERT_EQ(ConcurrentSQTestEvent::emptyConstructorInvokations, poolSize);

    provider.start();

    kpsr::mem::TestCacheListener<ConcurrentSQTestEvent> eventListener(10);
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    const int numPublish = 300;
    std::thread t2([&provider]{
        for (int i = 0; i < numPublish; i++) {
            ConcurrentSQTestEvent event1(i, "hello");
            provider.getPublisher()->publish(event1);
        }
        while (!(provider._internalQueue.size_approx()==0)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        provider.stop();
    });

    t2.join();

    ASSERT_EQ(numPublish - 1, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hello", eventListener.getLastReceivedEvent()->_message);

    ASSERT_GE(eventListener.counter, queueSize);
    ASSERT_EQ(ConcurrentSQTestEvent::emptyConstructorInvokations, poolSize);
    ASSERT_EQ(ConcurrentSQTestEvent::constructorInvokations, numPublish);
    ASSERT_GE(ConcurrentSQTestEvent::copyInvokations, queueSize);
    int totalMessages = provider.getSubscriber()->getSubscriptionStats("cacheListener")->_totalProcessed
            + ((kpsr::mem::ConcurrentQueuePublisher<ConcurrentSQTestEvent> * )provider.getPublisher())->_publicationStats._totalDiscardedEvents;
    ASSERT_EQ(totalMessages, numPublish);
}

TEST(ConcurrentEventEmitterTest, TransformForwaringTestNoPool) {
    ConcurrentSQTestEvent::emptyConstructorInvokations = 0;
    ConcurrentSQTestEvent::constructorInvokations = 0;
    ConcurrentSQTestEvent::copyInvokations = 0;

    ConcurrentSQTestNewEvent::emptyConstructorInvokations = 0;
    ConcurrentSQTestNewEvent::constructorInvokations = 0;
    ConcurrentSQTestNewEvent::copyInvokations = 0;

    const int poolSize = 0;
    const int queueSize = 4;
    kpsr::mem::ConcurrentMiddlewareProvider<ConcurrentSQTestEvent> provider(nullptr, "event", queueSize, poolSize, nullptr, nullptr, false, 1000);
    kpsr::mem::ConcurrentMiddlewareProvider<ConcurrentSQTestNewEvent> newProvider(nullptr, "newEvent", queueSize, poolSize, nullptr, nullptr, false, 1000);

    provider.start();
    newProvider.start();

    std::function<void(const ConcurrentSQTestEvent &, ConcurrentSQTestNewEvent &)> transformFunction = [] (const ConcurrentSQTestEvent & src, ConcurrentSQTestNewEvent & dest) {
        dest._label = src._message;
        dest._values = { (double) src._id };
    };

    auto forwarder = newProvider.getProcessForwarder(transformFunction);
    provider.getSubscriber()->registerListener("forwarderListener", forwarder->forwarderListenerFunction);

    kpsr::mem::TestCacheListener<ConcurrentSQTestNewEvent> eventListener(-1);
    newProvider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    const int numPublish = 10;
    std::thread t2([&provider, &newProvider]{
        for (int i = 0; i < numPublish; i++) {
            ConcurrentSQTestEvent event1(i, "hello");
            provider.getPublisher()->publish(event1);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        provider.stop();
        newProvider.stop();
    });

    t2.join();

    ASSERT_EQ(numPublish - 1, eventListener.getLastReceivedEvent()->_values[0]);
    ASSERT_EQ("hello", eventListener.getLastReceivedEvent()->_label);

    ASSERT_EQ(eventListener.counter, numPublish);
    ASSERT_EQ(ConcurrentSQTestEvent::emptyConstructorInvokations, poolSize);
    ASSERT_EQ(ConcurrentSQTestEvent::constructorInvokations, numPublish);
    ASSERT_EQ(ConcurrentSQTestEvent::copyInvokations, numPublish);

    ASSERT_EQ(ConcurrentSQTestNewEvent::emptyConstructorInvokations, numPublish);
    ASSERT_EQ(ConcurrentSQTestNewEvent::constructorInvokations, poolSize);
    ASSERT_EQ(ConcurrentSQTestNewEvent::copyInvokations, numPublish);
    ASSERT_EQ(newProvider.getSubscriber()->getSubscriptionStats("cacheListener")->_totalProcessed, numPublish);
}

TEST(ConcurrentEventEmitterTest, TransformForwaringTestWithPool) {
    ConcurrentSQTestEvent::emptyConstructorInvokations = 0;
    ConcurrentSQTestEvent::constructorInvokations = 0;
    ConcurrentSQTestEvent::copyInvokations = 0;

    ConcurrentSQTestNewEvent::emptyConstructorInvokations = 0;
    ConcurrentSQTestNewEvent::constructorInvokations = 0;
    ConcurrentSQTestNewEvent::copyInvokations = 0;

    const int poolSize = 4;
    const int queueSize = 4;
    const int numPublish = 10;
    kpsr::mem::ConcurrentMiddlewareProvider<ConcurrentSQTestEvent> provider(nullptr, "event", queueSize, poolSize, nullptr, nullptr, false, 1000);
    kpsr::mem::ConcurrentMiddlewareProvider<ConcurrentSQTestNewEvent> newProvider(nullptr, "newEvent", queueSize, poolSize, nullptr, nullptr, false, 1000);

    provider.start();
    newProvider.start();

    std::function<void(const ConcurrentSQTestEvent &, ConcurrentSQTestNewEvent &)> transformFunction = [] (const ConcurrentSQTestEvent & src, ConcurrentSQTestNewEvent & dest) {
        dest._label = src._message;
        dest._values = { (double) src._id };
    };

    auto forwarder = newProvider.getProcessForwarder(transformFunction);
    provider.getSubscriber()->registerListener("forwarderListener", forwarder->forwarderListenerFunction);

    kpsr::mem::TestCacheListener<ConcurrentSQTestNewEvent> eventListener(-1);
    newProvider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    std::thread t2([&provider, &newProvider]{
        for (int i = 0; i < numPublish; i++) {
            ConcurrentSQTestEvent event1(i, "hello");
            provider.getPublisher()->publish(event1);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        provider.stop();
        newProvider.stop();
    });

    t2.join();

    ASSERT_EQ(numPublish - 1, eventListener.getLastReceivedEvent()->_values[0]);
    ASSERT_EQ("hello", eventListener.getLastReceivedEvent()->_label);

    ASSERT_EQ(eventListener.counter, numPublish);
    ASSERT_EQ(ConcurrentSQTestEvent::emptyConstructorInvokations, poolSize);
    ASSERT_EQ(ConcurrentSQTestEvent::constructorInvokations, numPublish);
    ASSERT_EQ(ConcurrentSQTestEvent::copyInvokations, 0);

    ASSERT_EQ(ConcurrentSQTestNewEvent::emptyConstructorInvokations, poolSize);
    ASSERT_EQ(ConcurrentSQTestNewEvent::constructorInvokations, 0);
    ASSERT_EQ(ConcurrentSQTestNewEvent::copyInvokations, numPublish);

    ASSERT_EQ(newProvider.getSubscriber()->getSubscriptionStats("cacheListener")->_totalProcessed, numPublish);
}
