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

#include "gtest/gtest.h"

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <klepsydra/core/cache_listener.h>
#include <klepsydra/core/event_emitter_middleware_provider.h>

class EETestEvent
{
public:
    static int constructorInvokations;
    static int copyInvokations;

    EETestEvent(int id, const std::string &message)
        : _id(id)
        , _message(message)
    {
        EETestEvent::constructorInvokations++;
    }

    EETestEvent() { EETestEvent::constructorInvokations++; }

    int _id;
    std::string _message;
};

class EETestNewEvent
{
public:
    EETestNewEvent(const std::string &label, std::vector<double> values)
        : _label(label)
        , _values(values)
    {}

    EETestNewEvent() {}

    std::string _label;
    std::vector<double> _values;
};

int EETestEvent::constructorInvokations = 0;
int EETestEvent::copyInvokations = 0;

TEST(EventEmitterTest, SingleEventEmitterTopic)
{
    kpsr::EventEmitterMiddlewareProvider<EETestEvent> provider(nullptr, "event", 0, nullptr, nullptr);

    EETestEvent::constructorInvokations = 0;
    EETestEvent::copyInvokations = 0;
    std::shared_ptr<EETestEvent> event = std::make_shared<EETestEvent>(1, "hola");
    ASSERT_EQ(EETestEvent::constructorInvokations, 1);

    kpsr::mem::CacheListener<EETestEvent> eventListener;

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);
    ASSERT_EQ(EETestEvent::constructorInvokations, 1);

    provider.getPublisher()->publish(event);
    ASSERT_EQ(EETestEvent::constructorInvokations, 1);

    ASSERT_EQ(event->_id, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ(event->_message, eventListener.getLastReceivedEvent()->_message);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->totalProcessed, 1);
    ASSERT_NO_THROW(provider.getSubscriber()->removeListener("cacheListener"));

    std::shared_ptr<EETestEvent> event2 = std::make_shared<EETestEvent>(2, "hola2");
    ASSERT_NO_THROW(
        provider.getSubscriber()->registerListenerOnce(eventListener.cacheListenerFunction));
    provider.getPublisher()->publish(event);
    ASSERT_NO_THROW(provider.getPublisher()->publish(event));
    ASSERT_EQ(event->_id, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ(event->_message, eventListener.getLastReceivedEvent()->_message);
}

TEST(EventEmitterTest, SingleEventEmitterTopicWithSharePtrListener)
{
    kpsr::EventEmitterMiddlewareProvider<EETestEvent> provider(nullptr, "event", 0, nullptr, nullptr);

    EETestEvent::constructorInvokations = 0;
    EETestEvent::copyInvokations = 0;
    std::shared_ptr<EETestEvent> event = std::make_shared<EETestEvent>(1, "hola");
    ASSERT_EQ(EETestEvent::constructorInvokations, 1);

    kpsr::mem::CacheListener<std::shared_ptr<const EETestEvent>> eventListener;

    provider.getSubscriber()->registerSharedPtrListener("cacheListener",
                                                        eventListener.cacheListenerFunction);
    ASSERT_EQ(EETestEvent::constructorInvokations, 1);

    provider.getPublisher()->publish(event);
    ASSERT_EQ(EETestEvent::constructorInvokations, 1);

    ASSERT_EQ(event->_id, eventListener.getLastReceivedEvent()->get()->_id);
    ASSERT_EQ(event->_message, eventListener.getLastReceivedEvent()->get()->_message);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->totalProcessed, 1);
    ASSERT_NO_THROW(provider.getSubscriber()->removeListener("cacheListener"));

    std::shared_ptr<EETestEvent> event2 = std::make_shared<EETestEvent>(2, "hola2");
    ASSERT_NO_THROW(provider.getSubscriber()->registerSharedPtrListenerOnce(
        eventListener.cacheListenerFunction));
    provider.getPublisher()->publish(event);
    ASSERT_NO_THROW(provider.getPublisher()->publish(event));
    ASSERT_EQ(event->_id, eventListener.getLastReceivedEvent()->get()->_id);
    ASSERT_EQ(event->_message, eventListener.getLastReceivedEvent()->get()->_message);
}

TEST(EventEmitterTest, TwoEventEmitterTopics)
{
    kpsr::EventEmitterMiddlewareProvider<EETestEvent> provider(nullptr, "event", 0, nullptr, nullptr);
    kpsr::EventEmitterMiddlewareProvider<EETestNewEvent> newProvider(nullptr,
                                                                     "newEvent",
                                                                     0,
                                                                     nullptr,
                                                                     nullptr);

    EETestEvent::constructorInvokations = 0;

    kpsr::mem::CacheListener<EETestEvent> eventListener;
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    kpsr::mem::CacheListener<EETestNewEvent> newEventListener;
    newProvider.getSubscriber()->registerListener("cacheListener2",
                                                  newEventListener.cacheListenerFunction);

    std::shared_ptr<EETestEvent> event = std::make_shared<EETestEvent>(1, "hola");
    provider.getPublisher()->publish(event);
    ASSERT_EQ(event->_id, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ(event->_message, eventListener.getLastReceivedEvent()->_message);

    ASSERT_EQ(EETestEvent::constructorInvokations, 1);

    std::vector<double> values = {7, 5, 16, 8};
    std::shared_ptr<EETestNewEvent> newEvent = std::make_shared<EETestNewEvent>("adios", values);
    newProvider.getPublisher()->publish(newEvent);
    ASSERT_EQ(newEvent->_label, newEventListener.getLastReceivedEvent()->_label);
    ASSERT_EQ(newEvent->_values, newEventListener.getLastReceivedEvent()->_values);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->totalProcessed, 1);
    ASSERT_EQ(newProvider.getSubscriber()->getSubscriptionStats("cacheListener2")->totalProcessed,
              1);
}

TEST(EventEmitterTest, ContainerTest)
{
    kpsr::Container testContainer(nullptr, "testContainer");
    kpsr::EventEmitterMiddlewareProvider<EETestEvent> provider(&testContainer,
                                                               "event",
                                                               0,
                                                               nullptr,
                                                               nullptr);
    kpsr::mem::CacheListener<EETestEvent> eventListener;

    ASSERT_NO_THROW(provider.getSubscriber()->registerListener("cacheListener",
                                                               eventListener.cacheListenerFunction));
    std::shared_ptr<EETestEvent> event = std::make_shared<EETestEvent>(1, "hola");
    ASSERT_NO_THROW(provider.getPublisher()->publish(event));
    ASSERT_EQ(event->_id, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ(event->_message, eventListener.getLastReceivedEvent()->_message);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->totalProcessed, 1);
    ASSERT_NO_THROW(provider.getSubscriber()->removeListener("cacheListener"));
}

TEST(EventEmitterTest, TransformForwaringPerformanceTest)
{
    kpsr::EventEmitterMiddlewareProvider<EETestEvent> provider(nullptr, "event", 0, nullptr, nullptr);
    kpsr::EventEmitterMiddlewareProvider<EETestNewEvent> newProvider(nullptr,
                                                                     "newEvent",
                                                                     0,
                                                                     nullptr,
                                                                     nullptr);

    std::function<void(const EETestEvent &, EETestNewEvent &)> transformFunction =
        [](const EETestEvent &src, EETestNewEvent &dest) {
            dest._label = src._message;
            dest._values = {(double) src._id};
        };

    auto forwarder = newProvider.getProcessForwarder(transformFunction);
    provider.getSubscriber()->registerListener("forwarderListener",
                                               forwarder->forwarderListenerFunction);

    kpsr::mem::CacheListener<EETestNewEvent> eventListener;
    newProvider.getSubscriber()->registerListener("cacheListener",
                                                  eventListener.cacheListenerFunction);

    for (int i = 0; i < 1000000; i++) {
        std::shared_ptr<EETestEvent> event1 = std::make_shared<EETestEvent>(i, "hello");
        provider.getPublisher()->publish(event1);
    }

    spdlog::info("total forwarding time: {}",
                 provider.getSubscriber()
                     ->getSubscriptionStats("forwarderListener")
                     ->totalProcessingTimeInNanoSecs);
}

TEST(EventEmitterDeathTest, noSegmentationFault)
{
    kpsr::EventEmitterMiddlewareProvider<EETestEvent> provider(nullptr, "event", 0, nullptr, nullptr);

    auto subscriber = provider.getSubscriber();
    subscriber->registerListener("stats_test", [&](const EETestEvent &event) {
        subscriber->removeListener("stats_test");
    });

    std::shared_ptr<EETestEvent> event1 = std::make_shared<EETestEvent>(0, "hello");
    ASSERT_EXIT((provider.getPublisher()->publish(event1), exit(0)),
                ::testing::ExitedWithCode(0),
                ".*");
}

TEST(EventEmitterTest, listenerStatsNullAtEnd)
{
    kpsr::EventEmitterMiddlewareProvider<EETestEvent> provider(nullptr, "event", 0, nullptr, nullptr);

    auto subscriber = provider.getSubscriber();

    subscriber->registerListener("stats_test", [&](const EETestEvent &event) {
        subscriber->removeListener("stats_test");
    });

    std::shared_ptr<EETestEvent> event1 = std::make_shared<EETestEvent>(0, "hello");
    provider.getPublisher()->publish(event1);
    // Ensure that listener stats are removed AFTER the publish action has completely terminated.
    ASSERT_EQ(subscriber->getSubscriptionStats("stats_test"), nullptr);
}

TEST(EventEmitterTest, NullListener)
{
    kpsr::EventEmitterMiddlewareProvider<EETestEvent> provider(nullptr, "event", 0, nullptr, nullptr);

    auto subscriber = provider.getSubscriber();
    ASSERT_ANY_THROW(subscriber->registerListener("null_test", nullptr));
}

TEST(EventEmitterTest, NullListenerNoArgs)
{
    kpsr::SafeEventEmitter<std::string> emitter;

    std::function<void(const std::string &)> cb = nullptr;
    ASSERT_NO_THROW(emitter.on(nullptr, "null_test", "null_test", cb));
    ASSERT_NO_THROW(emitter.once("null_test", cb));
}

TEST(EventEmitterTest, ListenerOnce)
{
    kpsr::SafeEventEmitter<EETestEvent> emitter;

    EETestEvent event(1, "hola");

    kpsr::mem::CacheListener<EETestEvent> eventListener;

    unsigned int id;
    std::string event_id = "cacheListener";
    ASSERT_NO_THROW(id = emitter.once(event_id, eventListener.cacheListenerFunction));
    ASSERT_EQ(id, 1);
    ASSERT_NO_THROW(emitter.emitEvent(event_id, 0, event));
    ASSERT_ANY_THROW(emitter.removeListener(nullptr, id));

    ASSERT_NO_THROW(id = emitter.once("", eventListener.cacheListenerFunction));
    ASSERT_NO_THROW(emitter.emitEvent("", 0, event));
    ASSERT_NO_THROW(id = emitter.once("", nullptr));
    ASSERT_ANY_THROW(emitter.emitEvent("", 0, event));
}

TEST(EventEmitterTest, TestProcessPublishPool)
{
    int poolSize = 1;
    kpsr::EventEmitterMiddlewareProvider<EETestEvent> provider(nullptr,
                                                               "event",
                                                               poolSize,
                                                               nullptr,
                                                               nullptr);

    EETestEvent::constructorInvokations = 0;
    EETestEvent::copyInvokations = 0;
    kpsr::mem::TestCacheListener<EETestEvent> eventListener(
        0); // add sleep to make cacheListener slow.

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    for (int i = 0; i < 100; i++) {
        provider.getPublisher()->processAndPublish([i](EETestEvent &event) {
            event._id = i;
            event._message = "Processed message " + std::to_string(i);
        });
    }
    ASSERT_EQ(provider.getPublisher()->_publicationStats._totalEventAllocations, poolSize);
    ASSERT_EQ(eventListener.getLastReceivedEvent()->_id, 99);
    ASSERT_EQ(eventListener.getLastReceivedEvent()->_message,
              "Processed message " + std::to_string(99));
    provider.getSubscriber()->removeListener("cacheListener");
}

TEST(EventEmitterTest, TestProcessPublishNoPool)
{
    kpsr::EventEmitterMiddlewareProvider<EETestEvent> provider(nullptr, "event", 0, nullptr, nullptr);

    EETestEvent::constructorInvokations = 0;
    EETestEvent::copyInvokations = 0;
    kpsr::mem::TestCacheListener<EETestEvent> eventListener(
        0); // add sleep to make cacheListener slow.

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    EETestEvent event(1, "hola");

    int iterations = 100;
    for (int i = 0; i < iterations; i++) {
        provider.getPublisher()->processAndPublish([i](EETestEvent &event) {
            event._id = i;
            event._message = "Processed message " + std::to_string(i);
        });
    }
    ASSERT_GT(provider.getPublisher()->_publicationStats._totalEventAllocations, 0);
    ASSERT_EQ(provider.getPublisher()->_publicationStats._totalEventAllocations, iterations);
    ASSERT_EQ(eventListener.getLastReceivedEvent()->_id, iterations - 1);
    ASSERT_EQ(eventListener.getLastReceivedEvent()->_message,
              "Processed message " + std::to_string(iterations - 1));
    provider.getSubscriber()->removeListener("cacheListener");
}

TEST(EventEmitterTest, EventClonerPool)
{
    int poolSize = 1;
    kpsr::EventEmitterMiddlewareProvider<EETestEvent> provider(
        nullptr, "event", poolSize, nullptr, [](const EETestEvent &original, EETestEvent &cloned) {
            cloned._id = original._id;
            cloned._message = original._message + ":Cloned";
        });

    EETestEvent::constructorInvokations = 0;
    EETestEvent::copyInvokations = 0;
    kpsr::mem::TestCacheListener<EETestEvent> eventListener(
        0); // add sleep to make cacheListener slow.
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    EETestEvent eventOriginal(10, "Hello");
    provider.getPublisher()->publish(eventOriginal);
    ASSERT_EQ(provider.getPublisher()->_publicationStats._totalEventAllocations, poolSize);
    ASSERT_EQ(eventListener.getLastReceivedEvent()->_message, eventOriginal._message + ":Cloned");
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);
}

TEST(EventEmitterTest, EventClonerNoPool)
{
    int poolSize = 0;
    kpsr::EventEmitterMiddlewareProvider<EETestEvent> provider(
        nullptr, "event", poolSize, nullptr, [](const EETestEvent &original, EETestEvent &cloned) {
            cloned._id = original._id;
            cloned._message = original._message + ":Cloned";
        });

    EETestEvent::constructorInvokations = 0;
    EETestEvent::copyInvokations = 0;
    kpsr::mem::TestCacheListener<EETestEvent> eventListener(
        0); // add sleep to make cacheListener slow.
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    EETestEvent eventOriginal(10, "Hello");
    provider.getPublisher()->publish(eventOriginal);
    ASSERT_GT(provider.getPublisher()->_publicationStats._totalEventAllocations, poolSize);
    ASSERT_EQ(eventListener.getLastReceivedEvent()->_message, eventOriginal._message + ":Cloned");
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);
}

TEST(EventEmitterTest, InitializerPool)
{
    int poolSize = 1;
    kpsr::EventEmitterMiddlewareProvider<EETestEvent> provider(
        nullptr,
        "event",
        poolSize,
        [](EETestEvent &event) {
            event._id = 99;
            event._message = "Initialized:";
        },
        [](const EETestEvent &original, EETestEvent &cloned) {
            cloned._id = original._id;
            cloned._message = cloned._message + original._message + ":Cloned";
        });

    EETestEvent::constructorInvokations = 0;
    EETestEvent::copyInvokations = 0;
    kpsr::mem::TestCacheListener<EETestEvent> eventListener(
        0); // add sleep to make cacheListener slow.
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    EETestEvent eventOriginal(10, "Hello");
    provider.getPublisher()->publish(eventOriginal);
    ASSERT_EQ(provider.getPublisher()->_publicationStats._totalEventAllocations, poolSize);
    ASSERT_EQ(eventListener.getLastReceivedEvent()->_message,
              "Initialized:" + eventOriginal._message + ":Cloned");
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);
}

TEST(EventEmitterTest, InitializerNoPool)
{
    int poolSize = 0;
    kpsr::EventEmitterMiddlewareProvider<EETestEvent> provider(
        nullptr,
        "event",
        poolSize,
        [](EETestEvent &event) {
            event._id = 99;
            event._message = "Initialized:";
        },
        [](const EETestEvent &original, EETestEvent &cloned) {
            cloned._id = original._id;
            cloned._message = cloned._message + original._message + ":Cloned";
        });

    EETestEvent::constructorInvokations = 0;
    EETestEvent::copyInvokations = 0;
    kpsr::mem::TestCacheListener<EETestEvent> eventListener(
        0); // add sleep to make cacheListener slow.
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    EETestEvent eventOriginal(10, "Hello");
    provider.getPublisher()->publish(eventOriginal);
    ASSERT_GT(provider.getPublisher()->_publicationStats._totalEventAllocations, poolSize);
    ASSERT_EQ(eventListener.getLastReceivedEvent()->_message,
              "Initialized:" + eventOriginal._message + ":Cloned");
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);
}
