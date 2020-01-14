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

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <klepsydra/core/event_emitter_middleware_provider.h>
#include <klepsydra/core/cache_listener.h>

class EETestEvent {
public:

    static int constructorInvokations;
    static int copyInvokations;

    EETestEvent(int id, std::string message)
        : _id(id)
        , _message(message) {
        EETestEvent::constructorInvokations++;
    }

    EETestEvent() {
        EETestEvent::constructorInvokations++;
    }

    int _id;
    std::string _message;
};

class EETestNewEvent {
public:

    EETestNewEvent(std::string label, std::vector<double> values)
        : _label(label)
        , _values(values) {
    }

    EETestNewEvent() {
    }

    std::string _label;
    std::vector<double> _values;
};

int EETestEvent::constructorInvokations = 0;
int EETestEvent::copyInvokations = 0;

TEST(EventEmitterTest, SingleEventEmitterTopic) {
    kpsr::EventEmitterMiddlewareProvider<EETestEvent> provider(nullptr, "event", 0, nullptr, nullptr);

    EETestEvent::constructorInvokations = 0;
    EETestEvent::copyInvokations = 0;
    EETestEvent event(1, "hola");
    ASSERT_EQ(EETestEvent::constructorInvokations, 1);

    kpsr::mem::CacheListener<EETestEvent> eventListener;

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);
    ASSERT_EQ(EETestEvent::constructorInvokations, 1);

    provider.getPublisher()->publish(event);
    ASSERT_EQ(EETestEvent::constructorInvokations, 1);

    ASSERT_EQ(event._id, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ(event._message, eventListener.getLastReceivedEvent()->_message);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->_totalProcessed, 1);
}

TEST(EventEmitterTest, TwoEventEmitterTopics) {
    kpsr::EventEmitterMiddlewareProvider<EETestEvent> provider(nullptr, "event", 0, nullptr, nullptr);
    kpsr::EventEmitterMiddlewareProvider<EETestNewEvent> newProvider(nullptr, "newEvent", 0, nullptr, nullptr);

    EETestEvent::constructorInvokations = 0;

    kpsr::mem::CacheListener<EETestEvent> eventListener;
    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    kpsr::mem::CacheListener<EETestNewEvent> newEventListener;
    newProvider.getSubscriber()->registerListener("cacheListener2", newEventListener.cacheListenerFunction);

    EETestEvent event(1, "hola");
    provider.getPublisher()->publish(event);
    ASSERT_EQ(event._id, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ(event._message, eventListener.getLastReceivedEvent()->_message);

    ASSERT_EQ(EETestEvent::constructorInvokations, 1);

    EETestNewEvent newEvent("adios", {7, 5, 16, 8});
    newProvider.getPublisher()->publish(newEvent);
    ASSERT_EQ(newEvent._label, newEventListener.getLastReceivedEvent()->_label);
    ASSERT_EQ(newEvent._values, newEventListener.getLastReceivedEvent()->_values);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->_totalProcessed, 1);
    ASSERT_EQ(newProvider.getSubscriber()->getSubscriptionStats("cacheListener2")->_totalProcessed, 1);
}

TEST(EventEmitterTest, TransformForwaringPerformanceTest) {
    kpsr::EventEmitterMiddlewareProvider<EETestEvent> provider(nullptr, "event", 0, nullptr, nullptr);
    kpsr::EventEmitterMiddlewareProvider<EETestNewEvent> newProvider(nullptr, "newEvent", 0, nullptr, nullptr);

    std::function<void(const EETestEvent &, EETestNewEvent &)> transformFunction = [] (const EETestEvent & src, EETestNewEvent & dest) {
        dest._label = src._message;
        dest._values = { (double) src._id };
    };

    auto forwarder = newProvider.getProcessForwarder(transformFunction);
    provider.getSubscriber()->registerListener("forwarderListener", forwarder->forwarderListenerFunction);

    kpsr::mem::CacheListener<EETestNewEvent> eventListener;
    newProvider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    for (int i = 0; i < 1000000; i++) {
        EETestEvent event1(i, "hello");
        provider.getPublisher()->publish(event1);
    }

    spdlog::info("total forwarding time: {}", provider.getSubscriber()->getSubscriptionStats("forwarderListener")->_totalProcessingTimeInNanoSecs);
}

TEST(EventEmitterDeathTest, noSegmentationFault) {
    kpsr::EventEmitterMiddlewareProvider<EETestEvent> provider(nullptr, "event", 0, nullptr, nullptr);

    auto subscriber = provider.getSubscriber();
    subscriber->registerListener("stats_test", [&] (const EETestEvent & event) {
                                                   subscriber->removeListener("stats_test");});

    EETestEvent event1(0, "hello");
    ASSERT_EXIT((provider.getPublisher()->publish(event1), exit(0)),::testing::ExitedWithCode(0),".*");
}

TEST(EventEmitterTest, listenerStatsNullAtEnd) {
    kpsr::EventEmitterMiddlewareProvider<EETestEvent> provider(nullptr, "event", 0, nullptr, nullptr);

    auto subscriber = provider.getSubscriber();

    subscriber->registerListener("stats_test", [&] (const EETestEvent & event) {
                                                   subscriber->removeListener("stats_test");});

    EETestEvent event1(0, "hello");
    provider.getPublisher()->publish(event1);
    // Ensure that listener stats are removed AFTER the publish action has completely terminated.
    ASSERT_EQ(subscriber->getSubscriptionStats("stats_test"), nullptr);

}
