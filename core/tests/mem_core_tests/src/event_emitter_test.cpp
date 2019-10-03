/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************************/

#include <stdio.h>
#include <thread>
#include <unistd.h>
#include <math.h>

#include <sstream>
#include <fstream>

#include "gtest/gtest.h"

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

    std::cout << "total forwarding time: " << provider.getSubscriber()->getSubscriptionStats("forwarderListener")->_totalProcessingTimeInNanoSecs << std::endl;
}

