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

#include <memory>
#include <vector>
#include <iostream>
#include <functional>

#include <klepsydra/core/smart_object_pool.h>
#include <klepsydra/core/cache_listener.h>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>

#include "gtest/gtest.h"

class SensorData {
public:
    int id;
    std::vector<double> data;

    SensorData(int id, std::vector<double> data)
        : id(id)
        , data(data) {
        std::cout << "SensorData::SensorData. Main" << std::endl;
    }

    SensorData() {
        std::cout << "SensorData::SensorData. Empty" << std::endl;
    }

    SensorData(const SensorData & that)
        : id(that.id)
        , data(that.data) {
        std::cout << "SensorData::SensorData. Copy" << std::endl;
    }
};

class ELTestEvent {
public:

    static int constructorInvokations;
    static int emptyConstructorInvokations;
    static int copyInvokations;

    ELTestEvent(int id, std::string message)
        : _id(id)
        , _message(message) {
        ELTestEvent::constructorInvokations++;
    }

    ELTestEvent(const ELTestEvent & that)
        : _id(that._id)
        , _message(that._message) {
        ELTestEvent::copyInvokations++;
    }

    ELTestEvent() {
        ELTestEvent::emptyConstructorInvokations++;
    }

    int _id;
    std::string _message;
};

class ELTestNewEvent {
public:

    ELTestNewEvent(std::string label, std::vector<double> values)
        : _label(label)
        , _values(values) {
    }

    ELTestNewEvent() {
    }

    std::string _label;
    std::vector<double> _values;
};

int ELTestEvent::constructorInvokations = 0;
int ELTestEvent::emptyConstructorInvokations = 0;
int ELTestEvent::copyInvokations = 0;

TEST(EventLoopTest, SharePointerCasting) {

    std::shared_ptr<void> ringBufferPointer;
    {
        std::shared_ptr<SensorData> sensorDataPointer(new SensorData(1, {1, 2, 3}));
        ringBufferPointer = std::static_pointer_cast<void>(sensorDataPointer);
    }
    std::shared_ptr<SensorData> newSensorDataPointer = std::static_pointer_cast<SensorData>(ringBufferPointer);

    ASSERT_EQ(1, newSensorDataPointer->id);
    ASSERT_EQ(std::vector<double>({1, 2, 3}), newSensorDataPointer->data);
}

TEST(EventLoopTest, SharePointerCastingWithPool) {

    kpsr::SmartObjectPool<SensorData> pool(4);
    for (int i = 0; i < 10; i ++) {
        std::shared_ptr<void> ringBufferPointer;
        {
            std::shared_ptr<SensorData> sensorDataPointer = std::move(pool.acquire());
            ringBufferPointer = std::static_pointer_cast<void>(sensorDataPointer);
        }
        std::shared_ptr<SensorData> newSensorDataPointer = std::static_pointer_cast<SensorData>(ringBufferPointer);
    }
}

TEST(EventLoopTest, SingleEventEmitterTopicWitOuthPool) {
    kpsr::high_performance::EventLoopMiddlewareProvider<4> provider(nullptr);
    provider.start();

    ELTestEvent::constructorInvokations = 0;
    ELTestEvent::copyInvokations = 0;
    ELTestEvent::emptyConstructorInvokations = 0;
    kpsr::mem::TestCacheListener<ELTestEvent> eventListener(-1);

    kpsr::Subscriber<ELTestEvent> * subscriber = provider.getSubscriber<ELTestEvent>("ELTestEvent");
    subscriber->registerListener("cacheListener", eventListener.cacheListenerFunction);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::copyInvokations, 0);

    kpsr::Publisher<ELTestEvent> * publisher = provider.getPublisher<ELTestEvent>("ELTestEvent", 0, nullptr, nullptr);
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
    ASSERT_EQ(subscriber->getSubscriptionStats("cacheListener")->_totalProcessed, 10);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 10);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::copyInvokations, 20);

}

TEST(EventLoopTest, SingleEventEmitterTopicWithPool) {
    kpsr::high_performance::EventLoopMiddlewareProvider<4> provider(nullptr);
    provider.start();

    ELTestEvent::constructorInvokations = 0;
    ELTestEvent::copyInvokations = 0;
    ELTestEvent::emptyConstructorInvokations = 0;
    kpsr::mem::TestCacheListener<ELTestEvent> eventListener(-1);

    kpsr::Subscriber<ELTestEvent> * subscriber = provider.getSubscriber<ELTestEvent>("ELTestEvent");
    subscriber->registerListener("cacheListener", eventListener.cacheListenerFunction);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::copyInvokations, 0);

    kpsr::Publisher<ELTestEvent> * publisher = provider.getPublisher<ELTestEvent>("ELTestEvent", 4, nullptr, nullptr);
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
    ASSERT_EQ(subscriber->getSubscriptionStats("cacheListener")->_totalProcessed, 10);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 10);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 4);
    ASSERT_EQ(ELTestEvent::copyInvokations, 12);

}

TEST(EventLoopTest, SingleEventEmitterTwoTopicsWithOutPool) {
    kpsr::high_performance::EventLoopMiddlewareProvider<4> provider(nullptr);
    provider.start();

    ELTestEvent::constructorInvokations = 0;
    ELTestEvent::copyInvokations = 0;
    ELTestEvent::emptyConstructorInvokations = 0;

    kpsr::mem::TestCacheListener<ELTestEvent> eventListener(-1);

    kpsr::Subscriber<ELTestEvent> * subscriber = provider.getSubscriber<ELTestEvent>("ELTestEvent");
    subscriber->registerListener("eventListener", eventListener.cacheListenerFunction);

    kpsr::Publisher<ELTestEvent> * publisher = provider.getPublisher<ELTestEvent>("ELTestEvent", 0, nullptr, nullptr);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::copyInvokations, 0);

    kpsr::mem::TestCacheListener<ELTestNewEvent> newEventListener(-1);

    kpsr::Subscriber<ELTestNewEvent> * newSubscriber = provider.getSubscriber<ELTestNewEvent>("ELTestNewEvent");
    newSubscriber->registerListener("newEventListener", newEventListener.cacheListenerFunction);

    kpsr::Publisher<ELTestNewEvent> * newPublisher = provider.getPublisher<ELTestNewEvent>("ELTestNewEvent", 0, nullptr, nullptr);
    std::thread t1([&publisher]{
        for (int i = 0; i < 100; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            ELTestEvent event(i, "hola");
            publisher->publish(event);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    });

    std::thread t2([&newPublisher]{
        for (int i = 0; i < 200; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            ELTestNewEvent event("hola", {(double)i});
            newPublisher->publish(event);
        }
    });


    t1.join();
    t2.join();

    while (eventListener.getLastReceivedEvent()->_id < 99) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    provider.stop();

    ASSERT_EQ(99, eventListener.getLastReceivedEvent()->_id);
    ASSERT_EQ("hola", eventListener.getLastReceivedEvent()->_message);

    ASSERT_EQ(199, newEventListener.getLastReceivedEvent()->_values[0]);

    ASSERT_EQ(subscriber->getSubscriptionStats("eventListener")->_totalProcessed, 100);
    ASSERT_EQ(newSubscriber->getSubscriptionStats("newEventListener")->_totalProcessed, 200);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 100);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::copyInvokations, 200);

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
}

TEST(EventLoopTest, SingleEventEmitterTwoTopicsWithPool) {
    kpsr::high_performance::EventLoopMiddlewareProvider<4> provider(nullptr);
    provider.start();

    ELTestEvent::constructorInvokations = 0;
    ELTestEvent::copyInvokations = 0;
    ELTestEvent::emptyConstructorInvokations = 0;

    kpsr::mem::TestCacheListener<ELTestEvent> eventListener(10);

    kpsr::Subscriber<ELTestEvent> * subscriber = provider.getSubscriber<ELTestEvent>("ELTestEvent");
    subscriber->registerListener("eventListener", eventListener.cacheListenerFunction);

    kpsr::high_performance::EventLoopPublisher<ELTestEvent, 4> * publisher = (kpsr::high_performance::EventLoopPublisher<ELTestEvent, 4> * ) provider.getPublisher<ELTestEvent>("ELTestEvent", 6, nullptr, nullptr);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 0);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 6);
    ASSERT_EQ(ELTestEvent::copyInvokations, 0);

    kpsr::mem::TestCacheListener<ELTestNewEvent> newEventListener(1);

    kpsr::Subscriber<ELTestNewEvent> * newSubscriber = provider.getSubscriber<ELTestNewEvent>("ELTestNewEvent");
    newSubscriber->registerListener("newEventListener", newEventListener.cacheListenerFunction);

    kpsr::high_performance::EventLoopPublisher<ELTestNewEvent, 4> * newPublisher = ( kpsr::high_performance::EventLoopPublisher<ELTestNewEvent, 4> * ) provider.getPublisher<ELTestNewEvent>("ELTestNewEvent", 6, nullptr, nullptr);
    std::thread t1([&publisher]{
        for (int i = 0; i < 100; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            ELTestEvent event(i, "hola");
            publisher->publish(event);
        }
    });

    std::thread t2([&newPublisher]{
        for (int i = 0; i < 200; i++) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            ELTestNewEvent event("hola", {(double)i});
            newPublisher->publish(event);
        }
    });

    auto stats = subscriber->getSubscriptionStats("eventListener");
    auto newStats = newSubscriber->getSubscriptionStats("newEventListener");

    t1.join();
    t2.join();
    std::cout << "EventLoopTest::SingleEventEmitterTwoTopicsWithPool. Finishing.1" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    provider.stop();
    std::cout << "EventLoopTest::SingleEventEmitterTwoTopicsWithPool. Finishing.2" << std::endl;


    if (publisher->_discardedMessages == 0) {
        ASSERT_EQ(99, eventListener.getLastReceivedEvent()->_id);
    }
    ASSERT_EQ("hola", eventListener.getLastReceivedEvent()->_message);

    if (newPublisher->_discardedMessages == 0) {
        ASSERT_EQ(199, newEventListener.getLastReceivedEvent()->_values[0]);
    }

    long totalMessages = stats->_totalProcessed + publisher->_discardedMessages;
    ASSERT_EQ(totalMessages, 100);
    long totalNewMessages = newStats->_totalProcessed + newPublisher->_discardedMessages;
    ASSERT_EQ(totalNewMessages, 200);

    ASSERT_EQ(ELTestEvent::constructorInvokations, 100);
    ASSERT_EQ(ELTestEvent::emptyConstructorInvokations, 6);
    ASSERT_EQ(ELTestEvent::copyInvokations, eventListener.counter);
}
