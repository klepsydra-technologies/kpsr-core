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

#include <gtest/gtest.h>

#include <klepsydra/core/service.h>
#include <klepsydra/core/publisher.h>
#include <klepsydra/core/subscriber.h>
#include <klepsydra/core/event_emitter_middleware_provider.h>
#include <klepsydra/core/cache_listener.h>

#include <klepsydra/mem_core/basic_middleware_provider.h>

#include <klepsydra/zmq_core/zhelpers.hpp>
#include <klepsydra/zmq_core/from_zmq_middleware_provider.h>
#include <klepsydra/zmq_core/to_zmq_middleware_provider.h>

#include <klepsydra/serialization/binary_cereal_mapper.h>
#include <klepsydra/serialization/json_cereal_mapper.h>

TEST(ZmqCVTest, ZmqTestIntJson) {
    std::string serverUrl = "tcp://*:9001";
    std::string topic = "image_data";

    //  Prepare our context and publisher
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind(serverUrl);
    publisher.bind("ipc://cvMat-tests.ipc");
    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQMiddlewareProvider(nullptr, publisher);
    kpsr::Publisher<int> * toZMQPublisher = toZMQMiddlewareProvider.getJsonToMiddlewareChannel<int>(topic, 0);

    std::string clientUrl = "tcp://localhost:9001";

    zmq::socket_t subscriber (context, ZMQ_SUB);

    subscriber.connect(clientUrl);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
    kpsr::zmq_mdlw::FromZmqChannel<std::string> * _jsonFromZMQProvider = _fromZmqMiddlewareProvider.getJsonFromMiddlewareChannel<int>(subscriber, 100);
    _jsonFromZMQProvider->start();
    kpsr::EventEmitterMiddlewareProvider<int> imageDataProvider(nullptr, topic, 0, nullptr, nullptr);

    _jsonFromZMQProvider->registerToTopic(topic, imageDataProvider.getPublisher());
    kpsr::mem::TestCacheListener<int> cacheListener(10);
    imageDataProvider.getSubscriber()->registerListener("simple_reader", cacheListener.cacheListenerFunction);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    for (unsigned int i = 0; i < 5; ++i) {
        int matToSend = i+1;
        toZMQPublisher->publish(matToSend);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        auto event = cacheListener.getLastReceivedEvent();
        EXPECT_NE(event, nullptr);
        if (event) {
            EXPECT_EQ(*event, matToSend);
        }
    }

    _jsonFromZMQProvider->unregisterFromTopic(topic);
    _jsonFromZMQProvider->stop();
}

TEST(ZmqCVTest, ZmqTestIntBinary) {
    std::string serverUrl = "tcp://*:9001";
    std::string topic = "image_data";

    //  Prepare our context and publisher
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind(serverUrl);
    publisher.bind("ipc://cvMat-tests.ipc");
    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQMiddlewareProvider(nullptr, publisher);
    kpsr::Publisher<int> * toZMQPublisher = toZMQMiddlewareProvider.getBinaryToMiddlewareChannel<int>(topic, 0);

    std::string clientUrl = "tcp://localhost:9001";

    zmq::socket_t subscriber (context, ZMQ_SUB);

    subscriber.connect(clientUrl);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
    auto _binaryFromZMQProvider = _fromZmqMiddlewareProvider.getBinaryFromMiddlewareChannel<int>(subscriber, 100);
    _binaryFromZMQProvider->start();
    kpsr::EventEmitterMiddlewareProvider<int> imageDataProvider(nullptr, topic, 0, nullptr, nullptr);

    _binaryFromZMQProvider->registerToTopic(topic, imageDataProvider.getPublisher());
    kpsr::mem::TestCacheListener<int> cacheListener(10);
    imageDataProvider.getSubscriber()->registerListener("simple_reader", cacheListener.cacheListenerFunction);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    for (unsigned int i = 0; i < 5; ++i) {
        int matToSend = i+1;
        toZMQPublisher->publish(matToSend);
        
        while (cacheListener.counter < i+1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        auto event = cacheListener.getLastReceivedEvent();
        EXPECT_NE(event, nullptr);
        if (event) {
            EXPECT_EQ(*event, matToSend);
        }
    }

    _binaryFromZMQProvider->unregisterFromTopic(topic);
    _binaryFromZMQProvider->stop();
}
