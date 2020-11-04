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

#include <gtest/gtest.h>

#include <klepsydra/core/service.h>
#include <klepsydra/core/publisher.h>
#include <klepsydra/core/subscriber.h>
#include <klepsydra/core/event_emitter_middleware_provider.h>
#include <klepsydra/core/cache_listener.h>

#include <klepsydra/mem_core/basic_middleware_provider.h>

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

    for (int i = 0; i < 5; ++i) {
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
