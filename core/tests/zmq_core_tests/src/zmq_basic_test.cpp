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

class ZmqCVTest : public ::testing::Test {
protected:
    ZmqCVTest()
        : serverUrl("tcp://*:5556")
        , clientUrl("tcp://localhost:5556")
        , syncUrl("tcp://localhost:5557")
        , syncServiceUrl("tcp://*:5557")
        , topic("image_data")
        , context (1)
        , publisher(context, ZMQ_PUB)
        , subscriber(context, ZMQ_SUB)
        , syncclient(context, ZMQ_REQ)
        , syncservice (context, ZMQ_REP)
        {
            publisher.bind(serverUrl);
            publisher.bind("ipc://cvMat-tests.ipc");

            //  Socket to talk to server
            subscriber.connect(clientUrl);
            subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());
            // Set up publisher corresponding to each input.
            syncclient.connect(syncUrl);
            //  - send a synchronization request
            zmq::message_t message("", 1);
            syncservice.bind(syncServiceUrl);
            // Set up publisher corresponding to each input.
            syncclient.connect(syncUrl);
            //  - send a synchronization request
            syncclient.send(message);
            //  - wait for synchronization reply
            zmq::message_t recvMessage;
            syncservice.recv(recvMessage);
            syncservice.send(message);
            syncclient.recv(recvMessage);
        }
    
    std::string serverUrl; 
    std::string clientUrl;
    std::string syncUrl;
    std::string syncServiceUrl;
    std::string topic;
    zmq::context_t context;
    zmq::socket_t publisher;
    zmq::socket_t subscriber;
    zmq::socket_t syncclient;
    zmq::socket_t syncservice;
};

TEST_F(ZmqCVTest, ZmqTestIntJson) {

    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQMiddlewareProvider(nullptr, publisher);
    kpsr::Publisher<int> * toZMQPublisher = toZMQMiddlewareProvider.getJsonToMiddlewareChannel<int>(topic, 0);

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

TEST_F(ZmqCVTest, ZmqTestIntBinary) {
    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQMiddlewareProvider(nullptr, publisher);
    kpsr::Publisher<int> * toZMQPublisher = toZMQMiddlewareProvider.getBinaryToMiddlewareChannel<int>(topic, 0);

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
