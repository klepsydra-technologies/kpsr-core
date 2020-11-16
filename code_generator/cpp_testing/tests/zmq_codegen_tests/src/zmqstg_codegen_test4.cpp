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

#include <string>

#include <gtest/gtest.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <zmq.hpp>

#include <klepsydra/serialization/json_cereal_mapper.h>
#include <klepsydra/serialization/binary_cereal_mapper.h>

#include <klepsydra/core/event_emitter_middleware_provider.h>
#include <klepsydra/core/cache_listener.h>

#include <klepsydra/zmq_core/to_zmq_middleware_provider.h>
#include <klepsydra/zmq_core/from_zmq_middleware_provider.h>

#include <klepsydra/codegen/cereal/inheritance_vector4_serializer.h>

TEST(KpsrZMQCodegeTest, inheritanceMapperMapperTest) {

    std::string serverUrl = "tcp://*:5556";
    std::string topic = "test";

    //  Prepare our context and publisher
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind(serverUrl);
    publisher.bind("ipc://weather.ipc");

    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQProvider(nullptr, publisher);

    std::string clientUrl = "tcp://localhost:5556";

    //  Socket to talk to server
    spdlog::info("Collecting updates from weather server...\n");
    zmq::socket_t subscriber (context, ZMQ_SUB);

    subscriber.connect(clientUrl);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
    kpsr::zmq_mdlw::FromZmqChannel<std::string> * fromZMQProvider =
            _fromZmqMiddlewareProvider.getJsonFromMiddlewareChannel<kpsr::codegen::InheritanceVector4>(subscriber, 10);
    fromZMQProvider->start();

    kpsr::Publisher<kpsr::codegen::InheritanceVector4> * kpsrPublisher =
            toZMQProvider.getJsonToMiddlewareChannel<kpsr::codegen::InheritanceVector4>(topic, 0);

    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::InheritanceVector4> basicProvider(nullptr, "test", 0, nullptr, nullptr);

    fromZMQProvider->registerToTopic(topic, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::codegen::InheritanceVector4> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

    while (cacheListener.counter < 1) {
        kpsr::codegen::InheritanceVector4 event(1.0, 1.1, 1.2, 1.3);
        kpsrPublisher->publish(event);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
        kpsr::codegen::InheritanceVector4 event(2.0, 2.1, 2.2, 2.3);
        kpsrPublisher->publish(event);
    }

    {
        kpsr::codegen::InheritanceVector4 event(3.0, 3.1, 3.2, 3.3);
        kpsrPublisher->publish(event);
    }

    {
        kpsr::codegen::InheritanceVector4 event(4.0, 4.1, 4.2, 4.3);
        kpsrPublisher->publish(event);
    }

    kpsr::codegen::InheritanceVector4 event(5.0, 5.1, 5.2, 5.3);
    kpsrPublisher->publish(event);

    while (cacheListener.counter < 5) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    fromZMQProvider->stop();

    ASSERT_GE(cacheListener.counter, 5);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->a, event.a);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->b, event.b);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->c, event.c);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->d, event.d);
}

