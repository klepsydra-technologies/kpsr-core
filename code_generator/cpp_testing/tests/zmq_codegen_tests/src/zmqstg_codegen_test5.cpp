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

#include <klepsydra/codegen/cereal/composition_type_related_serializer.h>

TEST(KpsrZMQCodegeTest, compositionTypeRelatedMapperTest) {

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
            _fromZmqMiddlewareProvider.getJsonFromMiddlewareChannel<kpsr::codegen::CompositionTypeRelated>(subscriber, 10);
    fromZMQProvider->start();

    kpsr::Publisher<kpsr::codegen::CompositionTypeRelated> * kpsrPublisher =
            toZMQProvider.getJsonToMiddlewareChannel<kpsr::codegen::CompositionTypeRelated>(topic, 0);

    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::CompositionTypeRelated> basicProvider(nullptr, "test", 0, nullptr, nullptr);

    fromZMQProvider->registerToTopic(topic, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::codegen::CompositionTypeRelated> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

    unsigned short seq = 0;
    kpsr::codegen::CompositionTypeRelated event;

    int attempts = 0;
    while (cacheListener.counter < 5) {
        event.seq = seq++;
        event.newEnum = kpsr::codegen::NewEnum::new1;
        event.newEnumArray = {{kpsr::codegen::NewEnum::new2, kpsr::codegen::NewEnum::new3}};
        event.newEnumVector = {kpsr::codegen::NewEnum::new1, kpsr::codegen::NewEnum::new2};
        event.newEnumVectorPointer = {std::shared_ptr<kpsr::codegen::NewEnum>(new kpsr::codegen::NewEnum(kpsr::codegen::NewEnum::new1))};
        event.newEnumVectorSharedPtr = {std::shared_ptr<kpsr::codegen::NewEnum>(new kpsr::codegen::NewEnum(kpsr::codegen::NewEnum::new1))};

        event.oldEnum = kpsr::codegen::OldEnum::oldA;
        event.oldEnumArray = {{kpsr::codegen::OldEnum::oldA, kpsr::codegen::OldEnum::oldB}};
        event.oldEnumVector = {kpsr::codegen::OldEnum::oldA, kpsr::codegen::OldEnum::oldB};
        event.oldEnumVectorPointer = {std::shared_ptr<kpsr::codegen::OldEnum>(new kpsr::codegen::OldEnum(kpsr::codegen::OldEnum::oldA))};
        event.oldEnumVectorSharedPtr = {std::shared_ptr<kpsr::codegen::OldEnum>(new kpsr::codegen::OldEnum(kpsr::codegen::OldEnum::oldA))};

        event.positionArray = {{ kpsr::geometry::Vector3(seq++, 0.1, 0.2, 0.3), kpsr::geometry::Vector3(seq++, 1.1, 1.2, 1.3) }};
        event.positionVector = { kpsr::geometry::Vector3(seq++, 0.1, 0.2, 0.3), kpsr::geometry::Vector3(seq++, 1.1, 1.2, 1.3) };
        event.positionVectorPointer = { std::shared_ptr<kpsr::geometry::Vector3>(new kpsr::geometry::Vector3(seq++, 0.1, 0.2, 0.3)),
                                        std::shared_ptr<kpsr::geometry::Vector3>(new kpsr::geometry::Vector3(seq++, 1.1, 1.2, 1.3)) };
        event.positionVectorSharedPtr = { std::shared_ptr<kpsr::geometry::Vector3>(new kpsr::geometry::Vector3(seq++, 0.1, 0.2, 0.3)),
                                          std::shared_ptr<kpsr::geometry::Vector3>(new kpsr::geometry::Vector3(seq++, 1.1, 1.2, 1.3)) };

        event.quaternionArray = {{ kpsr::codegen::Vector4(0.1, 0.2, 0.3, 0.4), kpsr::codegen::Vector4(1.1, 1.2, 1.3, 1.4) }};
        event.quaternionVector = { kpsr::codegen::Vector4(0.1, 0.2, 0.3, 0.4), kpsr::codegen::Vector4(1.1, 1.2, 1.3, 1.4) };
        event.quaternionVectorPointer = { std::shared_ptr<kpsr::codegen::Vector4>(new kpsr::codegen::Vector4(0.1, 0.2, 0.3, 0.4)),
                                          std::shared_ptr<kpsr::codegen::Vector4>(new kpsr::codegen::Vector4(1.1, 1.2, 1.3, 1.4)) };
        event.quaternionVectorSharedPtr = { std::shared_ptr<kpsr::codegen::Vector4>(new kpsr::codegen::Vector4(0.1, 0.2, 0.3, 0.4)),
                                          std::shared_ptr<kpsr::codegen::Vector4>(new kpsr::codegen::Vector4(1.1, 1.2, 1.3, 1.4)) };
        event.quat = kpsr::geometry::Quaternion(0, 1.0, 2.0, 3.0, 4.0);
        event.gpsData = kpsr::geometry::Gps(1, 1.0, 2.0, 3.0);
        kpsrPublisher->publish(event);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        attempts++;
    }

    spdlog::info("Total publishing attempts: {}", attempts);


    fromZMQProvider->stop();

    ASSERT_GE(cacheListener.counter, 5);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->newEnum, event.newEnum);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->newEnumArray, event.newEnumArray);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->newEnumVector, event.newEnumVector);
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->newEnumVectorPointer[0], * event.newEnumVectorPointer[0]);
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->newEnumVectorSharedPtr[0].get(), * event.newEnumVectorSharedPtr[0].get());

    ASSERT_EQ(cacheListener.getLastReceivedEvent()->oldEnum, event.oldEnum);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->oldEnumArray, event.oldEnumArray);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->oldEnumVector[0], event.oldEnumVector[0]);
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->oldEnumVectorPointer[0], * event.oldEnumVectorPointer[0]);
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->oldEnumVectorSharedPtr[0].get(), * event.oldEnumVectorSharedPtr[0].get());

    ASSERT_EQ(cacheListener.getLastReceivedEvent()->positionArray[0].x, event.positionArray[0].x);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->positionVector[0].x, event.positionVector[0].x);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->positionVectorPointer[0]->x, event.positionVectorPointer[0]->x);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->positionVectorSharedPtr[0]->x, event.positionVectorSharedPtr[0]->x);

    ASSERT_EQ(cacheListener.getLastReceivedEvent()->quaternionArray[0].a, event.quaternionArray[0].a);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->quaternionVector[0].a, event.quaternionVector[0].a);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->quaternionVectorPointer[0]->a, event.quaternionVectorPointer[0]->a);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->quaternionVectorSharedPtr[0]->a, event.quaternionVectorSharedPtr[0]->a);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->quat.x, event.quat.x);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->gpsData.altitude, event.gpsData.altitude);
}

