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

#include <klepsydra/codegen/cereal//primitive_types_basic_serializer.h>
#include <klepsydra/codegen/cereal//primitive_types_array_serializer.h>
#include <klepsydra/codegen/cereal//primitive_types_vector_serializer.h>
#include <klepsydra/codegen/cereal//primitive_types_vector_shared_ptr_serializer.h>

TEST(KpsrZMQCodegeTest, primitiveTypeBasicMapperTest) {

    std::string serverUrl = "tcp://*:5556";
    std::string topic = "test";

    //  Prepare our context and publisher
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind(serverUrl);
    publisher.bind("ipc://test.ipc");

    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQProvider(nullptr, publisher);

    std::string clientUrl = "tcp://localhost:5556";

    //  Socket to talk to server
    spdlog::info("Collecting updates from test server...\n");
    zmq::socket_t subscriber (context, ZMQ_SUB);

    subscriber.connect(clientUrl);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
    kpsr::zmq_mdlw::FromZmqChannel<std::string> * fromZMQProvider =
            _fromZmqMiddlewareProvider.getJsonFromMiddlewareChannel<kpsr::codegen::PrimitiveTypesBasic>(subscriber, 10);
    fromZMQProvider->start();

    kpsr::Publisher<kpsr::codegen::PrimitiveTypesBasic> * kpsrPublisher =
            toZMQProvider.getJsonToMiddlewareChannel<kpsr::codegen::PrimitiveTypesBasic>(topic, 0);

    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::PrimitiveTypesBasic> basicProvider(nullptr, "test", 0, nullptr, nullptr);

    fromZMQProvider->registerToTopic(topic, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::codegen::PrimitiveTypesBasic> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

    unsigned short seq = 0;

    while (cacheListener.counter < 1) {
        kpsr::codegen::PrimitiveTypesBasic event(seq++, 'a', 0, 1, 2 , 3, 4, 5, 6.0, 6.1, true, "a1");
        kpsrPublisher->publish(event);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
        kpsr::codegen::PrimitiveTypesBasic event(seq++, 'a', 0, 1, 2 , 3, 4, 5, 6.0, 6.1, true, "a1");
        kpsrPublisher->publish(event);
    }

    {
        kpsr::codegen::PrimitiveTypesBasic event(seq++, 'a', 0, 1, 2 , 3, 4, 5, 6.0, 6.1, true, "a1");
        kpsrPublisher->publish(event);
    }

    {
        kpsr::codegen::PrimitiveTypesBasic event(seq++, 'a', 0, 1, 2 , 3, 4, 5, 6.0, 6.1, true, "a1");
        kpsrPublisher->publish(event);
    }

    kpsr::codegen::PrimitiveTypesBasic event(seq++, 'a', 0, 1, 2 , 3, 4, 5, 6.0, 6.1, true, "a1");
    kpsrPublisher->publish(event);

    while (cacheListener.counter < 5) {
        spdlog::info("second wait...");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    fromZMQProvider->stop();

    ASSERT_GE(cacheListener.counter, 5);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->a, event.a);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->b, event.b);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->c, event.c);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->d, event.d);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->e, event.e);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->f, event.f);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->g, event.g);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->h, event.h);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->i, event.i);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->j, event.j);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->k, event.k);
}

TEST(KpsrZMQCodegeTest, primitiveTypeArrayMapperTest) {

    std::string serverUrl = "tcp://*:5556";
    std::string topic = "test";

    //  Prepare our context and publisher
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind(serverUrl);
    publisher.bind("ipc://test.ipc");

    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQProvider(nullptr, publisher);

    std::string clientUrl = "tcp://localhost:5556";

    //  Socket to talk to server
    spdlog::info("Collecting updates from test server...\n");
    zmq::socket_t subscriber (context, ZMQ_SUB);

    subscriber.connect(clientUrl);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
    kpsr::zmq_mdlw::FromZmqChannel<std::string> * fromZMQProvider =
            _fromZmqMiddlewareProvider.getJsonFromMiddlewareChannel<kpsr::codegen::PrimitiveTypesArray>(subscriber, 10);
    fromZMQProvider->start();

    kpsr::Publisher<kpsr::codegen::PrimitiveTypesArray> * kpsrPublisher =
            toZMQProvider.getJsonToMiddlewareChannel<kpsr::codegen::PrimitiveTypesArray>(topic, 0);

    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::PrimitiveTypesArray> basicProvider(nullptr, "test", 0, nullptr, nullptr);

    fromZMQProvider->registerToTopic(topic, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::codegen::PrimitiveTypesArray> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

    unsigned short seq = 0;
    while (cacheListener.counter < 1) {
        kpsr::codegen::PrimitiveTypesArray event(seq++, {{'a', 'b'}}, {{0, 1, 2}}, {{3, 4, 5, 6}}, {{ 7, 8, 9, 10, 11 }}, {{12, 13, 14, 15, 16, 17}},
        {{ 18, 19, 20, 21, 22, 23, 24}}, {{ 25, 26, 27, 28, 29, 30, 31, 32 }},
        {{ 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}}, {{ 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9}},
        {{ true, false, true, false, true, false, true, false, true, false, true }},
        {{ "a1", "a2", "a3", "a4", "a5", "a6", "a7", "a8", "a9", "a10", "a11", "a12"}});
        kpsrPublisher->publish(event);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
        kpsr::codegen::PrimitiveTypesArray event(seq++, {{'a', 'b'}}, {{0, 1, 2}}, {{3, 4, 5, 6}}, {{ 7, 8, 9, 10, 11 }}, {{12, 13, 14, 15, 16, 17}},
        {{ 18, 19, 20, 21, 22, 23, 24}}, {{ 25, 26, 27, 28, 29, 30, 31, 32 }},
        {{ 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}}, {{ 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9}},
        {{ true, false, true, false, true, false, true, false, true, false, true }},
        {{ "b1", "b2", "b3", "b4", "b5", "b6", "b7", "b8", "b9", "b10", "b11", "b12"}});
        kpsrPublisher->publish(event);
    }

    {
        kpsr::codegen::PrimitiveTypesArray event(seq++, {{'a', 'b'}}, {{0, 1, 2}}, {{3, 4, 5, 6}}, {{ 7, 8, 9, 10, 11 }}, {{12, 13, 14, 15, 16, 17}},
        {{ 18, 19, 20, 21, 22, 23, 24}}, {{ 25, 26, 27, 28, 29, 30, 31, 32 }},
        {{ 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}}, {{ 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9}},
        {{ true, false, true, false, true, false, true, false, true, false, true }},
        {{ "c1", "c2", "c3", "c4", "c5", "c6", "c7", "c8", "c9", "c10", "c11", "c12"}});
        kpsrPublisher->publish(event);
    }

    {
        kpsr::codegen::PrimitiveTypesArray event(seq++, {{'a', 'b'}}, {{0, 1, 2}}, {{3, 4, 5, 6}}, {{ 7, 8, 9, 10, 11 }}, {{12, 13, 14, 15, 16, 17}},
        {{ 18, 19, 20, 21, 22, 23, 24}}, {{ 25, 26, 27, 28, 29, 30, 31, 32 }},
        {{ 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}}, {{ 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9}},
        {{ true, false, true, false, true, false, true, false, true, false, true }},
        {{ "d1", "d2", "d3", "d4", "d5", "d6", "d7", "d8", "d9", "d10", "d11", "d12"}});
        kpsrPublisher->publish(event);
    }

    kpsr::codegen::PrimitiveTypesArray event(seq++, {{'a', 'b'}}, {{0, 1, 2}}, {{3, 4, 5, 6}}, {{ 7, 8, 9, 10, 11 }}, {{12, 13, 14, 15, 16, 17}},
    {{ 18, 19, 20, 21, 22, 23, 24}}, {{ 25, 26, 27, 28, 29, 30, 31, 32 }},
    {{ 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}}, {{ 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9}},
    {{ true, false, true, false, true, false, true, false, true, false, true }},
    {{ "e1", "e2", "e3", "e4", "e5", "e6", "e7", "e8", "e9", "e10", "e11", "e12"}});
    kpsrPublisher->publish(event);

    while (cacheListener.counter < 5) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    fromZMQProvider->stop();

    ASSERT_GE(cacheListener.counter, 5);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->aa, event.aa);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->bb, event.bb);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->cc, event.cc);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->dd, event.dd);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->ee, event.ee);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->ff, event.ff);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->gg, event.gg);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->hh, event.hh);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->ii, event.ii);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->jj, event.jj);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->kk, event.kk);
}

TEST(KpsrZMQCodegeTest, primitiveTypeVectorMapperTest) {

    std::string serverUrl = "tcp://*:5556";
    std::string topic = "test";

    //  Prepare our context and publisher
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind(serverUrl);
    publisher.bind("ipc://test.ipc");

    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQProvider(nullptr, publisher);

    std::string clientUrl = "tcp://localhost:5556";

    //  Socket to talk to server
    spdlog::info("Collecting updates from test server...\n");
    zmq::socket_t subscriber (context, ZMQ_SUB);

    subscriber.connect(clientUrl);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
    kpsr::zmq_mdlw::FromZmqChannel<std::string> * fromZMQProvider =
            _fromZmqMiddlewareProvider.getJsonFromMiddlewareChannel<kpsr::codegen::PrimitiveTypesVector>(subscriber, 10);
    fromZMQProvider->start();

    kpsr::Publisher<kpsr::codegen::PrimitiveTypesVector> * kpsrPublisher =
            toZMQProvider.getJsonToMiddlewareChannel<kpsr::codegen::PrimitiveTypesVector>(topic, 0);

    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::PrimitiveTypesVector> basicProvider(nullptr, "test", 0, nullptr, nullptr);

    fromZMQProvider->registerToTopic(topic, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::codegen::PrimitiveTypesVector> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

    unsigned short seq = 0;
    while (cacheListener.counter < 1) {
        kpsr::codegen::PrimitiveTypesVector event(seq++, {'a', 'b'}, {0, 1, 2}, {3, 4, 5, 6}, { 7, 8, 9, 10, 11 }, {12, 13, 14, 15, 16, 17},
        { 18, 19, 20, 21, 22, 23, 24}, { 25, 26, 27, 28, 29, 30, 31, 32 },
        { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}, { 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9},
        { true, false, true, false, true, false, true, false, true, false, true },
        { "a1", "a2", "a3", "a4", "a5", "a6", "a7", "a8", "a9", "a10", "a11", "a12"});
        kpsrPublisher->publish(event);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
        kpsr::codegen::PrimitiveTypesVector event(seq++, {'a', 'b'}, {0, 1, 2}, {3, 4, 5, 6}, { 7, 8, 9, 10, 11 }, {12, 13, 14, 15, 16, 17},
        { 18, 19, 20, 21, 22, 23, 24}, { 25, 26, 27, 28, 29, 30, 31, 32 },
        { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}, { 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9},
        { true, false, true, false, true, false, true, false, true, false, true },
        { "b1", "b2", "b3", "b4", "b5", "b6", "b7", "b8", "b9", "b10", "b11", "b12"});
        kpsrPublisher->publish(event);
    }

    {
        kpsr::codegen::PrimitiveTypesVector event(seq++, {'a', 'b'}, {0, 1, 2}, {3, 4, 5, 6}, { 7, 8, 9, 10, 11 }, {12, 13, 14, 15, 16, 17},
        { 18, 19, 20, 21, 22, 23, 24}, { 25, 26, 27, 28, 29, 30, 31, 32 },
        { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}, { 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9},
        { true, false, true, false, true, false, true, false, true, false, true },
        { "c1", "c2", "c3", "c4", "c5", "c6", "c7", "c8", "c9", "c10", "c11", "c12"});
        kpsrPublisher->publish(event);
    }

    {
        kpsr::codegen::PrimitiveTypesVector event(seq++, {'a', 'b'}, {0, 1, 2}, {3, 4, 5, 6}, { 7, 8, 9, 10, 11 }, {12, 13, 14, 15, 16, 17},
        { 18, 19, 20, 21, 22, 23, 24}, { 25, 26, 27, 28, 29, 30, 31, 32 },
        { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}, { 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9},
        { true, false, true, false, true, false, true, false, true, false, true },
        { "d1", "d2", "d3", "d4", "d5", "d6", "d7", "d8", "d9", "d10", "d11", "d12"});
        kpsrPublisher->publish(event);
    }

    kpsr::codegen::PrimitiveTypesVector event(seq++, {'a', 'b'}, {0, 1, 2}, {3, 4, 5, 6}, { 7, 8, 9, 10, 11 }, {12, 13, 14, 15, 16, 17},
    { 18, 19, 20, 21, 22, 23, 24}, { 25, 26, 27, 28, 29, 30, 31, 32 },
    { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}, { 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9},
    { true, false, true, false, true, false, true, false, true, false, true },
    { "e1", "e2", "e3", "e4", "e5", "e6", "e7", "e8", "e9", "e10", "e11", "e12"});
    kpsrPublisher->publish(event);

    while (cacheListener.counter < 5) {
        spdlog::info("waiting...");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    fromZMQProvider->stop();

    ASSERT_GE(cacheListener.counter, 5);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->aaa, event.aaa);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->bbb, event.bbb);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->ccc, event.ccc);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->ddd, event.ddd);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->eee, event.eee);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->fff, event.fff);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->ggg, event.ggg);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->hhh, event.hhh);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->iii, event.iii);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->jjj, event.jjj);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->kkk, event.kkk);
}

TEST(KpsrZMQCodegeTest, primitiveTypeVectorSharedPtrMapperTest) {

    std::string serverUrl = "tcp://*:5556";
    std::string topic = "test";

    //  Prepare our context and publisher
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind(serverUrl);
    publisher.bind("ipc://test.ipc");

    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQProvider(nullptr, publisher);

    std::string clientUrl = "tcp://localhost:5556";

    //  Socket to talk to server
    spdlog::info("Collecting updates from test server...\n");
    zmq::socket_t subscriber (context, ZMQ_SUB);

    subscriber.connect(clientUrl);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
    kpsr::zmq_mdlw::FromZmqChannel<std::string> * fromZMQProvider =
            _fromZmqMiddlewareProvider.getJsonFromMiddlewareChannel<kpsr::codegen::PrimitiveTypesVectorSharedPtr>(subscriber, 10);
    fromZMQProvider->start();

    kpsr::Publisher<kpsr::codegen::PrimitiveTypesVectorSharedPtr> * kpsrPublisher =
            toZMQProvider.getJsonToMiddlewareChannel<kpsr::codegen::PrimitiveTypesVectorSharedPtr>(topic, 0);

    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::PrimitiveTypesVectorSharedPtr> basicProvider(nullptr, "test", 0, nullptr, nullptr);

    fromZMQProvider->registerToTopic(topic, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::codegen::PrimitiveTypesVectorSharedPtr> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

    unsigned short seq = 0;
    while (cacheListener.counter < 1) {
        kpsr::codegen::PrimitiveTypesVectorSharedPtr event(seq++,
        {std::shared_ptr<signed char>(new signed char('a'))},
        {std::shared_ptr<unsigned char>(new unsigned char(0))},
        {std::shared_ptr<short int>(new short int(1))},
        {std::shared_ptr<unsigned short int>(new unsigned short int(2))},
        {std::shared_ptr<unsigned int>(new unsigned int(3))},
        {std::shared_ptr<long long int>(new long long int(4))},
        {std::shared_ptr<unsigned long long int>(new unsigned long long int(5))},
        {std::shared_ptr<float>(new float(6.1))},
        {std::shared_ptr<double>(new double(6.2))},
        {std::shared_ptr<bool>(new bool(true))},
        {std::shared_ptr<std::string>(new std::string("hola"))});
        kpsrPublisher->publish(event);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
        kpsr::codegen::PrimitiveTypesVectorSharedPtr event(seq++,
        {std::shared_ptr<signed char>(new signed char('a'))},
        {std::shared_ptr<unsigned char>(new unsigned char(0))},
        {std::shared_ptr<short int>(new short int(1))},
        {std::shared_ptr<unsigned short int>(new unsigned short int(2))},
        {std::shared_ptr<unsigned int>(new unsigned int(3))},
        {std::shared_ptr<long long int>(new long long int(4))},
        {std::shared_ptr<unsigned long long int>(new unsigned long long int(5))},
        {std::shared_ptr<float>(new float(6.1))},
        {std::shared_ptr<double>(new double(6.2))},
        {std::shared_ptr<bool>(new bool(true))},
        {std::shared_ptr<std::string>(new std::string("hola"))});
        kpsrPublisher->publish(event);
    }

    {
        kpsr::codegen::PrimitiveTypesVectorSharedPtr event(seq++,
        {std::shared_ptr<signed char>(new signed char('a'))},
        {std::shared_ptr<unsigned char>(new unsigned char(0))},
        {std::shared_ptr<short int>(new short int(1))},
        {std::shared_ptr<unsigned short int>(new unsigned short int(2))},
        {std::shared_ptr<unsigned int>(new unsigned int(3))},
        {std::shared_ptr<long long int>(new long long int(4))},
        {std::shared_ptr<unsigned long long int>(new unsigned long long int(5))},
        {std::shared_ptr<float>(new float(6.1))},
        {std::shared_ptr<double>(new double(6.2))},
        {std::shared_ptr<bool>(new bool(true))},
        {std::shared_ptr<std::string>(new std::string("hola"))});
        kpsrPublisher->publish(event);
    }

    {
        kpsr::codegen::PrimitiveTypesVectorSharedPtr event(seq++,
        {std::shared_ptr<signed char>(new signed char('a'))},
        {std::shared_ptr<unsigned char>(new unsigned char(0))},
        {std::shared_ptr<short int>(new short int(1))},
        {std::shared_ptr<unsigned short int>(new unsigned short int(2))},
        {std::shared_ptr<unsigned int>(new unsigned int(3))},
        {std::shared_ptr<long long int>(new long long int(4))},
        {std::shared_ptr<unsigned long long int>(new unsigned long long int(5))},
        {std::shared_ptr<float>(new float(6.1))},
        {std::shared_ptr<double>(new double(6.2))},
        {std::shared_ptr<bool>(new bool(true))},
        {std::shared_ptr<std::string>(new std::string("hola"))});
        kpsrPublisher->publish(event);
    }

    kpsr::codegen::PrimitiveTypesVectorSharedPtr event(seq++,
    {std::shared_ptr<signed char>(new signed char('a'))},
    {std::shared_ptr<unsigned char>(new unsigned char(0))},
    {std::shared_ptr<short int>(new short int(1))},
    {std::shared_ptr<unsigned short int>(new unsigned short int(2))},
    {std::shared_ptr<unsigned int>(new unsigned int(3))},
    {std::shared_ptr<long long int>(new long long int(4))},
    {std::shared_ptr<unsigned long long int>(new unsigned long long int(5))},
    {std::shared_ptr<float>(new float(6.1))},
    {std::shared_ptr<double>(new double(6.2))},
    {std::shared_ptr<bool>(new bool(true))},
    {std::shared_ptr<std::string>(new std::string("hola"))});
    kpsrPublisher->publish(event);

    while (cacheListener.counter < 5) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    fromZMQProvider->stop();

    ASSERT_GE(cacheListener.counter, 5);
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->aaa[0].get(), * event.aaa[0].get());
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->bbb[0].get(), * event.bbb[0].get());
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->ccc[0].get(), * event.ccc[0].get());
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->ddd[0].get(), * event.ddd[0].get());
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->eee[0].get(), * event.eee[0].get());
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->fff[0].get(), * event.fff[0].get());
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->ggg[0].get(), * event.ggg[0].get());
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->hhh[0].get(), * event.hhh[0].get());
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->iii[0].get(), * event.iii[0].get());
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->jjj[0].get(), * event.jjj[0].get());
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->kkk[0].get(), * event.kkk[0].get());
}
