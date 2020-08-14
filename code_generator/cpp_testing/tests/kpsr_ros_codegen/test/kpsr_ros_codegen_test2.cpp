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

#include "std_msgs/String.h"
#include <gtest/gtest.h>

#include <klepsydra/core/event_emitter_middleware_provider.h>
#include <klepsydra/core/cache_listener.h>

#include <kpsr_ros_serialization/primitive_type_ros_mapper.h>

#include <kpsr_ros_core/to_ros_middleware_provider.h>
#include <kpsr_ros_core/from_ros_middleware_provider.h>

#include "primitive_types_basic_ros_mapper.h"
#include "primitive_types_array_ros_mapper.h"
#include "primitive_types_vector_ros_mapper.h"
#include "primitive_types_vector_shared_ptr_ros_mapper.h"
#include "primitive_types_vector_pointer_ros_mapper.h"

TEST(KpsrRosCodegeTest, primitiveTypeBasicMapperTest) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_codegen_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::PrimitiveTypesBasic> basicProvider(nullptr, "test", 0, nullptr, nullptr);

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider.registerToTopic<kpsr::codegen::PrimitiveTypesBasic, kpsr_ros_codegen::PrimitiveTypesBasic>("kpsr_ros_codegen_test_topic1", 10, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::codegen::PrimitiveTypesBasic> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);
    rate.sleep();

    ros::Publisher stringPublisher = nodeHandle.advertise<kpsr_ros_codegen::PrimitiveTypesBasic>("kpsr_ros_codegen_test_topic1", 10, true);

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<kpsr::codegen::PrimitiveTypesBasic> * kpsrPublisher = toRosProvider.getToMiddlewareChannel<kpsr::codegen::PrimitiveTypesBasic, kpsr_ros_codegen::PrimitiveTypesBasic>("kpsr_ros_codegen_test_topic1", 10, nullptr, stringPublisher);

    unsigned short seq = 0;
    {
        kpsr::codegen::PrimitiveTypesBasic event(seq++, 'a', 0, 1, 2 , 3, 4, 5, 6.0, 6.1, true, "a1");
        kpsrPublisher->publish(event);
    }
    ros::spinOnce();
    rate.sleep();

    {
        kpsr::codegen::PrimitiveTypesBasic event(seq++, 'a', 0, 1, 2 , 3, 4, 5, 6.0, 6.1, true, "a1");
        kpsrPublisher->publish(event);
    }
    ros::spinOnce();
    rate.sleep();

    {
        kpsr::codegen::PrimitiveTypesBasic event(seq++, 'a', 0, 1, 2 , 3, 4, 5, 6.0, 6.1, true, "a1");
        kpsrPublisher->publish(event);
    }
    ros::spinOnce();
    rate.sleep();

    {
        kpsr::codegen::PrimitiveTypesBasic event(seq++, 'a', 0, 1, 2 , 3, 4, 5, 6.0, 6.1, true, "a1");
        kpsrPublisher->publish(event);
    }
    ros::spinOnce();
    rate.sleep();

    kpsr::codegen::PrimitiveTypesBasic event(seq++, 'a', 0, 1, 2 , 3, 4, 5, 6.0, 6.1, true, "a1");
    kpsrPublisher->publish(event);
    ros::spinOnce();
    rate.sleep();

    while (cacheListener.counter < 5) {
        ros::spinOnce();
        rate.sleep();
    }

    ASSERT_EQ(cacheListener.counter, 5);
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

TEST(KpsrRosCodegeTest, primitiveTypeArrayMapperTest) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_codegen_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::PrimitiveTypesArray> basicProvider(nullptr, "test", 0, nullptr, nullptr);

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider.registerToTopic<kpsr::codegen::PrimitiveTypesArray, kpsr_ros_codegen::PrimitiveTypesArray>("kpsr_ros_codegen_test_topic1", 10, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::codegen::PrimitiveTypesArray> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);
    rate.sleep();

    ros::Publisher stringPublisher = nodeHandle.advertise<kpsr_ros_codegen::PrimitiveTypesArray>("kpsr_ros_codegen_test_topic1", 10, true);

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<kpsr::codegen::PrimitiveTypesArray> * kpsrPublisher = toRosProvider.getToMiddlewareChannel<kpsr::codegen::PrimitiveTypesArray, kpsr_ros_codegen::PrimitiveTypesArray>("kpsr_ros_codegen_test_topic1", 1, nullptr, stringPublisher);

    unsigned short seq = 0;
    {
        kpsr::codegen::PrimitiveTypesArray event(seq++, {{'a', 'b'}}, {{0, 1, 2}}, {{3, 4, 5, 6}}, {{ 7, 8, 9, 10, 11 }}, {{12, 13, 14, 15, 16, 17}},
        {{ 18, 19, 20, 21, 22, 23, 24}}, {{ 25, 26, 27, 28, 29, 30, 31, 32 }},
        {{ 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}}, {{ 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9}},
        {{ true, false, true, false, true, false, true, false, true, false, true }},
        {{ "a1", "a2", "a3", "a4", "a5", "a6", "a7", "a8", "a9", "a10", "a11", "a12"}});
        kpsrPublisher->publish(event);
    }
    ros::spinOnce();
    rate.sleep();

    {
        kpsr::codegen::PrimitiveTypesArray event(seq++, {{'a', 'b'}}, {{0, 1, 2}}, {{3, 4, 5, 6}}, {{ 7, 8, 9, 10, 11 }}, {{12, 13, 14, 15, 16, 17}},
        {{ 18, 19, 20, 21, 22, 23, 24}}, {{ 25, 26, 27, 28, 29, 30, 31, 32 }},
        {{ 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}}, {{ 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9}},
        {{ true, false, true, false, true, false, true, false, true, false, true }},
        {{ "b1", "b2", "b3", "b4", "b5", "b6", "b7", "b8", "b9", "b10", "b11", "b12"}});
        kpsrPublisher->publish(event);
    }
    ros::spinOnce();
    rate.sleep();

    {
        kpsr::codegen::PrimitiveTypesArray event(seq++, {{'a', 'b'}}, {{0, 1, 2}}, {{3, 4, 5, 6}}, {{ 7, 8, 9, 10, 11 }}, {{12, 13, 14, 15, 16, 17}},
        {{ 18, 19, 20, 21, 22, 23, 24}}, {{ 25, 26, 27, 28, 29, 30, 31, 32 }},
        {{ 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}}, {{ 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9}},
        {{ true, false, true, false, true, false, true, false, true, false, true }},
        {{ "c1", "c2", "c3", "c4", "c5", "c6", "c7", "c8", "c9", "c10", "c11", "c12"}});
        kpsrPublisher->publish(event);
    }
    ros::spinOnce();
    rate.sleep();

    {
        kpsr::codegen::PrimitiveTypesArray event(seq++, {{'a', 'b'}}, {{0, 1, 2}}, {{3, 4, 5, 6}}, {{ 7, 8, 9, 10, 11 }}, {{12, 13, 14, 15, 16, 17}},
        {{ 18, 19, 20, 21, 22, 23, 24}}, {{ 25, 26, 27, 28, 29, 30, 31, 32 }},
        {{ 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}}, {{ 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9}},
        {{ true, false, true, false, true, false, true, false, true, false, true }},
        {{ "d1", "d2", "d3", "d4", "d5", "d6", "d7", "d8", "d9", "d10", "d11", "d12"}});
        kpsrPublisher->publish(event);
    }
    ros::spinOnce();
    rate.sleep();

    kpsr::codegen::PrimitiveTypesArray event(seq++, {{'a', 'b'}}, {{0, 1, 2}}, {{3, 4, 5, 6}}, {{ 7, 8, 9, 10, 11 }}, {{12, 13, 14, 15, 16, 17}},
    {{ 18, 19, 20, 21, 22, 23, 24}}, {{ 25, 26, 27, 28, 29, 30, 31, 32 }},
    {{ 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}}, {{ 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9}},
    {{ true, false, true, false, true, false, true, false, true, false, true }},
    {{ "e1", "e2", "e3", "e4", "e5", "e6", "e7", "e8", "e9", "e10", "e11", "e12"}});
    kpsrPublisher->publish(event);
    ros::spinOnce();
    rate.sleep();

    while (cacheListener.counter < 5) {
        ros::spinOnce();
        rate.sleep();
    }

    ASSERT_EQ(cacheListener.counter, 5);
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

TEST(KpsrRosCodegeTest, primitiveTypeVectorMapperTest) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_codegen_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::PrimitiveTypesVector> basicProvider(nullptr, "test", 0, nullptr, nullptr);

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider.registerToTopic<kpsr::codegen::PrimitiveTypesVector, kpsr_ros_codegen::PrimitiveTypesVector>("kpsr_ros_codegen_test_topic2", 1, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::codegen::PrimitiveTypesVector> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);
    rate.sleep();

    ros::Publisher stringPublisher = nodeHandle.advertise<kpsr_ros_codegen::PrimitiveTypesVector>("kpsr_ros_codegen_test_topic2", 1, true);

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<kpsr::codegen::PrimitiveTypesVector> * kpsrPublisher = toRosProvider.getToMiddlewareChannel<kpsr::codegen::PrimitiveTypesVector, kpsr_ros_codegen::PrimitiveTypesVector>("kpsr_ros_codegen_test_topic2", 1, nullptr, stringPublisher);

    unsigned short seq = 0;
    {
        kpsr::codegen::PrimitiveTypesVector event(seq++, {'a', 'b'}, {0, 1, 2}, {3, 4, 5, 6}, { 7, 8, 9, 10, 11 }, {12, 13, 14, 15, 16, 17},
        { 18, 19, 20, 21, 22, 23, 24}, { 25, 26, 27, 28, 29, 30, 31, 32 },
        { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}, { 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9},
        { true, false, true, false, true, false, true, false, true, false, true },
        { "a1", "a2", "a3", "a4", "a5", "a6", "a7", "a8", "a9", "a10", "a11", "a12"});
        kpsrPublisher->publish(event);
    }
    ros::spinOnce();
    rate.sleep();

    {
        kpsr::codegen::PrimitiveTypesVector event(seq++, {'a', 'b'}, {0, 1, 2}, {3, 4, 5, 6}, { 7, 8, 9, 10, 11 }, {12, 13, 14, 15, 16, 17},
        { 18, 19, 20, 21, 22, 23, 24}, { 25, 26, 27, 28, 29, 30, 31, 32 },
        { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}, { 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9},
        { true, false, true, false, true, false, true, false, true, false, true },
        { "b1", "b2", "b3", "b4", "b5", "b6", "b7", "b8", "b9", "b10", "b11", "b12"});
        kpsrPublisher->publish(event);
    }
    ros::spinOnce();
    rate.sleep();

    {
        kpsr::codegen::PrimitiveTypesVector event(seq++, {'a', 'b'}, {0, 1, 2}, {3, 4, 5, 6}, { 7, 8, 9, 10, 11 }, {12, 13, 14, 15, 16, 17},
        { 18, 19, 20, 21, 22, 23, 24}, { 25, 26, 27, 28, 29, 30, 31, 32 },
        { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}, { 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9},
        { true, false, true, false, true, false, true, false, true, false, true },
        { "c1", "c2", "c3", "c4", "c5", "c6", "c7", "c8", "c9", "c10", "c11", "c12"});
        kpsrPublisher->publish(event);
    }
    ros::spinOnce();
    rate.sleep();

    {
        kpsr::codegen::PrimitiveTypesVector event(seq++, {'a', 'b'}, {0, 1, 2}, {3, 4, 5, 6}, { 7, 8, 9, 10, 11 }, {12, 13, 14, 15, 16, 17},
        { 18, 19, 20, 21, 22, 23, 24}, { 25, 26, 27, 28, 29, 30, 31, 32 },
        { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}, { 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9},
        { true, false, true, false, true, false, true, false, true, false, true },
        { "d1", "d2", "d3", "d4", "d5", "d6", "d7", "d8", "d9", "d10", "d11", "d12"});
        kpsrPublisher->publish(event);
    }
    ros::spinOnce();
    rate.sleep();

    kpsr::codegen::PrimitiveTypesVector event(seq++, {'a', 'b'}, {0, 1, 2}, {3, 4, 5, 6}, { 7, 8, 9, 10, 11 }, {12, 13, 14, 15, 16, 17},
    { 18, 19, 20, 21, 22, 23, 24}, { 25, 26, 27, 28, 29, 30, 31, 32 },
    { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9}, { 1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9},
    { true, false, true, false, true, false, true, false, true, false, true },
    { "e1", "e2", "e3", "e4", "e5", "e6", "e7", "e8", "e9", "e10", "e11", "e12"});
    kpsrPublisher->publish(event);
    ros::spinOnce();
    rate.sleep();

    while (cacheListener.counter < 5) {
        ros::spinOnce();
        rate.sleep();
    }

    ASSERT_EQ(cacheListener.counter, 5);
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

TEST(KpsrRosCodegeTest, primitiveTypeVectorSharedPtrMapperTest) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_codegen_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::PrimitiveTypesVectorSharedPtr> basicProvider(nullptr, "test", 0, nullptr, nullptr);

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider.registerToTopic<kpsr::codegen::PrimitiveTypesVectorSharedPtr, kpsr_ros_codegen::PrimitiveTypesVectorSharedPtr>("kpsr_ros_codegen_test_topic2", 1, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::codegen::PrimitiveTypesVectorSharedPtr> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);
    rate.sleep();

    ros::Publisher stringPublisher = nodeHandle.advertise<kpsr_ros_codegen::PrimitiveTypesVectorSharedPtr>("kpsr_ros_codegen_test_topic2", 1, true);

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<kpsr::codegen::PrimitiveTypesVectorSharedPtr> * kpsrPublisher = toRosProvider.getToMiddlewareChannel<kpsr::codegen::PrimitiveTypesVectorSharedPtr, kpsr_ros_codegen::PrimitiveTypesVectorSharedPtr>("kpsr_ros_codegen_test_topic2", 1, nullptr, stringPublisher);

    unsigned short seq = 0;
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
    ros::spinOnce();
    rate.sleep();

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
    ros::spinOnce();
    rate.sleep();

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
    ros::spinOnce();
    rate.sleep();

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
    ros::spinOnce();
    rate.sleep();

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
    ros::spinOnce();
    rate.sleep();

    while (cacheListener.counter < 5) {
        ros::spinOnce();
        rate.sleep();
    }

    ASSERT_EQ(cacheListener.counter, 5);
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

TEST(KpsrRosCodegeTest, primitiveTypeVectorPointerMapperTest) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_codegen_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::PrimitiveTypesVectorPointer> basicProvider(nullptr, "test", 0, nullptr, nullptr);

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider.registerToTopic<kpsr::codegen::PrimitiveTypesVectorPointer, kpsr_ros_codegen::PrimitiveTypesVectorPointer>("kpsr_ros_codegen_test_topic2", 1, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::codegen::PrimitiveTypesVectorPointer> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);
    rate.sleep();

    ros::Publisher stringPublisher = nodeHandle.advertise<kpsr_ros_codegen::PrimitiveTypesVectorPointer>("kpsr_ros_codegen_test_topic2", 1, true);

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<kpsr::codegen::PrimitiveTypesVectorPointer> * kpsrPublisher = toRosProvider.getToMiddlewareChannel<kpsr::codegen::PrimitiveTypesVectorPointer, kpsr_ros_codegen::PrimitiveTypesVectorPointer>("kpsr_ros_codegen_test_topic2", 1, nullptr, stringPublisher);

    unsigned short seq = 0;
    {
        kpsr::codegen::PrimitiveTypesVectorPointer event(seq++,
        {new signed char('a')}, {new unsigned char(0)}, {new short int(1)}, {new unsigned short int(2)},
        {new unsigned int(3)}, {new long long int(4)}, {new unsigned long long int(5)}, {new float(6.1)},
        {new double(6.2)}, {new bool(true)}, {new std::string("hola")});
        kpsrPublisher->publish(event);
    }
    ros::spinOnce();
    rate.sleep();

    {
        kpsr::codegen::PrimitiveTypesVectorPointer event(seq++,
        {new signed char('a')}, {new unsigned char(0)}, {new short int(1)}, {new unsigned short int(2)},
        {new unsigned int(3)}, {new long long int(4)}, {new unsigned long long int(5)}, {new float(6.1)},
        {new double(6.2)}, {new bool(true)}, {new std::string("hola")});
        kpsrPublisher->publish(event);
    }
    ros::spinOnce();
    rate.sleep();

    {
        kpsr::codegen::PrimitiveTypesVectorPointer event(seq++,
        {new signed char('a')}, {new unsigned char(0)}, {new short int(1)}, {new unsigned short int(2)},
        {new unsigned int(3)}, {new long long int(4)}, {new unsigned long long int(5)}, {new float(6.1)},
        {new double(6.2)}, {new bool(true)}, {new std::string("hola")});
        kpsrPublisher->publish(event);
    }
    ros::spinOnce();
    rate.sleep();

    {
        kpsr::codegen::PrimitiveTypesVectorPointer event(seq++,
        {new signed char('a')}, {new unsigned char(0)}, {new short int(1)}, {new unsigned short int(2)},
        {new unsigned int(3)}, {new long long int(4)}, {new unsigned long long int(5)}, {new float(6.1)},
        {new double(6.2)}, {new bool(true)}, {new std::string("hola")});
        kpsrPublisher->publish(event);
    }
    ros::spinOnce();
    rate.sleep();

    kpsr::codegen::PrimitiveTypesVectorPointer event(seq++,
    {new signed char('a')}, {new unsigned char(0)}, {new short int(1)}, {new unsigned short int(2)},
    {new unsigned int(3)}, {new long long int(4)}, {new unsigned long long int(5)}, {new float(6.1)},
    {new double(6.2)}, {new bool(true)}, {new std::string("hola")});
    kpsrPublisher->publish(event);
    ros::spinOnce();
    rate.sleep();

    while (cacheListener.counter < 5) {
        ros::spinOnce();
        rate.sleep();
    }

    ASSERT_EQ(cacheListener.counter, 5);
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->aaa[0], * event.aaa[0]);
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->bbb[0], * event.bbb[0]);
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->ccc[0], * event.ccc[0]);
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->ddd[0], * event.ddd[0]);
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->eee[0], * event.eee[0]);
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->fff[0], * event.fff[0]);
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->ggg[0], * event.ggg[0]);
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->hhh[0], * event.hhh[0]);
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->iii[0], * event.iii[0]);
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->jjj[0], * event.jjj[0]);
    ASSERT_EQ(* cacheListener.getLastReceivedEvent()->kkk[0], * event.kkk[0]);
}
