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

#include <klepsydra/core/event_emitter_middleware_provider.h>
#include <klepsydra/core/cache_listener.h>

#include <klepsydra/dds_serialization/primitive_type_dds_mapper.h>

#include <klepsydra/dds_core/to_dds_middleware_provider.h>
#include <klepsydra/dds_core/from_dds_middleware_provider.h>

#include "dds/dds.hpp"

#include <klepsydra/codegen/dds/inheritance_vector4_dds_mapper.h>

TEST(KpsrDdsCodegenTest4, inheritanceMapperMapperTest) {

    dds::domain::DomainParticipant dp(0);
    dds::pub::Publisher publisher(dp);
    dds::sub::Subscriber subscriber(dp);

    dds::topic::Topic<kpsr_dds_codegen::InheritanceVector4Data> topic(dp, "header_data_test");
    dds::pub::DataWriter<kpsr_dds_codegen::InheritanceVector4Data> dataWriter(publisher, topic);
    dds::sub::DataReader<kpsr_dds_codegen::InheritanceVector4Data> dataReader(subscriber, topic);

    kpsr::dds_mdlw::FromDDSMiddlewareProvider fromDDSProvider;
    kpsr::dds_mdlw::ToDDSMiddlewareProvider toDDSProvider(nullptr);

    kpsr::Publisher<kpsr::codegen::InheritanceVector4> * kpsrPublisher =
            toDDSProvider.getToMiddlewareChannel<kpsr::codegen::InheritanceVector4, kpsr_dds_codegen::InheritanceVector4Data>("kpsr_ros_codegen_test_topicA", 1, nullptr, &dataWriter);

    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::InheritanceVector4> basicProvider(nullptr, "test", 0, nullptr, nullptr);

    fromDDSProvider.registerToTopic("kpsr_dds_codegen_test_topic1", &dataReader, true, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::codegen::InheritanceVector4> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

    {
        kpsr::codegen::InheritanceVector4 event(1.0, 1.1, 1.2, 1.3);
        kpsrPublisher->publish(event);
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

    while (cacheListener.counter < 5) {
        spdlog::info("publishing loop... ");
        kpsrPublisher->publish(event);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    ASSERT_EQ(cacheListener.counter, 5);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->a, event.a);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->b, event.b);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->c, event.c);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->d, event.d);

    fromDDSProvider.unregisterFromTopic("kpsr_dds_codegen_test_topic1", &dataReader);
}

