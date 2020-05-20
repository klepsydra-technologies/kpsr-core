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

#include <klepsydra/codegen/dds/composition_type_related_dds_mapper.h>

TEST(KpsrDdsCodegenTest5, compositionTypeRelatedMapperTest) {
    dds::domain::DomainParticipant dp(0);
    dds::pub::Publisher publisher(dp);
    dds::sub::Subscriber subscriber(dp);

    dds::topic::Topic<kpsr_dds_codegen::CompositionTypeRelatedData> topic(dp, "header_data_test");
    dds::pub::DataWriter<kpsr_dds_codegen::CompositionTypeRelatedData> dataWriter(publisher, topic);
    dds::sub::DataReader<kpsr_dds_codegen::CompositionTypeRelatedData> dataReader(subscriber, topic);

    kpsr::dds_mdlw::FromDDSMiddlewareProvider fromDDSProvider;
    kpsr::dds_mdlw::ToDDSMiddlewareProvider toDDSProvider(nullptr);

    kpsr::Publisher<kpsr::codegen::CompositionTypeRelated> * kpsrPublisher =
            toDDSProvider.getToMiddlewareChannel<kpsr::codegen::CompositionTypeRelated, kpsr_dds_codegen::CompositionTypeRelatedData>("kpsr_ros_codegen_test_topicA", 1, nullptr, &dataWriter);

    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::CompositionTypeRelated> basicProvider(nullptr, "test", 0, nullptr, nullptr);

    fromDDSProvider.registerToTopic("kpsr_dds_codegen_test_topic1", &dataReader, true, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::codegen::CompositionTypeRelated> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

    unsigned short seq = 0;
    kpsr::codegen::CompositionTypeRelated event;
    for (int i = 0; i < 5; i++) {
        event.seq = seq++;
        event.newEnum = kpsr::codegen::NewEnum::new1;
        event.newEnumArray = {{kpsr::codegen::NewEnum::new2, kpsr::codegen::NewEnum::new3}};
        event.newEnumVector = {kpsr::codegen::NewEnum::new1, kpsr::codegen::NewEnum::new2};
        event.newEnumVectorPointer = {new kpsr::codegen::NewEnum(kpsr::codegen::NewEnum::new1)};
        event.newEnumVectorSharedPtr = {std::shared_ptr<kpsr::codegen::NewEnum>(new kpsr::codegen::NewEnum(kpsr::codegen::NewEnum::new1))};

        event.oldEnum = kpsr::codegen::OldEnum::oldA;
        event.oldEnumArray = {{kpsr::codegen::OldEnum::oldA, kpsr::codegen::OldEnum::oldB}};
        event.oldEnumVector = {kpsr::codegen::OldEnum::oldA, kpsr::codegen::OldEnum::oldB};
        event.oldEnumVectorPointer = {new kpsr::codegen::OldEnum(kpsr::codegen::OldEnum::oldB)};
        event.oldEnumVectorSharedPtr = {std::shared_ptr<kpsr::codegen::OldEnum>(new kpsr::codegen::OldEnum(kpsr::codegen::OldEnum::oldA))};

        event.positionArray = {{ kpsr::geometry::Vector3(seq++, 0.1, 0.2, 0.3), kpsr::geometry::Vector3(seq++, 1.1, 1.2, 1.3) }};
        event.positionVector = { kpsr::geometry::Vector3(seq++, 0.1, 0.2, 0.3), kpsr::geometry::Vector3(seq++, 1.1, 1.2, 1.3) };
        event.positionVectorPointer = { new kpsr::geometry::Vector3(seq++, 0.1, 0.2, 0.3), new kpsr::geometry::Vector3(seq++, 1.1, 1.2, 1.3) };
        event.positionVectorSharedPtr = { std::shared_ptr<kpsr::geometry::Vector3>(new kpsr::geometry::Vector3(seq++, 0.1, 0.2, 0.3)),
                                          std::shared_ptr<kpsr::geometry::Vector3>(new kpsr::geometry::Vector3(seq++, 1.1, 1.2, 1.3)) };

        event.quaternionArray = {{ kpsr::codegen::Vector4(0.1, 0.2, 0.3, 0.4), kpsr::codegen::Vector4(1.1, 1.2, 1.3, 1.4) }};
        event.quaternionVector = { kpsr::codegen::Vector4(0.1, 0.2, 0.3, 0.4), kpsr::codegen::Vector4(1.1, 1.2, 1.3, 1.4) };
        event.quaternionVectorPointer = { new kpsr::codegen::Vector4(0.1, 0.2, 0.3, 0.4), new kpsr::codegen::Vector4(1.1, 1.2, 1.3, 1.4) };
        event.quaternionVectorSharedPtr = { std::shared_ptr<kpsr::codegen::Vector4>(new kpsr::codegen::Vector4(0.1, 0.2, 0.3, 0.4)),
                                          std::shared_ptr<kpsr::codegen::Vector4>(new kpsr::codegen::Vector4(1.1, 1.2, 1.3, 1.4)) };
        event.quat = kpsr::geometry::Quaternion(0, 1.0, 2.0, 3.0, 4.0);
        event.gpsData = kpsr::geometry::Gps(1, 1.0, 2.0, 3.0);
        kpsrPublisher->publish(event);
    }

    while (cacheListener.counter < 5) {
        spdlog::info("publishing loop... ");
        kpsrPublisher->publish(event);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    ASSERT_EQ(cacheListener.counter, 5);
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

    fromDDSProvider.unregisterFromTopic("kpsr_dds_codegen_test_topic1", &dataReader);
}

