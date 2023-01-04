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

#include <math.h>
#include <stdio.h>
#include <thread>
#include <unistd.h>

#include <fstream>
#include <sstream>

#include "gtest/gtest.h"

#include <klepsydra/core/cache_listener.h>
#include <klepsydra/core/event_emitter_middleware_provider.h>

#include <klepsydra/codegen/composition_type_related.h>

TEST(CodegenPocoTests, CompositionRelatedPocoTest)
{
    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::CompositionTypeRelated> provider(nullptr,
                                                                                         "event",
                                                                                         0,
                                                                                         nullptr,
                                                                                         nullptr);

    kpsr::mem::CacheListener<kpsr::codegen::CompositionTypeRelated> eventListener;

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    kpsr::codegen::CompositionTypeRelated event;
    event.newEnum = kpsr::codegen::NewEnum::new1;
    event.newEnumArray = {{kpsr::codegen::NewEnum::new2, kpsr::codegen::NewEnum::new3}};
    event.newEnumVector = {kpsr::codegen::NewEnum::new1, kpsr::codegen::NewEnum::new2};
    event.newEnumVectorPointer = {new kpsr::codegen::NewEnum(kpsr::codegen::NewEnum::new1)};
    event.newEnumVectorSharedPtr = {std::shared_ptr<kpsr::codegen::NewEnum>(
        new kpsr::codegen::NewEnum(kpsr::codegen::NewEnum::new1))};

    event.oldEnum = kpsr::codegen::OldEnum::oldA;
    event.oldEnumArray = {{kpsr::codegen::OldEnum::oldA, kpsr::codegen::OldEnum::oldB}};
    event.oldEnumVector = {kpsr::codegen::OldEnum::oldA, kpsr::codegen::OldEnum::oldB};
    event.oldEnumVectorPointer = {new kpsr::codegen::OldEnum(kpsr::codegen::OldEnum::oldB)};
    event.oldEnumVectorSharedPtr = {std::shared_ptr<kpsr::codegen::OldEnum>(
        new kpsr::codegen::OldEnum(kpsr::codegen::OldEnum::oldA))};

    event.positionArray = {
        {kpsr::geometry::Vector3(1, 0.1, 0.2, 0.3), kpsr::geometry::Vector3(3, 1.1, 1.2, 1.3)}};
    event.positionVector = {kpsr::geometry::Vector3(2, 0.1, 0.2, 0.3),
                            kpsr::geometry::Vector3(4, 1.1, 1.2, 1.3)};
    event.positionVectorPointer = {new kpsr::geometry::Vector3(5, 0.1, 0.2, 0.3),
                                   new kpsr::geometry::Vector3(6, 1.1, 1.2, 1.3)};
    event.positionVectorSharedPtr = {std::shared_ptr<kpsr::geometry::Vector3>(
                                         new kpsr::geometry::Vector3(7, 0.1, 0.2, 0.3)),
                                     std::shared_ptr<kpsr::geometry::Vector3>(
                                         new kpsr::geometry::Vector3(8, 1.1, 1.2, 1.3))};

    event.quaternionArray = {
        {kpsr::codegen::Vector4(0.1, 0.2, 0.3, 0.4), kpsr::codegen::Vector4(1.1, 1.2, 1.3, 1.4)}};
    event.quaternionVector = {kpsr::codegen::Vector4(0.1, 0.2, 0.3, 0.4),
                              kpsr::codegen::Vector4(1.1, 1.2, 1.3, 1.4)};
    event.quaternionVectorPointer = {new kpsr::codegen::Vector4(0.1, 0.2, 0.3, 0.4),
                                     new kpsr::codegen::Vector4(1.1, 1.2, 1.3, 1.4)};
    event.quaternionVectorSharedPtr = {std::shared_ptr<kpsr::codegen::Vector4>(
                                           new kpsr::codegen::Vector4(0.1, 0.2, 0.3, 0.4)),
                                       std::shared_ptr<kpsr::codegen::Vector4>(
                                           new kpsr::codegen::Vector4(1.1, 1.2, 1.3, 1.4))};
    event.quat = kpsr::geometry::Quaternion(0, 1.0, 2.0, 3.0, 4.0);
    event.gpsData = kpsr::geometry::Gps(1, 1.0, 2.0, 3.0);
    provider.getPublisher()->publish(event);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    auto lastEventPtr = eventListener.getLastReceivedEvent();
    ASSERT_NE(lastEventPtr, nullptr);
    ASSERT_EQ(event.seq, lastEventPtr->seq);
    ASSERT_EQ(event.quat.x, lastEventPtr->quat.x);
    ASSERT_EQ(event.gpsData.altitude, lastEventPtr->gpsData.altitude);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->totalProcessed, 1);
}
