// Copyright 2023 Klepsydra Technologies AG
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <string>

#include "std_msgs/String.h"
#include <gtest/gtest.h>

#include <klepsydra/core/cache_listener.h>
#include <klepsydra/core/event_emitter_middleware_provider.h>

#include <kpsr_ros_serialization/primitive_type_ros_mapper.h>

#include <kpsr_ros_core/from_ros_middleware_provider.h>
#include <kpsr_ros_core/to_ros_middleware_provider.h>

#include "composition_type_related_ros_mapper.h"

TEST(KpsrRosCodegeTest, compositionTypeRelatedMapperTest)
{
    int argc = 0;
    char **argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_codegen_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::CompositionTypeRelated>
        basicProvider(nullptr, "test", 0, nullptr, nullptr);

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider
        .registerToTopic<kpsr::codegen::CompositionTypeRelated,
                         kpsr_ros_codegen::CompositionTypeRelated>("kpsr_ros_codegen_test_topic1",
                                                                   10,
                                                                   basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::codegen::CompositionTypeRelated> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener",
                                                    cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

    ros::Publisher stringPublisher = nodeHandle.advertise<kpsr_ros_codegen::CompositionTypeRelated>(
        "kpsr_ros_codegen_test_topic1", 10, true);

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<kpsr::codegen::CompositionTypeRelated> *kpsrPublisher =
        toRosProvider.getToMiddlewareChannel<kpsr::codegen::CompositionTypeRelated,
                                             kpsr_ros_codegen::CompositionTypeRelated>(
            "kpsr_ros_codegen_test_topic1", 1, nullptr, stringPublisher);

    unsigned short seq = 0;

    int maxNumAttempts = 10;
    int numAttempts = 0;
    while ((numAttempts < maxNumAttempts) && (0 == stringPublisher.getNumSubscribers())) {
        numAttempts++;
        rate.sleep();
    }
    ASSERT_LE(numAttempts, maxNumAttempts);

    kpsr::codegen::CompositionTypeRelated event;
    for (int i = 0; i < 5; i++) {
        event.seq = seq++;
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

        event.positionArray = {{kpsr::geometry::Vector3(seq++, 0.1, 0.2, 0.3),
                                kpsr::geometry::Vector3(seq++, 1.1, 1.2, 1.3)}};
        event.positionVector = {kpsr::geometry::Vector3(seq++, 0.1, 0.2, 0.3),
                                kpsr::geometry::Vector3(seq++, 1.1, 1.2, 1.3)};
        event.positionVectorPointer = {new kpsr::geometry::Vector3(seq++, 0.1, 0.2, 0.3),
                                       new kpsr::geometry::Vector3(seq++, 1.1, 1.2, 1.3)};
        event.positionVectorSharedPtr = {std::shared_ptr<kpsr::geometry::Vector3>(
                                             new kpsr::geometry::Vector3(seq++, 0.1, 0.2, 0.3)),
                                         std::shared_ptr<kpsr::geometry::Vector3>(
                                             new kpsr::geometry::Vector3(seq++, 1.1, 1.2, 1.3))};

        event.quaternionArray = {{kpsr::codegen::Vector4(0.1, 0.2, 0.3, 0.4),
                                  kpsr::codegen::Vector4(1.1, 1.2, 1.3, 1.4)}};
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
        kpsrPublisher->publish(event);
        ros::spinOnce();
        rate.sleep();
    }

    numAttempts = 0;
    while ((numAttempts < maxNumAttempts) && ros::ok()) {
        numAttempts++;
        ros::spinOnce();
        rate.sleep();
    }

    ASSERT_EQ(cacheListener.counter, 5);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->newEnum, event.newEnum);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->newEnumArray, event.newEnumArray);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->newEnumVector, event.newEnumVector);
    ASSERT_EQ(*cacheListener.getLastReceivedEvent()->newEnumVectorPointer[0],
              *event.newEnumVectorPointer[0]);
    ASSERT_EQ(*cacheListener.getLastReceivedEvent()->newEnumVectorSharedPtr[0].get(),
              *event.newEnumVectorSharedPtr[0].get());

    ASSERT_EQ(cacheListener.getLastReceivedEvent()->oldEnum, event.oldEnum);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->oldEnumArray, event.oldEnumArray);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->oldEnumVector[0], event.oldEnumVector[0]);
    ASSERT_EQ(*cacheListener.getLastReceivedEvent()->oldEnumVectorPointer[0],
              *event.oldEnumVectorPointer[0]);
    ASSERT_EQ(*cacheListener.getLastReceivedEvent()->oldEnumVectorSharedPtr[0].get(),
              *event.oldEnumVectorSharedPtr[0].get());

    ASSERT_EQ(cacheListener.getLastReceivedEvent()->positionArray[0].x, event.positionArray[0].x);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->positionVector[0].x, event.positionVector[0].x);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->positionVectorPointer[0]->x,
              event.positionVectorPointer[0]->x);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->positionVectorSharedPtr[0]->x,
              event.positionVectorSharedPtr[0]->x);

    ASSERT_EQ(cacheListener.getLastReceivedEvent()->quaternionArray[0].a,
              event.quaternionArray[0].a);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->quaternionVector[0].a,
              event.quaternionVector[0].a);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->quaternionVectorPointer[0]->a,
              event.quaternionVectorPointer[0]->a);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->quaternionVectorSharedPtr[0]->a,
              event.quaternionVectorSharedPtr[0]->a);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->quat.x, event.quat.x);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->gpsData.altitude, event.gpsData.altitude);
}
