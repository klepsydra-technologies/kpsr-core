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

#include "inheritance_vector4_ros_mapper.h"

TEST(KpsrRosCodegeTest, inheritanceMapperMapperTest)
{
    int argc = 0;
    char **argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_codegen_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);
    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::InheritanceVector4> basicProvider(nullptr,
                                                                                          "test",
                                                                                          0,
                                                                                          nullptr,
                                                                                          nullptr);

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider
        .registerToTopic<kpsr::codegen::InheritanceVector4, kpsr_ros_codegen::InheritanceVector4>(
            "kpsr_ros_codegen_test_topic1", 10, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::codegen::InheritanceVector4> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener",
                                                    cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);
    ros::Publisher stringPublisher =
        nodeHandle.advertise<kpsr_ros_codegen::InheritanceVector4>("kpsr_ros_codegen_test_topic1",
                                                                   10,
                                                                   true);

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<kpsr::codegen::InheritanceVector4> *kpsrPublisher =
        toRosProvider.getToMiddlewareChannel<kpsr::codegen::InheritanceVector4,
                                             kpsr_ros_codegen::InheritanceVector4>(
            "kpsr_ros_codegen_test_topic1", 1, nullptr, stringPublisher);

    int maxNumAttempts = 10;
    int numAttempts = 0;
    while ((numAttempts < maxNumAttempts) && (0 == stringPublisher.getNumSubscribers())) {
        numAttempts++;
        rate.sleep();
    }
    ASSERT_LE(numAttempts, maxNumAttempts);

    {
        kpsr::codegen::InheritanceVector4 event(1.0, 1.1, 1.2, 1.3);
        kpsrPublisher->publish(event);
    }
    ros::spinOnce();
    rate.sleep();

    {
        kpsr::codegen::InheritanceVector4 event(2.0, 2.1, 2.2, 2.3);
        kpsrPublisher->publish(event);
    }
    ros::spinOnce();
    rate.sleep();

    {
        kpsr::codegen::InheritanceVector4 event(3.0, 3.1, 3.2, 3.3);
        kpsrPublisher->publish(event);
    }
    ros::spinOnce();
    rate.sleep();

    {
        kpsr::codegen::InheritanceVector4 event(4.0, 4.1, 4.2, 4.3);
        kpsrPublisher->publish(event);
    }
    ros::spinOnce();
    rate.sleep();

    kpsr::codegen::InheritanceVector4 event(5.0, 5.1, 5.2, 5.3);
    kpsrPublisher->publish(event);
    ros::spinOnce();
    rate.sleep();

    numAttempts = 0;
    while ((numAttempts < maxNumAttempts) && ros::ok()) {
        numAttempts++;
        ros::spinOnce();
        rate.sleep();
    }

    ASSERT_EQ(cacheListener.counter, 5);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->a, event.a);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->b, event.b);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->c, event.c);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->d, event.d);
}
