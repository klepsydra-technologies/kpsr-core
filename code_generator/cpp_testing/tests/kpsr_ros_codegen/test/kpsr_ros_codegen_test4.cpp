/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
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

#include "inheritance_vector4_ros_mapper.h"

TEST(KpsrRosCodegeTest, inheritanceMapperMapperTest) {

    int argc = 0;
    char ** argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_codegen_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    ros::Publisher stringPublisher = nodeHandle.advertise<kpsr_ros_codegen::InheritanceVector4>("kpsr_ros_codegen_test_topic1", 1);

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<kpsr::codegen::InheritanceVector4> * kpsrPublisher = toRosProvider.getToMiddlewareChannel<kpsr::codegen::InheritanceVector4, kpsr_ros_codegen::InheritanceVector4>("kpsr_ros_codegen_test_topic1", 1, nullptr, stringPublisher);

    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::InheritanceVector4> basicProvider(nullptr, "test", 0, nullptr, nullptr);

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider.registerToTopic<kpsr::codegen::InheritanceVector4, kpsr_ros_codegen::InheritanceVector4>("kpsr_ros_codegen_test_topic1", 1, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::codegen::InheritanceVector4> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

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

    while (cacheListener.counter < 5) {
        ros::spinOnce();
        rate.sleep();
    }

    ASSERT_EQ(cacheListener.counter, 5);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->a, event.a);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->b, event.b);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->c, event.c);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->d, event.d);
}

