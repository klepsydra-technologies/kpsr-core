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


#include <math.h>
#include <stdio.h>
#include <thread>
#include <unistd.h>

#include <fstream>
#include <sstream>

#include "gtest/gtest.h"

#include <klepsydra/core/cache_listener.h>
#include <klepsydra/core/event_emitter_middleware_provider.h>

#include <klepsydra/codegen/gps.h>
#include <klepsydra/codegen/header.h>
#include <klepsydra/codegen/imu.h>
#include <klepsydra/codegen/quaternion.h>
#include <klepsydra/codegen/vector3.h>

TEST(CodegenPocoTests, HeaderPocoTest)
{
    kpsr::EventEmitterMiddlewareProvider<kpsr::geometry::Header> provider(nullptr,
                                                                          "event",
                                                                          0,
                                                                          nullptr,
                                                                          nullptr);

    kpsr::mem::CacheListener<kpsr::geometry::Header> eventListener;

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    kpsr::geometry::Header event(1, "hola");
    provider.getPublisher()->publish(event);

    ASSERT_EQ(event.seq, eventListener.getLastReceivedEvent()->seq);
    ASSERT_EQ(event.frame_id, eventListener.getLastReceivedEvent()->frame_id);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->totalProcessed, 1);
}

TEST(CodegenPocoTests, GpsPocoTest)
{
    kpsr::EventEmitterMiddlewareProvider<kpsr::geometry::Gps> provider(nullptr,
                                                                       "event",
                                                                       0,
                                                                       nullptr,
                                                                       nullptr);

    kpsr::mem::CacheListener<kpsr::geometry::Gps> eventListener;

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    kpsr::geometry::Gps event(1, 1.0, 2.0, 3.0);
    provider.getPublisher()->publish(event);

    ASSERT_EQ(event.altitude, eventListener.getLastReceivedEvent()->altitude);
    ASSERT_EQ(event.latitude, eventListener.getLastReceivedEvent()->latitude);
    ASSERT_EQ(event.longitude, eventListener.getLastReceivedEvent()->longitude);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->totalProcessed, 1);
}

TEST(CodegenPocoTests, QuaternionPocoTest)
{
    kpsr::EventEmitterMiddlewareProvider<kpsr::geometry::Quaternion> provider(nullptr,
                                                                              "event",
                                                                              0,
                                                                              nullptr,
                                                                              nullptr);

    kpsr::mem::CacheListener<kpsr::geometry::Quaternion> eventListener;

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    kpsr::geometry::Quaternion event(0, 1.0, 2.0, 3.0, 4.0);
    provider.getPublisher()->publish(event);

    ASSERT_EQ(event.x, eventListener.getLastReceivedEvent()->x);
    ASSERT_EQ(event.y, eventListener.getLastReceivedEvent()->y);
    ASSERT_EQ(event.z, eventListener.getLastReceivedEvent()->z);
    ASSERT_EQ(event.w, eventListener.getLastReceivedEvent()->w);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->totalProcessed, 1);
}

TEST(CodegenPocoTests, Vector3PocoTest)
{
    kpsr::EventEmitterMiddlewareProvider<kpsr::geometry::Vector3> provider(nullptr,
                                                                           "event",
                                                                           0,
                                                                           nullptr,
                                                                           nullptr);

    kpsr::mem::CacheListener<kpsr::geometry::Vector3> eventListener;

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    kpsr::geometry::Vector3 event(1, 1.0, 2.0, 3.0);
    provider.getPublisher()->publish(event);

    ASSERT_EQ(event.x, eventListener.getLastReceivedEvent()->x);
    ASSERT_EQ(event.y, eventListener.getLastReceivedEvent()->y);
    ASSERT_EQ(event.z, eventListener.getLastReceivedEvent()->z);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->totalProcessed, 1);
}

TEST(CodegenPocoTests, ImuPocoTest)
{
    kpsr::EventEmitterMiddlewareProvider<kpsr::geometry::Imu> provider(nullptr,
                                                                       "event",
                                                                       0,
                                                                       nullptr,
                                                                       nullptr);

    kpsr::mem::CacheListener<kpsr::geometry::Imu> eventListener;

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    kpsr::geometry::Quaternion orientation(1, 0.1, 0.2, 0.3, 0.4);
    std::array<double, 9> orientation_covariance{{1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9}};
    kpsr::geometry::Vector3 angular_velocity(2, 0.5, 0.6, 0.7);
    std::array<double, 9> angular_velocity_covariance{{2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9}};
    kpsr::geometry::Vector3 linear_acceleration(3, 0.8, 0.9, 1.0);
    std::array<double, 9> linear_acceleration_covariance{
        {3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9}};

    kpsr::geometry::Imu event(1,
                              orientation,
                              orientation_covariance,
                              angular_velocity,
                              angular_velocity_covariance,
                              linear_acceleration,
                              linear_acceleration_covariance);
    provider.getPublisher()->publish(event);

    ASSERT_EQ(event.orientation.x, eventListener.getLastReceivedEvent()->orientation.x);
    ASSERT_EQ(event.orientation.y, eventListener.getLastReceivedEvent()->orientation.y);
    ASSERT_EQ(event.orientation.z, eventListener.getLastReceivedEvent()->orientation.z);
    ASSERT_EQ(event.orientation.w, eventListener.getLastReceivedEvent()->orientation.w);

    ASSERT_EQ(event.angular_velocity.x, eventListener.getLastReceivedEvent()->angular_velocity.x);
    ASSERT_EQ(event.angular_velocity.y, eventListener.getLastReceivedEvent()->angular_velocity.y);
    ASSERT_EQ(event.angular_velocity.z, eventListener.getLastReceivedEvent()->angular_velocity.z);

    for (int i = 0; i < 9; i++) {
        ASSERT_EQ(event.orientation_covariance[i],
                  eventListener.getLastReceivedEvent()->orientation_covariance[i]);
        ASSERT_EQ(event.angular_velocity_covariance[i],
                  eventListener.getLastReceivedEvent()->angular_velocity_covariance[i]);
        ASSERT_EQ(event.linear_acceleration_covariance[i],
                  eventListener.getLastReceivedEvent()->linear_acceleration_covariance[i]);
    }

    ASSERT_EQ(event.linear_acceleration.x,
              eventListener.getLastReceivedEvent()->linear_acceleration.x);
    ASSERT_EQ(event.linear_acceleration.y,
              eventListener.getLastReceivedEvent()->linear_acceleration.y);
    ASSERT_EQ(event.linear_acceleration.z,
              eventListener.getLastReceivedEvent()->linear_acceleration.z);

    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->totalProcessed, 1);
}
