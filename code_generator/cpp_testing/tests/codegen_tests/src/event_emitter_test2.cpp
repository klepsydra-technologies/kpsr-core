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

#include <klepsydra/codegen/primitive_types_array.h>
#include <klepsydra/codegen/primitive_types_basic.h>
#include <klepsydra/codegen/primitive_types_vector.h>
#include <klepsydra/codegen/primitive_types_vector_shared_ptr.h>

TEST(CodegenPocoTests, PrimiteTypesAllPocoTest)
{
    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::PrimitiveTypesBasic> provider(nullptr,
                                                                                      "event",
                                                                                      0,
                                                                                      nullptr,
                                                                                      nullptr);

    kpsr::mem::CacheListener<kpsr::codegen::PrimitiveTypesBasic> eventListener;

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    kpsr::codegen::PrimitiveTypesBasic event;
    provider.getPublisher()->publish(event);

    //    ASSERT_EQ(event.seq, eventListener.getLastReceivedEvent()->seq);
    //    ASSERT_EQ(event.frame_id, eventListener.getLastReceivedEvent()->frame_id);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->totalProcessed, 1);
}

TEST(CodegenPocoTests, PrimiteTypesArrayPocoTest)
{
    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::PrimitiveTypesArray> provider(nullptr,
                                                                                      "event",
                                                                                      0,
                                                                                      nullptr,
                                                                                      nullptr);

    kpsr::mem::CacheListener<kpsr::codegen::PrimitiveTypesArray> eventListener;

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    kpsr::codegen::PrimitiveTypesArray event;
    provider.getPublisher()->publish(event);

    //    ASSERT_EQ(event.seq, eventListener.getLastReceivedEvent()->seq);
    //    ASSERT_EQ(event.frame_id, eventListener.getLastReceivedEvent()->frame_id);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->totalProcessed, 1);
}

TEST(CodegenPocoTests, PrimiteTypesVectorPocoTest)
{
    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::PrimitiveTypesVector> provider(nullptr,
                                                                                       "event",
                                                                                       0,
                                                                                       nullptr,
                                                                                       nullptr);

    kpsr::mem::CacheListener<kpsr::codegen::PrimitiveTypesVector> eventListener;

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    kpsr::codegen::PrimitiveTypesVector event;
    provider.getPublisher()->publish(event);

    //    ASSERT_EQ(event.seq, eventListener.getLastReceivedEvent()->seq);
    //    ASSERT_EQ(event.frame_id, eventListener.getLastReceivedEvent()->frame_id);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->totalProcessed, 1);
}

TEST(CodegenPocoTests, PrimiteTypesVectorSharedPtrPocoTest)
{
    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::PrimitiveTypesVectorSharedPtr>
        provider(nullptr, "event", 0, nullptr, nullptr);

    kpsr::mem::CacheListener<kpsr::codegen::PrimitiveTypesVectorSharedPtr> eventListener;

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    kpsr::codegen::PrimitiveTypesVectorSharedPtr event;
    provider.getPublisher()->publish(event);

    //    ASSERT_EQ(event.seq, eventListener.getLastReceivedEvent()->seq);
    //    ASSERT_EQ(event.frame_id, eventListener.getLastReceivedEvent()->frame_id);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->totalProcessed, 1);
}
