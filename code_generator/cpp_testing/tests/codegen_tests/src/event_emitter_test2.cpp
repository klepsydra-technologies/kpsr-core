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

#include <stdio.h>
#include <thread>
#include <unistd.h>
#include <math.h>

#include <sstream>
#include <fstream>

#include "gtest/gtest.h"

#include <klepsydra/core/event_emitter_middleware_provider.h>
#include <klepsydra/core/cache_listener.h>

#include <klepsydra/codegen/primitive_types_basic.h>
#include <klepsydra/codegen/primitive_types_array.h>
#include <klepsydra/codegen/primitive_types_vector.h>
#include <klepsydra/codegen/primitive_types_vector_shared_ptr.h>

TEST(CodegenPocoTests, PrimiteTypesAllPocoTest) {
    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::PrimitiveTypesBasic> provider(nullptr, "event", 0, nullptr, nullptr);

    kpsr::mem::CacheListener<kpsr::codegen::PrimitiveTypesBasic> eventListener;

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    kpsr::codegen::PrimitiveTypesBasic event;
    provider.getPublisher()->publish(event);

//    ASSERT_EQ(event.seq, eventListener.getLastReceivedEvent()->seq);
//    ASSERT_EQ(event.frame_id, eventListener.getLastReceivedEvent()->frame_id);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->_totalProcessed, 1);
}

TEST(CodegenPocoTests, PrimiteTypesArrayPocoTest) {
    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::PrimitiveTypesArray> provider(nullptr, "event", 0, nullptr, nullptr);

    kpsr::mem::CacheListener<kpsr::codegen::PrimitiveTypesArray> eventListener;

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    kpsr::codegen::PrimitiveTypesArray event;
    provider.getPublisher()->publish(event);

//    ASSERT_EQ(event.seq, eventListener.getLastReceivedEvent()->seq);
//    ASSERT_EQ(event.frame_id, eventListener.getLastReceivedEvent()->frame_id);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->_totalProcessed, 1);
}

TEST(CodegenPocoTests, PrimiteTypesVectorPocoTest) {
    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::PrimitiveTypesVector> provider(nullptr, "event", 0, nullptr, nullptr);

    kpsr::mem::CacheListener<kpsr::codegen::PrimitiveTypesVector> eventListener;

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    kpsr::codegen::PrimitiveTypesVector event;
    provider.getPublisher()->publish(event);

//    ASSERT_EQ(event.seq, eventListener.getLastReceivedEvent()->seq);
//    ASSERT_EQ(event.frame_id, eventListener.getLastReceivedEvent()->frame_id);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->_totalProcessed, 1);
}

TEST(CodegenPocoTests, PrimiteTypesVectorSharedPtrPocoTest) {
    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::PrimitiveTypesVectorSharedPtr> provider(nullptr, "event", 0, nullptr, nullptr);

    kpsr::mem::CacheListener<kpsr::codegen::PrimitiveTypesVectorSharedPtr> eventListener;

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    kpsr::codegen::PrimitiveTypesVectorSharedPtr event;
    provider.getPublisher()->publish(event);

//    ASSERT_EQ(event.seq, eventListener.getLastReceivedEvent()->seq);
//    ASSERT_EQ(event.frame_id, eventListener.getLastReceivedEvent()->frame_id);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->_totalProcessed, 1);
}
