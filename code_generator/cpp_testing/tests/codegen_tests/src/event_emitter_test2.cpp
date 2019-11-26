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
