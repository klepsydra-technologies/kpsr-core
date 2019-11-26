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

#include <klepsydra/codegen/inheritance_vector4.h>

TEST(CodegenPocoTests, InheritanceVector4PocoTest) {
    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::InheritanceVector4> provider(nullptr, "event", 0, nullptr, nullptr);

    kpsr::mem::CacheListener<kpsr::codegen::InheritanceVector4> eventListener;

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    kpsr::codegen::InheritanceVector4 event(1.0, 2.0, 3.0, 4.0);
    provider.getPublisher()->publish(event);

    ASSERT_EQ(event.a, eventListener.getLastReceivedEvent()->a);
    ASSERT_EQ(event.b, eventListener.getLastReceivedEvent()->b);
    ASSERT_EQ(event.c, eventListener.getLastReceivedEvent()->c);
    ASSERT_EQ(event.d, eventListener.getLastReceivedEvent()->d);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->_totalProcessed, 1);
}
