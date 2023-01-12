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

#include <klepsydra/codegen/inheritance_vector4.h>

TEST(CodegenPocoTests, InheritanceVector4PocoTest)
{
    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::InheritanceVector4> provider(nullptr,
                                                                                     "event",
                                                                                     0,
                                                                                     nullptr,
                                                                                     nullptr);

    kpsr::mem::CacheListener<kpsr::codegen::InheritanceVector4> eventListener;

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    kpsr::codegen::InheritanceVector4 event(1.0, 2.0, 3.0, 4.0);
    provider.getPublisher()->publish(event);

    ASSERT_EQ(event.a, eventListener.getLastReceivedEvent()->a);
    ASSERT_EQ(event.b, eventListener.getLastReceivedEvent()->b);
    ASSERT_EQ(event.c, eventListener.getLastReceivedEvent()->c);
    ASSERT_EQ(event.d, eventListener.getLastReceivedEvent()->d);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->totalProcessed, 1);
}
