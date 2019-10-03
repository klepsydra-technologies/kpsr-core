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
