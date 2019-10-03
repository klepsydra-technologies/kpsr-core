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

#include <klepsydra/codegen/composition_type.h>

TEST(CodegenPocoTests, CompositionPocoTest) {
    kpsr::EventEmitterMiddlewareProvider<kpsr::codegen::CompositionType> provider(nullptr, "event", 0, nullptr, nullptr);

    kpsr::mem::CacheListener<kpsr::codegen::CompositionType> eventListener;

    provider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

    kpsr::codegen::CompositionType event;
    event.newEnum = kpsr::codegen::NewEnum::new1;
    event.newEnumArray = {{kpsr::codegen::NewEnum::new2, kpsr::codegen::NewEnum::new3}};
    event.newEnumVector = {kpsr::codegen::NewEnum::new1, kpsr::codegen::NewEnum::new2};
    event.newEnumVectorPointer = {new kpsr::codegen::NewEnum(kpsr::codegen::NewEnum::new1)};
    event.newEnumVectorSharedPtr = {std::shared_ptr<kpsr::codegen::NewEnum>(new kpsr::codegen::NewEnum(kpsr::codegen::NewEnum::new1))};

    event.oldEnum = kpsr::codegen::OldEnum::oldA;
    event.oldEnumArray = {{kpsr::codegen::OldEnum::oldA, kpsr::codegen::OldEnum::oldB}};
    event.oldEnumVector = {kpsr::codegen::OldEnum::oldA, kpsr::codegen::OldEnum::oldB};
    event.oldEnumVectorPointer = {new kpsr::codegen::OldEnum(kpsr::codegen::OldEnum::oldB)};
    event.oldEnumVectorSharedPtr = {std::shared_ptr<kpsr::codegen::OldEnum>(new kpsr::codegen::OldEnum(kpsr::codegen::OldEnum::oldA))};

    event.positionArray = {{ kpsr::geometry::Vector3(1, 0.1, 0.2, 0.3), kpsr::geometry::Vector3(3, 1.1, 1.2, 1.3) }};
    event.positionVector = { kpsr::geometry::Vector3(2, 0.1, 0.2, 0.3), kpsr::geometry::Vector3(4, 1.1, 1.2, 1.3) };
    event.positionVectorPointer = { new kpsr::geometry::Vector3(5, 0.1, 0.2, 0.3), new kpsr::geometry::Vector3(6, 1.1, 1.2, 1.3) };
    event.positionVectorSharedPtr = { std::shared_ptr<kpsr::geometry::Vector3>(new kpsr::geometry::Vector3(7, 0.1, 0.2, 0.3)),
                                      std::shared_ptr<kpsr::geometry::Vector3>(new kpsr::geometry::Vector3(8, 1.1, 1.2, 1.3)) };

    event.quaternionArray = {{ kpsr::codegen::Vector4(0.1, 0.2, 0.3, 0.4), kpsr::codegen::Vector4(1.1, 1.2, 1.3, 1.4) }};
    event.quaternionVector = { kpsr::codegen::Vector4(0.1, 0.2, 0.3, 0.4), kpsr::codegen::Vector4(1.1, 1.2, 1.3, 1.4) };
    event.quaternionVectorPointer = { new kpsr::codegen::Vector4(0.1, 0.2, 0.3, 0.4), new kpsr::codegen::Vector4(1.1, 1.2, 1.3, 1.4) };
    event.quaternionVectorSharedPtr = { std::shared_ptr<kpsr::codegen::Vector4>(new kpsr::codegen::Vector4(0.1, 0.2, 0.3, 0.4)),
                                      std::shared_ptr<kpsr::codegen::Vector4>(new kpsr::codegen::Vector4(1.1, 1.2, 1.3, 1.4)) };

    provider.getPublisher()->publish(event);

//    ASSERT_EQ(event.seq, eventListener.getLastReceivedEvent()->seq);
//    ASSERT_EQ(event.frame_id, eventListener.getLastReceivedEvent()->frame_id);
    ASSERT_EQ(provider.getSubscriber()->getSubscriptionStats("cacheListener")->_totalProcessed, 1);
}
