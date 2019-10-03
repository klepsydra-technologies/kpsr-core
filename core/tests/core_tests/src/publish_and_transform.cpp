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

#include <klepsydra/core/event_transform_forwarder.h>
#include <klepsydra/core/event_emitter_subscriber.h>
#include <klepsydra/core/event_emitter_publisher.h>
#include <klepsydra/core/cache_listener.h>

#include "gtest/gtest.h"

class RawMessage {
public:
    std::string message1;
    std::string message2;
    std::string message3;
};

TEST(EventTransformForwarder, BasicTest) {
    kpsr::EventEmitter eventEmitter;
    kpsr::EventEmitterSubscriber<RawMessage> rawSubcriber(nullptr, eventEmitter, "raw");
    kpsr::EventEmitterPublisher<RawMessage> rawPublisher(nullptr, "raw", eventEmitter, 0, nullptr, nullptr);

    kpsr::EventEmitterSubscriber<std::string> processedSubcriber(nullptr, eventEmitter, "processed");
    kpsr::EventEmitterPublisher<std::string> processedPublisher(nullptr, "processed", eventEmitter, 0, nullptr, nullptr);

    kpsr::EventTransformForwarder<RawMessage, std::string> eventTransformer(
                [] (const RawMessage & event, std::string & transformed) {
        transformed = event.message2;
    }, &processedPublisher);

    rawSubcriber.registerListener("transformer", eventTransformer.forwarderListenerFunction);

    kpsr::mem::CacheListener<std::string> processedListener;
    processedSubcriber.registerListener("test", processedListener.cacheListenerFunction);

    RawMessage raw;
    raw.message1 = "hola";
    raw.message2 = "hello";
    raw.message3 = "hallo";

    rawPublisher.publish(raw);

    ASSERT_EQ(* processedListener.getLastReceivedEvent(), "hello");
}
