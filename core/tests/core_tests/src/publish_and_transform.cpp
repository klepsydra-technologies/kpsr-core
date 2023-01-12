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

#include <klepsydra/core/cache_listener.h>
#include <klepsydra/core/event_emitter_publisher.h>
#include <klepsydra/core/event_emitter_subscriber.h>
#include <klepsydra/core/event_transform_forwarder.h>
#include <klepsydra/core/safe_event_emitter.h>

#include "gtest/gtest.h"

class RawMessage
{
public:
    std::string message1;
    std::string message2;
    std::string message3;
};

TEST(EventTransformForwarder, BasicTest)
{
    std::shared_ptr<kpsr::EventEmitterInterface<std::shared_ptr<const RawMessage>>> eventEmitter1 =
        std::make_shared<kpsr::SafeEventEmitter<std::shared_ptr<const RawMessage>>>();
    kpsr::EventEmitterSubscriber<RawMessage> rawSubcriber(nullptr, eventEmitter1, "raw");
    kpsr::EventEmitterPublisher<RawMessage> rawPublisher(nullptr,
                                                         "raw",
                                                         eventEmitter1,
                                                         0,
                                                         nullptr,
                                                         nullptr);

    std::shared_ptr<kpsr::EventEmitterInterface<std::shared_ptr<const std::string>>> eventEmitter2 =
        std::make_shared<kpsr::SafeEventEmitter<std::shared_ptr<const std::string>>>();
    kpsr::EventEmitterSubscriber<std::string> processedSubcriber(nullptr,
                                                                 eventEmitter2,
                                                                 "processed");
    kpsr::EventEmitterPublisher<std::string> processedPublisher(nullptr,
                                                                "processed",
                                                                eventEmitter2,
                                                                0,
                                                                nullptr,
                                                                nullptr);

    kpsr::EventTransformForwarder<RawMessage, std::string>
        eventTransformer([](const RawMessage &event,
                            std::string &transformed) { transformed = event.message2; },
                         &processedPublisher);

    rawSubcriber.registerListener("transformer", eventTransformer.forwarderListenerFunction);

    kpsr::mem::CacheListener<std::string> processedListener;
    processedSubcriber.registerListener("test", processedListener.cacheListenerFunction);

    RawMessage raw;
    raw.message1 = "hola";
    raw.message2 = "hello";
    raw.message3 = "hallo";

    rawPublisher.publish(raw);

    ASSERT_EQ(*processedListener.getLastReceivedEvent(), "hello");
}
