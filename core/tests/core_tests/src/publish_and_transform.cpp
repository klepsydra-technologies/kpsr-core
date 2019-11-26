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
