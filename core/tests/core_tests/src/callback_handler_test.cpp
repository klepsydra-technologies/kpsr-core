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

#include <klepsydra/core/cache_listener.h>
#include <klepsydra/core/callback_handler.h>
#include <klepsydra/core/event_emitter_publisher.h>
#include <klepsydra/core/event_emitter_subscriber.h>
#include <klepsydra/core/safe_event_emitter.h>

#include "gtest/gtest.h"

class TestRequest
{
public:
    int id;
    std::string message;
};

class TestReply
{
public:
    int id;
    bool ack;
};

TEST(CallbackHandler, BasicTest)
{
    std::shared_ptr<kpsr::EventEmitterInterface<std::shared_ptr<const TestRequest>>> eventEmitter1 =
        std::make_shared<kpsr::SafeEventEmitter<std::shared_ptr<const TestRequest>>>();
    kpsr::EventEmitterSubscriber<TestRequest> requestSubcriber(nullptr, eventEmitter1, "request");
    kpsr::EventEmitterPublisher<TestRequest> requestPublisher(nullptr,
                                                              "request",
                                                              eventEmitter1,
                                                              0,
                                                              nullptr,
                                                              nullptr);

    std::shared_ptr<kpsr::EventEmitterInterface<std::shared_ptr<const TestReply>>> eventEmitter2 =
        std::make_shared<kpsr::SafeEventEmitter<std::shared_ptr<const TestReply>>>();
    kpsr::EventEmitterSubscriber<TestReply> replySubcriber(nullptr, eventEmitter2, "reply");
    kpsr::EventEmitterPublisher<TestReply> replyPublisher(nullptr,
                                                          "reply",
                                                          eventEmitter2,
                                                          0,
                                                          nullptr,
                                                          nullptr);

    kpsr::CallbackHandler<TestRequest, TestReply> callbackHandler("test",
                                                                  &requestPublisher,
                                                                  &replySubcriber,
                                                                  [](const TestRequest &request,
                                                                     const TestReply &reply) {
                                                                      return request.id == reply.id;
                                                                  });

    kpsr::mem::CacheListener<TestReply> replyListener;

    TestRequest request;
    request.id = 1;
    request.message = "hola";

    TestReply reply;
    reply.id = 1;
    reply.ack = true;

    callbackHandler.requestAndReply(request, replyListener.cacheListenerFunction);
    replyPublisher.publish(reply);
    reply.ack = false;
    replyPublisher.publish(reply);

    ASSERT_TRUE(replyListener.getLastReceivedEvent()->ack);

    request.id = 2;
    request.message = "hola";

    reply.id = 2;
    reply.ack = false;

    callbackHandler.requestAndReply(request, replyListener.cacheListenerFunction);
    replyPublisher.publish(reply);

    ASSERT_FALSE(replyListener.getLastReceivedEvent()->ack);
}
