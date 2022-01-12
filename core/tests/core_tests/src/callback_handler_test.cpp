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
