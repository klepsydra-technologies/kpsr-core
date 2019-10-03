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

#include <klepsydra/core/callback_handler.h>
#include <klepsydra/core/event_emitter_subscriber.h>
#include <klepsydra/core/event_emitter_publisher.h>
#include <klepsydra/core/cache_listener.h>

#include "gtest/gtest.h"

class TestRequest {
public:
    int id;
    std::string message;
};

class TestReply {
public:
    int id;
    bool ack;
};

TEST(CallbackHandler, BasicTest) {
    kpsr::EventEmitter eventEmitter;
    kpsr::EventEmitterSubscriber<TestRequest> requestSubcriber(nullptr, eventEmitter, "request");
    kpsr::EventEmitterPublisher<TestRequest> requestPublisher(nullptr, "request", eventEmitter, 0, nullptr, nullptr);

    kpsr::EventEmitterSubscriber<TestReply> replySubcriber(nullptr, eventEmitter, "reply");
    kpsr::EventEmitterPublisher<TestReply> replyPublisher(nullptr, "reply", eventEmitter, 0, nullptr, nullptr);

    kpsr::CallbackHandler<TestRequest, TestReply> callbackHandler("test", &requestPublisher, &replySubcriber,
                                                                  [] (const TestRequest & request, const TestReply & reply) {
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
