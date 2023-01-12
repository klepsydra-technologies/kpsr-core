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

#include <spdlog/sinks/ostream_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <klepsydra/core/smart_object_pool.h>
#include <klepsydra/high_performance/data_multiplexer_middleware_provider.h>
#include <klepsydra/high_performance/event_loop_middleware_provider.h>

#include "gtest/gtest.h"

TEST(StreamingTest, BasictTest)
{
    kpsr::SmartObjectPool<std::string> pool("pool", 4);
    kpsr::high_performance::DataMultiplexerMiddlewareProvider<std::string, 4> dm(nullptr, "dm");
    kpsr::high_performance::EventLoopMiddlewareProvider<8> el1(nullptr, "el1");
    kpsr::high_performance::EventLoopMiddlewareProvider<4> el2(nullptr, "el2");

    kpsr::Subscriber<std::string> *sub1 = el1.getSubscriber<std::string>("pubsub1");
    kpsr::Publisher<std::string> *pub1 = el1.getPublisher<std::string>("pubsub1",
                                                                       4,
                                                                       nullptr,
                                                                       nullptr);

    kpsr::Subscriber<std::string> *sub2 = dm.getSubscriber("dm.subscriber");
    kpsr::Publisher<std::string> *pub2 = dm.getPublisher();

    kpsr::Subscriber<std::string> *sub3 = el2.getSubscriber<std::string>("pubsub3");
    kpsr::Publisher<std::string> *pub3 = el2.getPublisher<std::string>("pubsub3",
                                                                       4,
                                                                       nullptr,
                                                                       nullptr);

    el1.start();
    el2.start();

    int eventsReceived = 0;
    sub1->registerListener("listener1", [&](const std::string &event) { pub2->publish(event); });

    sub2->registerListener("listener2", [&](const std::string &event) { pub3->publish(event); });

    sub3->registerListener("listener3", [&](const std::string &event) {
        std::cout << "listener3. event received: " << event << std::endl;
        eventsReceived++;
    });

    std::thread thread([&]() {
        for (int i = 0; i < 10; i++) {
            std::shared_ptr<std::string> newEvent = pool.acquire();
            *(newEvent.get()) = "event_" + std::to_string(i);
            pub1->publish(newEvent);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    });

    thread.join();

    sub3->removeListener("listener3");
    sub2->removeListener("listener2");
    sub1->removeListener("listener1");

    el2.stop();
    el1.stop();

    ASSERT_EQ(eventsReceived, 10);
}
