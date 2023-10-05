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

#include "gtest/gtest.h"

#include <klepsydra/core/core_container.h>
#include <klepsydra/core/unsafe_single_listener_event_emitter.h>

TEST(UnsafeSingleListenerEventEmitterTest, BasicTests)
{
    kpsr::UnsafeSingleListenerEventEmitter<std::string> eventEmitter;

    ASSERT_NO_THROW(eventEmitter.removeListener(nullptr, 0));

    std::string eventCopy;
    unsigned int listenerId = eventEmitter.on(nullptr,
                                              "listener",
                                              "subscriber",
                                              [&eventCopy](const std::string &event) {
                                                  eventCopy = event;
                                              });
    ASSERT_EQ(listenerId, 0);

    eventEmitter.emitEvent("subscriber", 0, "hola");
    ASSERT_EQ(eventCopy, "hola");

    eventEmitter.emitEvent("subscriber1", 0, "adios");
    ASSERT_EQ(eventCopy, "hola");

    unsigned int listenerId2 = eventEmitter.on(nullptr,
                                               "listener",
                                               "subscriber",
                                               [](const std::string &) { return; });
    ASSERT_EQ(listenerId2, 0);

    eventEmitter.emitEvent("subscriber", 0, "adios");
    ASSERT_EQ(eventCopy, "adios");

    ASSERT_ANY_THROW(eventEmitter.once("once", nullptr));

    ASSERT_NO_THROW(eventEmitter.discardEvent("subscriber"));
    ASSERT_NO_THROW(eventEmitter.discardEvent("subscriber1"));

    auto noStats = eventEmitter.getListenerStats(1);
    ASSERT_EQ(noStats.get(), nullptr);

    eventEmitter.removeListener(nullptr, 1);

    eventEmitter.emitEvent("subscriber", 0, "hola");
    ASSERT_EQ(eventCopy, "hola");

    eventEmitter.removeListener(nullptr, 0);

    eventEmitter.emitEvent("subscriber", 0, "adios");
    ASSERT_EQ(eventCopy, "hola");
}

TEST(UnsafeSingleListenerEventEmitterTest, BasicTestsContainer)
{
    kpsr::UnsafeSingleListenerEventEmitter<std::string> eventEmitter;

    kpsr::CoreContainer container(nullptr, "eventEmitterTests");
    std::string eventCopy;
    unsigned int listenerId = eventEmitter.on(&container,
                                              "listener",
                                              "subscriber",
                                              [&eventCopy](const std::string &event) {
                                                  eventCopy = event;
                                              });
    ASSERT_EQ(listenerId, 0);

    eventEmitter.emitEvent("subscriber", 0, "hola");
    ASSERT_EQ(eventCopy, "hola");

    eventEmitter.emitEvent("subscriber1", 0, "adios");
    ASSERT_EQ(eventCopy, "hola");

    unsigned int listenerId2 = eventEmitter.on(&container,
                                               "listener",
                                               "subscriber",
                                               [](const std::string &) { return; });
    ASSERT_EQ(listenerId2, listenerId);

    eventEmitter.emitEvent("subscriber", 0, "adios");
    ASSERT_EQ(eventCopy, "adios");

    auto noStats = eventEmitter.getListenerStats(1);
    ASSERT_EQ(noStats.get(), nullptr);

    ASSERT_NO_THROW(eventEmitter.discardEvent("subscriber1"));

    auto realStats = eventEmitter.getListenerStats(0);
    ASSERT_NE(realStats.get(), nullptr);
    ASSERT_EQ(realStats->totalDiscardedEvents, 0);

    ASSERT_NO_THROW(eventEmitter.discardEvent("subscriber"));
    ASSERT_EQ(realStats->totalDiscardedEvents, 1);

    eventEmitter.removeListener(&container, 1);

    eventEmitter.emitEvent("subscriber", 10, "hola2");
    ASSERT_EQ(eventCopy, "hola2");

    eventEmitter.emitEvent("subscriber", 0, "hola");
    ASSERT_EQ(eventCopy, "hola");

    eventEmitter.removeListener(&container, 0);

    eventEmitter.emitEvent("subscriber", 0, "adios");
    ASSERT_EQ(eventCopy, "hola");
}

TEST(UnsafeSingleListenerEventEmitter, RemoveListeners)
{
    {
        kpsr::UnsafeSingleListenerEventEmitter<std::string> eventEmitter;

        std::string eventCopy;
        unsigned int listenerId = eventEmitter.on(nullptr,
                                                  "listener",
                                                  "subscriber",
                                                  [&eventCopy](const std::string &event) {
                                                      eventCopy = event;
                                                  });
        ASSERT_EQ(listenerId, 0);

        ASSERT_NO_THROW(eventEmitter.removeAllListeners(nullptr));
    }
    {
        kpsr::UnsafeSingleListenerEventEmitter<std::string> eventEmitter;

        kpsr::CoreContainer container(nullptr, "eventEmitterTests");
        std::string eventCopy;
        unsigned int listenerId = eventEmitter.on(&container,
                                                  "listener",
                                                  "subscriber",
                                                  [&eventCopy](const std::string &event) {
                                                      eventCopy = event;
                                                  });
        ASSERT_EQ(listenerId, 0);
        ASSERT_NO_THROW(eventEmitter.removeAllListeners(nullptr));
        ASSERT_NO_THROW(eventEmitter.removeAllListeners(&container));
    }
}

TEST(UnsafeSingleListenerEventEmitter, RemoveCreateTheSameListener)
{
    kpsr::UnsafeSingleListenerEventEmitter<std::string> eventEmitter;

    std::string eventCopy;
    unsigned int listenerId_0 = eventEmitter.on(nullptr,
                                                "listener",
                                                "subscriber",
                                                [&eventCopy](const std::string &event) {
                                                    eventCopy = event;
                                                });

    eventEmitter.emitEvent("subscriber", 0, "hola");
    ASSERT_EQ(eventCopy, "hola");

    ASSERT_NO_THROW(eventEmitter.removeListener(nullptr, listenerId_0));
    unsigned int listenerId_1 = eventEmitter.on(nullptr,
                                                "listener",
                                                "subscriber",
                                                [&eventCopy](const std::string &event) {
                                                    eventCopy = event;
                                                });
    eventEmitter.emitEvent("subscriber", 0, "hola");
    ASSERT_EQ(eventCopy, "hola");
    ASSERT_NO_THROW(eventEmitter.removeListener(nullptr, listenerId_1));
}
