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

#include "gtest/gtest.h"

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

    kpsr::Container container(nullptr, "eventEmitterTests");
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

        kpsr::Container container(nullptr, "eventEmitterTests");
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
