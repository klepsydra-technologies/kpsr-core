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
#include <atomic>

#include <sstream>
#include <fstream>

#include "gtest/gtest.h"

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <klepsydra/core/smart_object_pool.h>
#include <klepsydra/core/time_utils.h>

class PoolTestObject {
public:

    static std::atomic_int constructorInvokations;
    static std::atomic_int emptyConstructorInvokations;
    static std::atomic_int copyInvokations;

    PoolTestObject(int id, const std::string & message)
        : _id(id)
        , _message(message) {
        PoolTestObject::constructorInvokations++;
    }

    PoolTestObject() {
        spdlog::info("new empty invocation!!!");
        PoolTestObject::emptyConstructorInvokations++;
    }

    PoolTestObject(const PoolTestObject & that)
        : _id(that._id)
        , _message(that._message) {
        PoolTestObject::copyInvokations++;
    }

    int _id;
    std::string _message;
};

std::atomic_int PoolTestObject::constructorInvokations(0);
std::atomic_int PoolTestObject::emptyConstructorInvokations(0);
std::atomic_int PoolTestObject::copyInvokations(0);

TEST(SmartObjectPoolTest, nominalCase) {
    kpsr::SmartObjectPool<PoolTestObject> objectPool(4);
    ASSERT_EQ(PoolTestObject::emptyConstructorInvokations, 4);

    auto object1 = objectPool.acquire();
    auto object2 = objectPool.acquire();
    auto object3 = objectPool.acquire();
    auto object4 = objectPool.acquire();
    ASSERT_THROW(objectPool.acquire(), std::out_of_range);
    ASSERT_THROW(objectPool.acquire(), std::out_of_range);
    ASSERT_THROW(objectPool.acquire(), std::out_of_range);

    ASSERT_EQ(objectPool.objectPoolFails, 3);
}

TEST(SmartObjectPoolTest, nominalCaseWithFailures) {

    PoolTestObject::constructorInvokations = 0;
    PoolTestObject::emptyConstructorInvokations = 0;
    PoolTestObject::copyInvokations = 0;

    kpsr::SmartObjectPool<PoolTestObject> objectPool(4);
    ASSERT_EQ(PoolTestObject::emptyConstructorInvokations, 4);

    auto object1 = objectPool.acquire();
    auto object2 = objectPool.acquire();
    auto object3 = objectPool.acquire();
    auto object4 = objectPool.acquire();
    ASSERT_THROW(objectPool.acquire(), std::out_of_range);
    ASSERT_THROW(objectPool.acquire(), std::out_of_range);
    ASSERT_THROW(objectPool.acquire(), std::out_of_range);

    ASSERT_EQ(objectPool.objectPoolFails, 3);
}

TEST(SmartObjectPoolTest, initializerFunction) {

    PoolTestObject::constructorInvokations = 0;
    PoolTestObject::emptyConstructorInvokations = 0;
    PoolTestObject::copyInvokations = 0;

    std::function<void(PoolTestObject &)> initializer = [] (PoolTestObject & object){
        spdlog::info("in initializer");
        object._id = 0;
        object._message = "hola";
    };


    kpsr::SmartObjectPool<PoolTestObject> objectPool(4, initializer);
    ASSERT_EQ(PoolTestObject::emptyConstructorInvokations, 4);

    auto object1 = objectPool.acquire();
    ASSERT_EQ(object1->_id, 0);
    ASSERT_EQ(object1->_message, "hola");

    auto object2 = objectPool.acquire();
    ASSERT_EQ(object1->_id, 0);
    ASSERT_EQ(object1->_message, "hola");

    auto object3 = objectPool.acquire();
    ASSERT_EQ(object1->_id, 0);
    ASSERT_EQ(object1->_message, "hola");

    auto object4 = objectPool.acquire();
    ASSERT_EQ(object1->_id, 0);
    ASSERT_EQ(object1->_message, "hola");

    ASSERT_THROW(objectPool.acquire(), std::out_of_range);
    ASSERT_THROW(objectPool.acquire(), std::out_of_range);
    ASSERT_THROW(objectPool.acquire(), std::out_of_range);

    ASSERT_EQ(objectPool.objectPoolFails, 3);
}

TEST(SmartObjectPoolTest, performanceTest) {
    class PoolTestThread {
    public:
        PoolTestThread(kpsr::SmartObjectPool<PoolTestObject> & objectPool)
            : noAllocations(0)
            , timeAcquiring(0)
            , totalTime(0)
            , _localObjects(20)
            , _objectPool(objectPool)
        {}

        void start() {
            _thread = std::thread([&]{
                this->run();
            });
        }

        void run() {
            for (int i = 0; i < 5000; i++) {
                long long unsigned int startMs = kpsr::TimeUtils::getCurrentMillisecondsAsLlu();
                {
                    _localObjects[i % 20] = _objectPool.acquire();
                    noAllocations++;
                    long long unsigned int stopMS = kpsr::TimeUtils::getCurrentMillisecondsAsLlu();
                    timeAcquiring += (stopMS - startMs);
                }
                long long unsigned int stopMS = kpsr::TimeUtils::getCurrentMillisecondsAsLlu();
                totalTime += (stopMS - startMs);
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }

        void stop() {
            _thread.join();
        }

        unsigned long long int noAllocations, timeAcquiring, totalTime;

    private:
        std::vector<std::shared_ptr<PoolTestObject>> _localObjects;
        kpsr::SmartObjectPool<PoolTestObject> & _objectPool;
        std::thread _thread;
    };

    std::vector<std::shared_ptr<PoolTestThread>> threadPool(200);
    kpsr::SmartObjectPool<PoolTestObject> pool(20000);

    for (int i = 0; i < 200; i++) {
        threadPool[i] = std::shared_ptr<PoolTestThread>(new PoolTestThread(pool));
        threadPool[i]->start();
    }
    for (int i = 0; i < 200; i++) {
        threadPool[i]->stop();
    }

    for (int i = 0; i < 200; i++) {
        spdlog::info("Thread[{}]. noAllocations: {}"
                  ". timeAcquiring: {}"
                  ". totalTime: {}", i, threadPool[i]->noAllocations,
                  threadPool[i]->timeAcquiring,
                  threadPool[i]->totalTime
        );
    }
}
