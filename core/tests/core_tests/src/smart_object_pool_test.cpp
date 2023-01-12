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

#include <atomic>
#include <stdio.h>
#include <thread>
#include <unistd.h>

#include <fstream>
#include <sstream>

#include "gtest/gtest.h"

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <klepsydra/core/smart_object_pool.h>
#include <klepsydra/core/time_utils.h>

class PoolTestObject
{
public:
    static std::atomic_int constructorInvokations;
    static std::atomic_int emptyConstructorInvokations;
    static std::atomic_int copyInvokations;

    PoolTestObject(int id, const std::string &message)
        : _id(id)
        , _message(message)
    {
        PoolTestObject::constructorInvokations++;
    }

    PoolTestObject()
    {
        spdlog::debug("new empty invocation!!!");
        PoolTestObject::emptyConstructorInvokations++;
    }

    PoolTestObject(const PoolTestObject &that)
        : _id(that._id)
        , _message(that._message)
    {
        PoolTestObject::copyInvokations++;
    }

    int _id;
    std::string _message;
};

std::atomic_int PoolTestObject::constructorInvokations(0);
std::atomic_int PoolTestObject::emptyConstructorInvokations(0);
std::atomic_int PoolTestObject::copyInvokations(0);

TEST(SmartObjectPoolTest, nominalCase)
{
    kpsr::SmartObjectPool<PoolTestObject> objectPool("SmartObjectPoolTest", 4);
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

TEST(SmartObjectPoolTest, nominalCaseWithFailures)
{
    PoolTestObject::constructorInvokations = 0;
    PoolTestObject::emptyConstructorInvokations = 0;
    PoolTestObject::copyInvokations = 0;

    kpsr::SmartObjectPool<PoolTestObject> objectPool("SmartObjectPoolTest", 4);
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

TEST(SmartObjectPoolTest, initializerFunction)
{
    PoolTestObject::constructorInvokations = 0;
    PoolTestObject::emptyConstructorInvokations = 0;
    PoolTestObject::copyInvokations = 0;

    std::function<void(PoolTestObject &)> initializer = [](PoolTestObject &object) {
        spdlog::info("in initializer");
        object._id = 0;
        object._message = "hola";
    };

    kpsr::SmartObjectPool<PoolTestObject> objectPool("SmartObjectPoolTest", 4, initializer);
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

TEST(SmartObjectPoolTest, performanceTest)
{
    class PoolTestThread
    {
    public:
        PoolTestThread(kpsr::SmartObjectPool<PoolTestObject> &objectPool)
            : noAllocations(0)
            , timeAcquiring(0)
            , totalTime(0)
            , _localObjects(20)
            , _objectPool(objectPool)
        {}

        void start()
        {
            _thread = std::thread([&] { this->run(); });
        }

        void run()
        {
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

        void stop() { _thread.join(); }

        unsigned long long int noAllocations, timeAcquiring, totalTime;

    private:
        std::vector<std::shared_ptr<PoolTestObject>> _localObjects;
        kpsr::SmartObjectPool<PoolTestObject> &_objectPool;
        std::thread _thread;
    };

    std::vector<std::shared_ptr<PoolTestThread>> threadPool(200);
    kpsr::SmartObjectPool<PoolTestObject> pool("SmartObjectPoolTest", 20000);

    for (int i = 0; i < 200; i++) {
        threadPool[i] = std::shared_ptr<PoolTestThread>(new PoolTestThread(pool));
        threadPool[i]->start();
    }
    for (int i = 0; i < 200; i++) {
        threadPool[i]->stop();
    }

    for (int i = 0; i < 200; i++) {
        spdlog::debug("Thread[{}]. noAllocations: {}"
                      ". timeAcquiring: {}"
                      ". totalTime: {}",
                      i,
                      threadPool[i]->noAllocations,
                      threadPool[i]->timeAcquiring,
                      threadPool[i]->totalTime);
    }
}
