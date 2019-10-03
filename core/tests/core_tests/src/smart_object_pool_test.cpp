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
#include <atomic>

#include <sstream>
#include <fstream>

#include "gtest/gtest.h"

#include <klepsydra/core/smart_object_pool.h>
#include <klepsydra/core/time_utils.h>

class PoolTestObject {
public:

    static std::atomic_int constructorInvokations;
    static std::atomic_int emptyConstructorInvokations;
    static std::atomic_int copyInvokations;

    PoolTestObject(int id, std::string message)
        : _id(id)
        , _message(message) {
        PoolTestObject::constructorInvokations++;
    }

    PoolTestObject() {
        std::cout << "new empty invocation!!!" << std::endl;
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
        std::cout << "in initializer" << std::endl;
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
        std::cout << "Thread[" << i << "]. noAllocations: " << threadPool[i]->noAllocations
                  << ". timeAcquiring: " << threadPool[i]->timeAcquiring
                  << ". totalTime: " << threadPool[i]->totalTime << std::endl;
    }
}
