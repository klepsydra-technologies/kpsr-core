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

#include <random>

#include <spdlog/spdlog.h>

#include <klepsydra/high_performance/event_loop_middleware_provider.h>

#include "test_result_data.h"

class PerformanceTest
{
public:
    PerformanceTest(kpsr::EventEmitterType eventEmitterType,
                    size_t dataSize,
                    int poolSize,
                    int numListeners)
        : _eventEmitterType(eventEmitterType)
        , _dataSize(dataSize)
        , _poolSize(poolSize)
        , _numListeners(numListeners)
        , _totalProcessed(0)
        , _eventLoop(nullptr,
                     "event_loop",
                     kpsr::high_performance::EVENT_LOOP_START_TIMEOUT_MICROSEC,
                     {},
                     _eventEmitterType)
        , _data(_dataSize)
    {
        for (int i = 0; i < _numListeners; i++) {
            std::string name = "channel_" + std::to_string(i);
            std::string listenerName = "listener_" + std::to_string(i);
            _eventLoop.getSubscriber<std::string>(name, _eventEmitterType)
                ->registerListener(listenerName, [this, listenerName](const std::string &event) {
                    _totalProcessed++;
                    //                spdlog::info("listener call. listenerName: {}. _totalProcessed: {}. event: {}", listenerName, _totalProcessed, event);
                });
        }

        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
        std::uniform_real_distribution<> distribution(1.0, 60.0);
        for (size_t j = 0; j < _dataSize; j++) {
            _data[j] = distribution(gen);
        }

        _eventLoop.start();
    }

    ~PerformanceTest()
    {
        for (int i = 0; i < _numListeners; i++) {
            std::string name = "channel_" + std::to_string(i);
            std::string listenerName = "listener_" + std::to_string(i);
            _eventLoop.getSubscriber<std::string>(name, _eventEmitterType)
                ->removeListener(listenerName);
        }
        _eventLoop.stop();
    }

    long test()
    {
        long totalExecutionTime = 0;
        _totalProcessed = 0;
        long before = kpsr::TimeUtils::getCurrentMilliseconds();

        spdlog::debug("starting benchmark...");

        std::vector<std::thread> threads(0);
        for (int index = 0; index < _numListeners; index++) {
            threads.push_back(std::thread([this, index]() {
                std::string name = "channel_" + std::to_string(index);
                kpsr::Publisher<std::string> *publisher =
                    _eventLoop.getPublisher<std::string>(name, _poolSize, nullptr, nullptr);
                for (int i = 0; i < 100; i++) {
                    std::string strData = "publisher_" + std::to_string(index) + "_" +
                                          std::to_string(i);
                    publisher->publish(strData);
                }
            }));
        }
        for (int i = 0; i < _numListeners; i++) {
            threads[i].join();
        }

        while (_totalProcessed < (_numListeners * 100)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        spdlog::debug("finishing benchmark.");

        long after = kpsr::TimeUtils::getCurrentMilliseconds();
        totalExecutionTime = after - before;
        return totalExecutionTime;
    }

private:
    kpsr::EventEmitterType _eventEmitterType;
    size_t _dataSize;
    int _poolSize;
    int _numListeners;
    std::atomic_int _totalProcessed;
    kpsr::high_performance::EventLoopMiddlewareProvider<4096> _eventLoop;
    std::vector<float> _data;
};

int main(int argc, char **argv)
{
    std::vector<std::tuple<std::string, kpsr::EventEmitterType>> scenarios(2);
    scenarios[0] = {"Fixed Listeners", kpsr::EventEmitterType::UNSAFE_MULTI};
    scenarios[1] = {"Full Event Emitter", kpsr::EventEmitterType::SAFE};

    std::vector<size_t> dataSizes(3);
    dataSizes[0] = 10;
    dataSizes[1] = 100;
    dataSizes[2] = 1000;

    for (auto scenario : scenarios) {
        for (int poolSize = 0; poolSize <= 200; poolSize += 200) {
            for (int numListeners = 10; numListeners <= 20; numListeners += 10) {
                for (auto dataSize : dataSizes) {
                    std::vector<long> totalExecutionTimes;

                    spdlog::debug(
                        "Starting. {}\tData size: {}\tPool size: {}\tNumber of listeners: {}.",
                        std::get<0>(scenario),
                        dataSize,
                        poolSize,
                        numListeners);

                    PerformanceTest performanceTest(std::get<1>(scenario),
                                                    dataSize,
                                                    poolSize,
                                                    numListeners);

                    for (int i = 0; i < 4; i++) {
                        totalExecutionTimes.push_back(performanceTest.test());
                    }

                    TestResults totalExecutionTimesResult(totalExecutionTimes);

                    spdlog::info(
                        "{}\tData size: {:5}\tPool size: {:5}\tNumber of listeners: {:5}\ttotal "
                        "execution time: {:5}\tAvg: {:.4f}\tStdev: {:.4f}",
                        std::get<0>(scenario),
                        dataSize,
                        poolSize,
                        numListeners,
                        totalExecutionTimesResult.sum,
                        totalExecutionTimesResult.average,
                        totalExecutionTimesResult.stddev);
                }
            }
        }
        spdlog::info("***");
    }
}
