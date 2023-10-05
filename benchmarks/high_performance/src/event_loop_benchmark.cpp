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

#include <benchmark/benchmark.h>
#include <spdlog/spdlog.h>

#include "test_result_data.h"
#include <klepsydra/high_performance/event_loop_middleware_provider.h>

static const std::vector<std::tuple<std::string, kpsr::EventEmitterType>> scenarios =
    {{"Single Listener", kpsr::EventEmitterType::UNSAFE_SINGLE},
     {"Fixed Listeners", kpsr::EventEmitterType::UNSAFE_MULTI},
     {"Safe Emitter", kpsr::EventEmitterType::SAFE}};

class EventLoopPerformanceTest
{
public:
    EventLoopPerformanceTest(const ::benchmark::State &state)
        : totalLatency(0)
        , _eventEmitterType(std::get<1>(scenarios[state.range(0)]))
        , _poolSize(state.range(1))
        , _numListeners(
              kpsr::EventEmitterType::UNSAFE_SINGLE == _eventEmitterType ? 1 : state.threads())
        , _sleepUS(5)
        , _unsafeBufferPointer(state.range(2))
        , _publishers(_numListeners)
        , totalProcessedItems(_numListeners)
        , _eventLoop(nullptr,
                     "event_loop",
                     kpsr::high_performance::EVENT_LOOP_START_TIMEOUT_MICROSEC,
                     {},
                     _eventEmitterType)
    {
        spdlog::debug("{}. _numListeners: {}, _eventEmitterType: {}, _unsafeBufferPointer: {}",
                      __PRETTY_FUNCTION__,
                      _numListeners,
                      _eventEmitterType,
                      _unsafeBufferPointer);

        publishedItems.resize(_numListeners, 0);
        std::fill(totalProcessedItems.begin(), totalProcessedItems.end(), 0);

        for (int i = 0; i < _numListeners; i++) {
            std::string name = "channel_" + std::to_string(i);

            kpsr::Publisher<unsigned long long> *publisher =
                _eventLoop.getPublisher<unsigned long long>(name,
                                                            _poolSize,
                                                            nullptr,
                                                            nullptr,
                                                            _unsafeBufferPointer);
            _publishers[i] = publisher;

            std::string listenerName = "listener_" + std::to_string(i);
            spdlog::debug("Registering listener {} on subscriber {}", listenerName, name);
            _eventLoop.getSubscriber<unsigned long long>(name, _eventEmitterType)
                ->registerListener(listenerName,
                                   [this, i, name, listenerName](const unsigned long long &event) {
                                       unsigned long long now =
                                           kpsr::TimeUtils::getCurrentNanosecondsAsLlu();
                                       totalLatency += now - event;
                                       totalProcessedItems[i]++;
                                       spdlog::debug("{}. i: {}, name: {}, listenerName: {} , "
                                                     "Latency: {} ns, totalProcessedItems[{}]: {}, "
                                                     "this: {:x}",
                                                     __PRETTY_FUNCTION__,
                                                     i,
                                                     name,
                                                     listenerName,
                                                     (now - event),
                                                     i,
                                                     totalProcessedItems[i],
                                                     (unsigned long) this);
                                   });
        }
        _eventLoop.start();
    }

    ~EventLoopPerformanceTest()
    {
        spdlog::debug("{}", __PRETTY_FUNCTION__);
        for (auto i = 0; i < _numListeners; i++) {
            std::string name = "channel_" + std::to_string(i);
            std::string listenerName = "listener_" + std::to_string(i);
            _eventLoop.getSubscriber<unsigned long long>(name, _eventEmitterType)
                ->removeListener(listenerName);
        }
        _eventLoop.stop();
    }

    unsigned long long totalLatency;

    kpsr::EventEmitterType _eventEmitterType;
    int _poolSize;
    int _numListeners;
    int _sleepUS;
    bool _unsafeBufferPointer;
    std::vector<int> publishedItems;

    std::vector<kpsr::Publisher<unsigned long long> *> _publishers;

    std::vector<std::atomic_int> totalProcessedItems;
    kpsr::high_performance::EventLoopMiddlewareProvider<2048> _eventLoop;
};

std::unique_ptr<EventLoopPerformanceTest> sut;

void DoSetup(const benchmark::State &state)
{
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%c] [%H:%M:%S %f] [%n] [%l] [%t] %v");

    spdlog::debug("{}", __PRETTY_FUNCTION__);
    if (sut) {
        // NOOP
    } else {
        sut.reset(new EventLoopPerformanceTest(state));
    }
}

void DoTeardown(const benchmark::State &state)
{
    spdlog::debug("{}", __PRETTY_FUNCTION__);
    if (sut) {
        sut.reset();
    }
}

void EventLoopBenchmarkParametersMulti(benchmark::internal::Benchmark *benchmark)
{
    benchmark->ArgNames({"EmitterType", "PoolSize", "Unsafe"});

    for (int unsafePtrCheck = 0; unsafePtrCheck <= 1; unsafePtrCheck++) {
        for (int poolSize = 0; poolSize <= 100; poolSize += 100) {
            for (int emitterType = 1; emitterType <= 2; emitterType += 1) {
                benchmark->Args({emitterType, poolSize, unsafePtrCheck});
            }
        }
    }
}

void EventLoopBenchmarkParametersSingle(benchmark::internal::Benchmark *benchmark)
{
    benchmark->ArgNames({"EmitterType", "PoolSize", "Unsafe"});

    for (int unsafePtrCheck = 0; unsafePtrCheck <= 1; unsafePtrCheck++) {
        for (int poolSize = 0; poolSize <= 100; poolSize += 100) {
            benchmark->Args({0, poolSize, unsafePtrCheck});
        }
    }
}

static void EventLoopPerformanceTest_PublishTest(benchmark::State &state)
{
    int index = kpsr::EventEmitterType::UNSAFE_SINGLE == sut->_eventEmitterType
                    ? 0
                    : state.thread_index();

    kpsr::Publisher<unsigned long long> *publisher = sut->_publishers[index];

    long long unsigned int discardedMessages(0);
    for (auto _ : state) {
        sut->publishedItems[index]++;
        unsigned long long event = kpsr::TimeUtils::getCurrentNanosecondsAsLlu();
        spdlog::debug("publishing to publisher index: {}", index);
        publisher->publish(event);
        for (int i = 0; i < 10; i++) {
            if (sut->totalProcessedItems[index] < (sut->publishedItems[index] - 50)) {
                auto eventLoopPublisher = dynamic_cast<
                    kpsr::high_performance::EventLoopPublisher<unsigned long long, 2048> *>(
                    publisher);
                spdlog::debug(
                    "waiting: sut->totalProcessedItems[{}] = {}. "
                    "sut->publishedItems[{}] = {}, eventLoopPublisher->_discardedMessages: {},"
                    "sut: {:x}",
                    index,
                    sut->totalProcessedItems[index],
                    index,
                    sut->publishedItems[index],
                    eventLoopPublisher->_discardedMessages,
                    (unsigned long) sut.get());
                std::this_thread::sleep_for(std::chrono::microseconds(sut->_sleepUS));
            } else {
                spdlog::debug("published to publisher index: {}", index);
                break;
            }
        }
        if (sut->totalProcessedItems[index] < sut->publishedItems[index]) {
            spdlog::debug("{}. eventloop subscribers did not finish.", __PRETTY_FUNCTION__);
        }
    }
    auto eventLoopPublisher =
        dynamic_cast<kpsr::high_performance::EventLoopPublisher<unsigned long long, 2048> *>(
            publisher);
    discardedMessages += eventLoopPublisher->_discardedMessages;

    int totalProcessed = 0;
    for (int totalProcessedItem : sut->totalProcessedItems) {
        totalProcessed += totalProcessedItem;
    }
    spdlog::debug("finished the publish loop");
    spdlog::debug("totalProcessed : {}", totalProcessed);
    spdlog::debug("discarded : {}", discardedMessages);

    int mulFactor = (kpsr::EventEmitterType::UNSAFE_SINGLE == sut->_eventEmitterType) ? 10 : 1;
    int rightSize = static_cast<unsigned long long>(
        std::accumulate(sut->publishedItems.begin(), sut->publishedItems.end(), 0));

    for (int i = 0; i < 10; i++) {
        int totalProcessed = 0;
        for (int totalProcessedItem : sut->totalProcessedItems) {
            totalProcessed += totalProcessedItem;
        }
        int leftSide = (totalProcessed * mulFactor) + discardedMessages;

        if (leftSide < rightSize) {
            spdlog::debug("waiting on the publish loop: leftSide = {}. rightSize = {}",
                          leftSide,
                          rightSize);
            std::this_thread::sleep_for(std::chrono::microseconds(sut->_sleepUS));
        } else {
            spdlog::debug("{}. Benchmark completed.", __PRETTY_FUNCTION__);
            break;
        }
    }
    for (int totalProcessedItem : sut->totalProcessedItems) {
        totalProcessed += totalProcessedItem;
    }
    int leftSide = (totalProcessed * mulFactor) + discardedMessages;
    if (leftSide < rightSize) {
        spdlog::error("{}. Benchmark failed.", __PRETTY_FUNCTION__);
    }

    if (index == 0) {
        state.counters["Latency"] = benchmark::Counter(sut->totalLatency / 1000.0);
        state.counters["Processed"] = totalProcessed;
        state.counters["AverageLatency"] = benchmark::Counter(sut->totalLatency / 1000.0 /
                                                              totalProcessed);
    }
    std::string name = "channel_" + std::to_string(index);
    std::string listenerName = "listener_" + std::to_string(index);
    sut->_eventLoop.getSubscriber<unsigned long long>(name, sut->_eventEmitterType)
        ->removeListener(listenerName);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

BENCHMARK(EventLoopPerformanceTest_PublishTest)
    ->Apply(EventLoopBenchmarkParametersSingle)
    ->UseRealTime()
    ->Setup(DoSetup)
    ->Teardown(DoTeardown);

BENCHMARK(EventLoopPerformanceTest_PublishTest)
    ->Apply(EventLoopBenchmarkParametersMulti)
    ->ThreadRange(1, 8)
    ->UseRealTime()
    ->Setup(DoSetup)
    ->Teardown(DoTeardown);
