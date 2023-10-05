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

#include <benchmark/benchmark.h>
#include <chrono>
#include <functional>
#include <klepsydra/high_performance/data_multiplexer_middleware_provider.h>
#include <klepsydra/high_performance/event_loop_middleware_provider.h>

static void EventLoopThreePubThreeSubBenchmark(benchmark::State &state)
{
    kpsr::high_performance::EventLoopMiddlewareProvider<32> provider(nullptr);

    provider.start();
    auto publisher1 = provider.getPublisher<long int>("eventLoopTopic1", 0, nullptr, nullptr);
    auto publisher2 = provider.getPublisher<long int>("eventLoopTopic2", 0, nullptr, nullptr);
    auto publisher3 = provider.getPublisher<long int>("eventLoopTopic3", 0, nullptr, nullptr);

    size_t iterations1 = 0;
    size_t iterations2 = 0;
    size_t iterations3 = 0;

    long int totalLatency1 = 0;
    long int totalLatency2 = 0;
    long int totalLatency3 = 0;

    size_t publishCounter = 0;

    provider.getSubscriber<long int>("eventLoopTopic1")
        ->registerListener("iterationsCounter1",
                           [&iterations1, &totalLatency1](const long int &event) {
                               iterations1++;
                               totalLatency1 +=
                                   std::chrono::duration_cast<std::chrono::microseconds>(
                                       std::chrono::system_clock::now().time_since_epoch())
                                       .count() -
                                   event;
                           });

    provider.getSubscriber<long int>("eventLoopTopic2")
        ->registerListener("iterationsCounter2",
                           [&iterations2, &totalLatency2](const long int &event) {
                               iterations2++;
                               totalLatency2 +=
                                   std::chrono::duration_cast<std::chrono::microseconds>(
                                       std::chrono::system_clock::now().time_since_epoch())
                                       .count() -
                                   event;
                           });

    provider.getSubscriber<long int>("eventLoopTopic3")
        ->registerListener("iterationsCounter3",
                           [&iterations3, &totalLatency3](const long int &event) {
                               iterations3++;
                               totalLatency3 +=
                                   std::chrono::duration_cast<std::chrono::microseconds>(
                                       std::chrono::system_clock::now().time_since_epoch())
                                       .count() -
                                   event;
                           });

    long int message = 0;

    for (auto _ : state) {
        auto start = std::chrono::high_resolution_clock::now();

        message = std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count();
        publisher1->publish(message);
        message = std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count();
        publisher2->publish(message);
        message = std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count();
        publisher3->publish(message);

        publishCounter++;

        while ((iterations1 < publishCounter) || (iterations2 < publishCounter) ||
               (iterations3 < publishCounter)) {
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }

        auto end = std::chrono::high_resolution_clock::now();

        auto elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end -
                                                                                         start);

        state.SetIterationTime(elapsed_seconds.count());
    }

    provider.getSubscriber<long int>("eventLoopTopic1")->removeListener("iterationsCounter1");
    provider.getSubscriber<long int>("eventLoopTopic2")->removeListener("iterationsCounter2");
    provider.getSubscriber<long int>("eventLoopTopic3")->removeListener("iterationsCounter3");

    provider.stop();

    state.counters["AvgLatency1"] = benchmark::Counter(totalLatency1 / iterations1);
    state.counters["AvgLatency2"] = benchmark::Counter(totalLatency2 / iterations2);
    state.counters["AvgLatency3"] = benchmark::Counter(totalLatency2 / iterations3);
}

static void DataMultiplexerOnePubThreeSubBenchmark(benchmark::State &state)
{
    kpsr::high_performance::DataMultiplexerMiddlewareProvider<long int, 32>
        provider(nullptr, "dataMultiplexer");

    auto publisher = provider.getPublisher();

    size_t iterations1 = 0;
    size_t iterations2 = 0;
    size_t iterations3 = 0;

    long int totalLatency1 = 0;
    long int totalLatency2 = 0;
    long int totalLatency3 = 0;

    size_t publishCounter = 0;

    provider.getSubscriber("dataMultiplexerTopic1")
        ->registerListener("iterationsCounter1",
                           [&iterations1, &totalLatency1](const long int &event) {
                               iterations1++;
                               totalLatency1 +=
                                   std::chrono::duration_cast<std::chrono::microseconds>(
                                       std::chrono::system_clock::now().time_since_epoch())
                                       .count() -
                                   event;
                           });

    provider.getSubscriber("dataMultiplexerTopic2")
        ->registerListener("iterationsCounter2",
                           [&iterations2, &totalLatency2](const long int &event) {
                               iterations2++;
                               totalLatency2 +=
                                   std::chrono::duration_cast<std::chrono::microseconds>(
                                       std::chrono::system_clock::now().time_since_epoch())
                                       .count() -
                                   event;
                           });

    provider.getSubscriber("dataMultiplexerTopic3")
        ->registerListener("iterationsCounter3",
                           [&iterations3, &totalLatency3](const long int &event) {
                               iterations3++;
                               totalLatency3 +=
                                   std::chrono::duration_cast<std::chrono::microseconds>(
                                       std::chrono::system_clock::now().time_since_epoch())
                                       .count() -
                                   event;
                           });

    long int message = 0;

    for (auto _ : state) {
        auto start = std::chrono::high_resolution_clock::now();

        message = std::chrono::duration_cast<std::chrono::microseconds>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count();
        publisher->publish(message);

        publishCounter++;

        while ((iterations1 < publishCounter) || (iterations2 < publishCounter) ||
               (iterations3 < publishCounter)) {
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }

        auto end = std::chrono::high_resolution_clock::now();

        auto elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end -
                                                                                         start);

        state.SetIterationTime(elapsed_seconds.count());
    }
    provider.getSubscriber("dataMultiplexerTopic1")->removeListener("iterationsCounter1");
    provider.getSubscriber("dataMultiplexerTopic2")->removeListener("iterationsCounter2");
    provider.getSubscriber("dataMultiplexerTopic3")->removeListener("iterationsCounter3");

    state.counters["AvgLatency1"] = benchmark::Counter(totalLatency1 / iterations1);
    state.counters["AvgLatency2"] = benchmark::Counter(totalLatency2 / iterations2);
    state.counters["AvgLatency3"] = benchmark::Counter(totalLatency2 / iterations3);
}

BENCHMARK(EventLoopThreePubThreeSubBenchmark)->UseRealTime();
BENCHMARK(DataMultiplexerOnePubThreeSubBenchmark)->UseRealTime();