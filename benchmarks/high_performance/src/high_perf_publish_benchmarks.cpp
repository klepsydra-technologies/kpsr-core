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

#include <algorithm>
#include <random>
#include <vector>

#include <benchmark/benchmark.h>

#include <klepsydra/high_performance/data_multiplexer_middleware_provider.h>
#include <klepsydra/mem_core/basic_middleware_provider.h>

static void DataMultiplexerPublish(benchmark::State &state)
{
    kpsr::high_performance::DataMultiplexerMiddlewareProvider<std::vector<float>, 4>
        provider(nullptr, "test", nullptr, nullptr);

    auto publisher = provider.getPublisher();
    auto subscriber = provider.getSubscriber("testSubscriber");

    std::random_device random_device;
    auto rng = std::mt19937(random_device());
    auto f32rng = std::bind(std::uniform_real_distribution<float>(-1.0f, +1.0f), std::ref(rng));

    size_t eventSize = 100;
    std::vector<float> testEvent(eventSize);
    float mean = 0;
    size_t iterations = 0;
    auto meanCalculator = [&mean, &iterations](const std::vector<float> &event) {
        mean = std::accumulate(event.begin(), event.end(), 0.0f);
        iterations++;
    };
    subscriber->registerListener("meanCalculator", meanCalculator);

    auto &publisherStats = publisher->publicationStats;
    auto listenerStats = subscriber->getSubscriptionStats("meanCalculator");
    for (auto _ : state) {
        std::generate(testEvent.begin(), testEvent.end(), std::ref(f32rng));
        auto start = std::chrono::high_resolution_clock::now();
        publisher->publish(testEvent);
        auto end = std::chrono::high_resolution_clock::now();
        std::this_thread::sleep_for(std::chrono::microseconds(10));

        auto elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end -
                                                                                         start);

        state.SetIterationTime(elapsed_seconds.count());
    }
    state.counters["publishedEvents"] = publisherStats.totalProcessed -
                                        publisherStats.totalDiscardedEvents;
    state.counters["listenerIterations"] = iterations;
    state.counters["listenerEvents"] = listenerStats->totalProcessed;
}

BENCHMARK(DataMultiplexerPublish)->UseRealTime();

static void BasicQueuePublish(benchmark::State &state)
{
    kpsr::mem::BasicMiddlewareProvider<std::vector<float>>
        provider(nullptr, "test", 10, 0, nullptr, nullptr, false);

    auto publisher = provider.getPublisher();
    auto subscriber = provider.getSubscriber();

    std::random_device random_device;
    auto rng = std::mt19937(random_device());
    auto f32rng = std::bind(std::uniform_real_distribution<float>(-1.0f, +1.0f), std::ref(rng));

    size_t eventSize = 100;
    std::vector<float> testEvent(eventSize);
    float mean = 0;
    size_t iterations = 0;
    auto meanCalculator = [&mean, &iterations](const std::vector<float> &event) {
        mean = std::accumulate(event.begin(), event.end(), 0.0f);
        iterations++;
    };
    subscriber->registerListener("meanCalculator", meanCalculator);

    auto &publisherStats = publisher->publicationStats;
    auto listenerStats = subscriber->getSubscriptionStats("meanCalculator");
    provider.start();
    for (auto _ : state) {
        std::generate(testEvent.begin(), testEvent.end(), std::ref(f32rng));
        auto start = std::chrono::high_resolution_clock::now();
        publisher->publish(testEvent);
        auto end = std::chrono::high_resolution_clock::now();
        std::this_thread::sleep_for(std::chrono::microseconds(10));
        auto elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(end -
                                                                                         start);

        state.SetIterationTime(elapsed_seconds.count());
    }
    provider.stop();
    state.counters["publishedEvents"] = publisherStats.totalProcessed -
                                        publisherStats.totalDiscardedEvents;
    state.counters["listenerIterations"] = iterations;
    state.counters["listenerEvents"] = listenerStats->totalProcessed;
}

BENCHMARK(BasicQueuePublish)->UseRealTime();
