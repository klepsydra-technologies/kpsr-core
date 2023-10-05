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
#include <iomanip>
#include <spdlog/spdlog.h>
#include <sstream>

#include <klepsydra/core/core_container.h>
#include <klepsydra/core/event_emitter_middleware_provider.h>

#include "test_result_data.h"

static const std::vector<std::tuple<std::string, kpsr::EventEmitterType>> scenarios =
    {{"Single Listener", kpsr::EventEmitterType::UNSAFE_SINGLE},
     {"Fixed Listeners", kpsr::EventEmitterType::UNSAFE_MULTI},
     {"Safe Emitter", kpsr::EventEmitterType::SAFE}};

class EventEmitterPerformanceTest : public benchmark::Fixture
{
public:
    void SetUp(const ::benchmark::State &state)
    {
        auto scenarioId = state.range(0);
        _eventEmitterType = std::get<1>(scenarios[scenarioId]);
        _poolSize = state.range(1);
        _dataSize = state.range(2);
        _numListeners = state.range(3);
        bool useContainer = state.range(4);
        if (useContainer) {
            container = std::unique_ptr<kpsr::CoreContainer>(
                new kpsr::CoreContainer(nullptr, "testContainer"));
        }
        eventEmitter.reset(new kpsr::EventEmitterMiddlewareProvider<std::vector<float>>(
            container.get(), "event", _poolSize, nullptr, nullptr, _eventEmitterType));

        for (auto i = 0; i < _numListeners; i++) {
            eventEmitter->getSubscriber()->registerListener(prefix + std::to_string(i),
                                                            [](const std::vector<float> &) {});
        }

        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
        std::uniform_real_distribution<> distribution(1.0, 60.0);
        _data.resize(_dataSize);
        for (size_t j = 0; j < _dataSize; j++) {
            _data[j] = distribution(gen);
        }
    }

    void TearDown(const ::benchmark::State &state)
    {
        for (auto i = 0; i < _numListeners; i++) {
            eventEmitter->getSubscriber()->removeListener(prefix + std::to_string(i));
        }
        eventEmitter.reset(nullptr);
    }

    kpsr::EventEmitterType _eventEmitterType;
    size_t _dataSize;
    int _poolSize;
    int _numListeners;
    std::unique_ptr<kpsr::Container> container;
    std::unique_ptr<kpsr::EventEmitterMiddlewareProvider<std::vector<float>>> eventEmitter;
    std::vector<float> _data;
    std::string prefix = "listener_";
};

BENCHMARK_DEFINE_F(EventEmitterPerformanceTest, PublishTest)(benchmark::State &state)
{
    for (auto _ : state) {
        eventEmitter->getPublisher()->publish(_data);
    }
    auto eventEmitterTypeName = std::get<0>(scenarios[state.range(0)]);
    std::stringstream label;
    label << std::left << std::setw(20) << eventEmitterTypeName << "pool size: " << std::right
          << std::setw(2) << std::to_string(_poolSize) << " dataSize: " << std::setw(20)
          << std::to_string(_dataSize) + " num listeners: " << std::setw(3)
          << std::to_string(_numListeners);
    state.SetLabel(label.str());
}

BENCHMARK_REGISTER_F(EventEmitterPerformanceTest, PublishTest)
    ->ArgNames({"EmitterType", "PoolSize", "DataSize", "NumListeners", "UseContainer"})
    ->ArgsProduct({benchmark::CreateDenseRange(0, 2, /*step=*/1),
                   benchmark::CreateDenseRange(0, 1, /*step=*/1),
                   benchmark::CreateRange(10, 1000, /*multi=*/10),
                   benchmark::CreateRange(1, 1, /*multi=*/1),
                   benchmark::CreateDenseRange(0, 1, /*step=*/1)});

BENCHMARK_REGISTER_F(EventEmitterPerformanceTest, PublishTest)
    ->ArgNames({"EmitterType", "PoolSize", "DataSize", "NumListeners", "UseContainer"})
    ->ArgsProduct({benchmark::CreateDenseRange(1, 2, /*step=*/1),
                   benchmark::CreateDenseRange(0, 1, /*step=*/1),
                   benchmark::CreateRange(10, 1000, /*multi=*/10),
                   benchmark::CreateRange(10, 10, /*multi=*/1),
                   benchmark::CreateDenseRange(0, 1, /*step=*/1)});
