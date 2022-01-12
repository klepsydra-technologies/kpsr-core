#include <random>

#include <benchmark/benchmark.h>
#include <spdlog/spdlog.h>

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
            container = std::unique_ptr<kpsr::Container>(
                new kpsr::Container(nullptr, "testContainer"));
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
    std::string label = eventEmitterTypeName + ",\t pool size: " + std::to_string(_poolSize) +
                        ",\t dataSize :" + std::to_string(_dataSize) +
                        ",\t num listeners: " + std::to_string(_numListeners);
    state.SetLabel(label);
}

BENCHMARK_REGISTER_F(EventEmitterPerformanceTest, PublishTest)
    ->ArgsProduct({benchmark::CreateDenseRange(0, 2, /*step=*/1),
                   benchmark::CreateDenseRange(0, 1, /*step=*/1),
                   benchmark::CreateRange(10, 1000, /*multi=*/10),
                   benchmark::CreateRange(1, 1, /*multi=*/1),
                   benchmark::CreateDenseRange(0, 1, /*step=*/1)});

BENCHMARK_REGISTER_F(EventEmitterPerformanceTest, PublishTest)
    ->ArgsProduct({benchmark::CreateDenseRange(1, 2, /*step=*/1),
                   benchmark::CreateDenseRange(0, 1, /*step=*/1),
                   benchmark::CreateRange(10, 1000, /*multi=*/10),
                   benchmark::CreateRange(10, 10, /*multi=*/1),
                   benchmark::CreateDenseRange(0, 1, /*step=*/1)});
