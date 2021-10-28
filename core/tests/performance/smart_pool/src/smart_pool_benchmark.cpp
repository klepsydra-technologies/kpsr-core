#include <iostream>
#include <thread>

#include <benchmark/benchmark.h>

#include "point3d_cloud.h"
#include <klepsydra/core/smart_object_pool.h>

static void BM_SmartPoolAcquireInt(benchmark::State &state)
{
    // Perform setup here
    const int poolSize = 1024;

    kpsr::SmartObjectPool<int> smartPool("SmartPoolBenchmark", poolSize, nullptr);
    for (auto _ : state) {
        // This code gets timed
        std::shared_ptr<int> event = std::move(smartPool.acquire());
        // do nothing with event;
    }
}

static void BM_SharedObjectCreateInt(benchmark::State &state)
{
    // Perform setup here

    for (auto _ : state) {
        // This code gets timed
        std::shared_ptr<int> event = std::make_shared<int>();
        // do nothing with event;
    }
}

static void BM_SmartPoolAcquirePoint3D(benchmark::State &state)
{
    // Perform setup here
    const int poolSize = 1024;

    kpsr::SmartObjectPool<Point3dCloud> smartPool("SmartPoolBenchmark", poolSize, nullptr);
    for (auto _ : state) {
        // This code gets timed
        std::shared_ptr<Point3dCloud> event = std::move(smartPool.acquire());
        // do nothing with event;
    }
}

static void BM_SharedObjectCreatePoint3D(benchmark::State &state)
{
    // Perform setup here

    for (auto _ : state) {
        // This code gets timed
        std::shared_ptr<Point3dCloud> event = std::make_shared<Point3dCloud>();
        // do nothing with event;
    }
}

static void BM_SharedObjectCreateVector(benchmark::State &state)
{
    // Perform setup here
    const int vectorSize(state.range(0));
    for (auto _ : state) {
        // This code gets timed
        std::shared_ptr<std::vector<float>> event = std::make_shared<std::vector<float>>();
        event->resize(vectorSize);
        // do nothing with event;
    }
}

class SmartPoolFixture : public benchmark::Fixture
{
public:
    void SetUp(const ::benchmark::State &state)
    {
        if (state.thread_index() == 0) {
            assert(smartPool.get() == nullptr);
            const int poolSize = state.range(0);
            const int vectorSize(state.range(1));
            smartPool.reset(
                new kpsr::SmartObjectPool<std::vector<float>>("SmartPoolBenchmark",
                                                              poolSize,
                                                              [&](std::vector<float> &vec) {
                                                                  vec.resize(vectorSize);
                                                              }));
        }
    }

    void TearDown(const ::benchmark::State &state)
    {
        if (state.thread_index() == 0) {
            smartPool.reset();
        }
    }

    std::unique_ptr<kpsr::SmartObjectPool<std::vector<float>>> smartPool;
};

BENCHMARK_DEFINE_F(SmartPoolFixture, AcquireVector)(benchmark::State &state)
{
    // Perform setup here
    for (auto _ : state) {
        std::shared_ptr<std::vector<float>> event = std::move(smartPool->acquire());
    }
}

static void BM_SharedObjectCreateVectorOperate(benchmark::State &state)
{
    // Perform setup here
    const int vectorSize(state.range(0));
    for (auto _ : state) {
        // This code gets timed
        std::shared_ptr<std::vector<float>> event = std::make_shared<std::vector<float>>();
        event->resize(vectorSize);
        for (auto &i : *event) {
            benchmark::DoNotOptimize(i += 1);
        }
        // do nothing with event;
    }
}

BENCHMARK_DEFINE_F(SmartPoolFixture, VectorMultiThreadOperate)(benchmark::State &state)
{
    for (auto _ : state) {
        std::shared_ptr<std::vector<float>> event = std::move(smartPool->acquire());
        for (auto &i : *event) {
            benchmark::DoNotOptimize(i += 1);
        }
    }
}

// Register the function as a benchmark
BENCHMARK(BM_SmartPoolAcquireInt);
BENCHMARK(BM_SharedObjectCreateInt);
BENCHMARK(BM_SmartPoolAcquirePoint3D);
BENCHMARK(BM_SharedObjectCreatePoint3D);

BENCHMARK_REGISTER_F(SmartPoolFixture, AcquireVector)
    ->Threads(1)
    ->UseRealTime()
    ->Ranges({{128, 1 << 10}, {256, 256}});
BENCHMARK(BM_SharedObjectCreateVector)->Threads(1)->UseRealTime()->Range(128, 1 << 10);

// Multithreaded tests
BENCHMARK_REGISTER_F(SmartPoolFixture, AcquireVector)
    ->Threads(10)
    ->UseRealTime()
    ->Ranges({{128, 1 << 10}, {256, 256}});
BENCHMARK(BM_SharedObjectCreateVector)->Threads(10)->UseRealTime()->Range(128, 1 << 10);

BENCHMARK_REGISTER_F(SmartPoolFixture, VectorMultiThreadOperate)
    ->Threads(10)
    ->UseRealTime()
    ->Ranges({{128, 1 << 10}, {256, 1024}});
BENCHMARK(BM_SharedObjectCreateVectorOperate)->Threads(10)->UseRealTime()->DenseRange(256, 1024, 256);

// Run the benchmark
BENCHMARK_MAIN();
