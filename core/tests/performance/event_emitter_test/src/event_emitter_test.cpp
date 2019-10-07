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
#include <math.h>

#include <sstream>
#include <fstream>
#include <math.h>
#include <random>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

#include <klepsydra/core/event_emitter_middleware_provider.h>
#include <klepsydra/core/cache_listener.h>

#include "laser_scan_event.h"
#include "point3d_cloud.h"
#include "test_result_data.h"

#include "gtest/gtest.h"

class PerformanceTest {
public:
    long totalExecutionTime = 0;
    long totalTransformationTime = 0;
    long totalConstructionTime = 0;
    long totalForwardingTime = 0;
    long totalCacheListenerTime = 0;

    void test() {
        int laserData = 50;
        kpsr::EventEmitterMiddlewareProvider<kpsr::sensors::LaserScanEvent> provider(nullptr, "event", 0, nullptr, nullptr);
        kpsr::EventEmitterMiddlewareProvider<Point3dCloud> newProvider(nullptr, "newEvent", 0, nullptr, nullptr);

        long before = kpsr::TimeUtils::getCurrentNanoseconds();

        std::function<void(const kpsr::sensors::LaserScanEvent &, Point3dCloud &)> transformFunction = [this] (const kpsr::sensors::LaserScanEvent & src, Point3dCloud & dest) {
            long before = kpsr::TimeUtils::getCurrentNanoseconds();
            dest._values.resize(src.ranges.size());
            dest._label = "hola";
            double yaw = src.angle_min;
            for (int i = 0; i < src.ranges.size(); i++) {
                dest._values[i].x = src.ranges[i] * std::cos(yaw);
                dest._values[i].y = src.ranges[i] * std::sin(yaw);
                dest._values[i].z = 0;
                yaw += src.angle_increment;
            }
            long after = kpsr::TimeUtils::getCurrentNanoseconds();
            this->totalTransformationTime += after - before;
        };

        auto forwarder = newProvider.getProcessForwarder(transformFunction);
        provider.getSubscriber()->registerListener("forwarderListener", forwarder->forwarderListenerFunction);

        kpsr::mem::CacheListener<Point3dCloud> eventListener;
        newProvider.getSubscriber()->registerListener("cacheListener", eventListener.cacheListenerFunction);

        std::random_device rd;  //Will be used to obtain a seed for the random number engine
        std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
        std::uniform_real_distribution<> distribution(1.0, 60.0);

        for (int i = 0; i < 1000; i++) {
            long before = kpsr::TimeUtils::getCurrentNanoseconds();
            kpsr::sensors::LaserScanEvent event;
            event.angle_increment = M_PI/laserData;
            event.angle_min = - M_PI/2;
            event.angle_max = + M_PI/2;
            event.ranges.resize(laserData);
            event.intensities.resize(laserData);
            for (int j = 0; j < laserData; j ++) {
                event.ranges[j] = distribution(gen);
                event.intensities[j] = distribution(gen)/60;
            }
            long after = kpsr::TimeUtils::getCurrentNanoseconds();
            totalConstructionTime += after - before;
            provider.getPublisher()->publish(event);
        }

        long after = kpsr::TimeUtils::getCurrentNanoseconds();
        totalExecutionTime += after - before;
        totalForwardingTime = provider.getSubscriber()->getSubscriptionStats("forwarderListener")->_totalProcessingTimeInNanoSecs;
        totalCacheListenerTime = eventListener.totalCopyingTime;
    }
};

int main(int argc, char **argv)
{
    std::vector<long> totalExecutionTimes;
    std::vector<long> totalTransformationTimes;
    std::vector<long> totalConstructionTimes;
    std::vector<long> totalForwardingTimes;
    std::vector<long> totalCacheListenerTimes;

    PerformanceTest performanceTest;

    for (int i = 0; i < 100; i ++) {
        performanceTest.test();
        totalExecutionTimes.push_back(performanceTest.totalExecutionTime);
        totalTransformationTimes.push_back(performanceTest.totalTransformationTime);
        totalConstructionTimes.push_back(performanceTest.totalConstructionTime);
        totalForwardingTimes.push_back(performanceTest.totalForwardingTime);
        totalCacheListenerTimes.push_back(performanceTest.totalCacheListenerTime);
    }

    TestResults totalExecutionTimesResult = (totalExecutionTimes);
    TestResults totalTransformationTimesResult = (totalTransformationTimes);
    TestResults totalConstructionTimesResult = (totalConstructionTimes);
    TestResults totalForwardingTimesResult = (totalForwardingTimes);
    TestResults totalCacheListenerTimesResult = (totalCacheListenerTimes);

    spdlog::info("total execution time: {} / {}", totalExecutionTimesResult.average, totalExecutionTimesResult.stddev);
    spdlog::info("total transformation time: {} / {}", totalTransformationTimesResult.average, totalTransformationTimesResult.stddev);
    spdlog::info("total construction time: {} / {}", totalConstructionTimesResult.average, totalConstructionTimesResult.stddev);
    spdlog::info("total forwarding time: {} / {}", totalForwardingTimesResult.average, totalForwardingTimesResult.stddev);
    spdlog::info("total cacheListener time: {} / {}", totalCacheListenerTimesResult.average, totalCacheListenerTimesResult.stddev);
}
