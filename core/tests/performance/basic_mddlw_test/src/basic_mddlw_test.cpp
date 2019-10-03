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

#include <getopt.h>
#include <stdlib.h>

#include <klepsydra/mem_core/basic_middleware_provider.h>

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

    void test(int objectPoolSize, bool withInitializer) {

        int laserData = 50;

        std::function<void(kpsr::sensors::LaserScanEvent &)> laserScanInitFunc =
                [&laserData] (kpsr::sensors::LaserScanEvent & event) {
            event.ranges.resize(laserData);
            event.intensities.resize(laserData);
        };

        std::function<void(Point3dCloud &)> point3CloudInitFunc =
                [&laserData] (Point3dCloud & event) {
            event._values.resize(laserData, Point3dCloud::Point3d());
        };

        kpsr::mem::BasicMiddlewareProvider<kpsr::sensors::LaserScanEvent> provider(nullptr, "event", 4, objectPoolSize, withInitializer ? laserScanInitFunc : nullptr, nullptr, false);
        kpsr::mem::BasicMiddlewareProvider<Point3dCloud> newProvider(nullptr, "newEvent", 4, objectPoolSize, withInitializer ? point3CloudInitFunc : nullptr, nullptr, false);
        provider.start();
        newProvider.start();

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

        long before = kpsr::TimeUtils::getCurrentNanoseconds();

        for (int i = 0; i < 100; i++) {
            long before = kpsr::TimeUtils::getCurrentNanoseconds();
            std::shared_ptr<kpsr::sensors::LaserScanEvent> event(new kpsr::sensors::LaserScanEvent());
            event->ranges.resize(laserData);
            event->intensities.resize(laserData);
            event->angle_increment = M_PI/laserData;
            event->angle_min = - M_PI/2;
            event->angle_max = + M_PI/2;
            for (int j = 0; j < laserData; j ++) {
                event->ranges[j] = (double) i;
                event->intensities[j] = (double) i/60;
            }
            long after = kpsr::TimeUtils::getCurrentNanoseconds();
            totalConstructionTime += after - before;
            provider.getPublisher()->publish(* event.get());
        }

        while (!provider._internalQueue.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        while (!newProvider._internalQueue.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        provider.stop();
        newProvider.stop();

        long after = kpsr::TimeUtils::getCurrentNanoseconds();
        totalExecutionTime += after - before;
        totalForwardingTime = provider.getSubscriber()->getSubscriptionStats("forwarderListener")->_totalProcessingTimeInNanoSecs;
    }
};


int main(int argc, char **argv)
{
    int objectPoolSize = 0;
    bool withInitializer = false;

    char c;
    while ((c = getopt(argc, argv, "o:i")) != -1) {
        switch(c) {
        case 'o':
            objectPoolSize = atoi(optarg);
            break;
        case 'i':
            withInitializer = true;
            break;
        default:
            printf("Usage: \n");
            printf("    -o <object poolsize>   : default 0\n");
            printf("    -i <with init function> : default false\n");
            break;
        }
    }
    std::cout << "PerformanceTest::test/ objectPoolSize: " << objectPoolSize << " . withInitializer: " << (withInitializer ? "true" : "false") << std::endl;

    std::vector<long> totalExecutionTimes;
    std::vector<long> totalTransformationTimes;
    std::vector<long> totalConstructionTimes;
    std::vector<long> totalForwardingTimes;

    PerformanceTest performanceTest;

    for (int i = 0; i < 100; i ++) {
        performanceTest.test(objectPoolSize, withInitializer);
        totalExecutionTimes.push_back(performanceTest.totalExecutionTime);
        totalTransformationTimes.push_back(performanceTest.totalTransformationTime);
        totalConstructionTimes.push_back(performanceTest.totalConstructionTime);
        totalForwardingTimes.push_back(performanceTest.totalForwardingTime);
    }

    TestResults totalExecutionTimesResult = (totalExecutionTimes);
    TestResults totalTransformationTimesResult = (totalTransformationTimes);
    TestResults totalConstructionTimesResult = (totalConstructionTimes);
    TestResults totalForwardingTimesResult = (totalForwardingTimes);

    std::cout << "total execution time: " << totalExecutionTimesResult.average << " / " << totalExecutionTimesResult.stddev << std::endl;
    std::cout << "total transformation time: "  << totalTransformationTimesResult.average << " / " << totalTransformationTimesResult.stddev << std::endl;
    std::cout << "total construction time: "  << totalConstructionTimesResult.average << " / " << totalConstructionTimesResult.stddev << std::endl;
    std::cout << "total forwarding time: "  << totalForwardingTimesResult.average << " / " << totalForwardingTimesResult.stddev << std::endl;
}
