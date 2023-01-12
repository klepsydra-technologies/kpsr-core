/*
 * Copyright 2023 Klepsydra Technologies AG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// This code has been automatically generated, manual modification might be inadvertently overridden.

#ifndef KPSR_SENSORS_LASER_SCAN_EVENT_H_
#define KPSR_SENSORS_LASER_SCAN_EVENT_H_

// Include section.
#include <vector>

#include <klepsydra/core/sensor.h>

namespace kpsr {
namespace sensors {
// Klepsydra generated event class.
class LaserScanEvent : public Sensor
{
public:
    static int constructorInvokations;
    static int emptyConstructorInvokations;
    static int copyConstructorInvokations;

    // Default constructor.
    LaserScanEvent() { LaserScanEvent::copyConstructorInvokations++; }

    // Main constructor.
    LaserScanEvent(std::string frameId,
                   int seq,
                   float angle_min,
                   float angle_max,
                   float angle_increment,
                   float time_increment,
                   float scan_time,
                   float range_min,
                   float range_max,
                   std::vector<float> ranges,
                   std::vector<float> intensities)
        : Sensor(frameId, seq)
        , angle_min(angle_min)
        , angle_max(angle_max)
        , angle_increment(angle_increment)
        , time_increment(time_increment)
        , scan_time(scan_time)
        , range_min(range_min)
        , range_max(range_max)
        , ranges(ranges)
        , intensities(intensities)
    {
        LaserScanEvent::constructorInvokations++;
    }

    // Clone constructor. Needed by klepsydra core APIs.
    LaserScanEvent(const LaserScanEvent &that)
        : angle_min(that.angle_min)
        , angle_max(that.angle_max)
        , angle_increment(that.angle_increment)
        , time_increment(that.time_increment)
        , scan_time(that.scan_time)
        , range_min(that.range_min)
        , range_max(that.range_max)
        , ranges(that.ranges)
        , intensities(that.intensities)
    {
        LaserScanEvent::copyConstructorInvokations++;
    }

    // Clone method. Needed by klepsydra core APIs.
    void clone(const LaserScanEvent &that)
    {
        Sensor::clone(that);
        this->angle_min = that.angle_min;
        this->angle_max = that.angle_max;
        this->angle_increment = that.angle_increment;
        this->time_increment = that.time_increment;
        this->scan_time = that.scan_time;
        this->range_min = that.range_min;
        this->range_max = that.range_max;
        this->ranges = that.ranges;
        this->intensities = that.intensities;
    }

    // List of fields.
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    std::vector<float> ranges;
    std::vector<float> intensities;
};
} // namespace sensors
} // namespace kpsr

int kpsr::sensors::LaserScanEvent::constructorInvokations = 0;
int kpsr::sensors::LaserScanEvent::emptyConstructorInvokations = 0;
int kpsr::sensors::LaserScanEvent::copyConstructorInvokations = 0;

#endif
