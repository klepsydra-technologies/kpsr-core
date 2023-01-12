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

#ifndef SENSOR_H
#define SENSOR_H

#include <chrono>
#include <string>

namespace kpsr {
/*!
 * @brief The Sensor struct
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-application
 *
 * @details This class is the base class for all sensor data available in Klepsydra. This includes: images,
 * positioning, laser data, etc.
 * It has a similar structure to the ROS <a href="http://wiki.ros.org/sensor_msgs">sensor_msgs Package</a>. However,
 * at the same time it contains the same API as most Klepsydra POCO including a clone method, a constructor
 * based on another instance and a constructor receiving all fields.
 */
struct Sensor
{
public:
    /*!
     * @brief Sensor
     */
    Sensor()
    {
        std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
        timestamp = ms.count();
    }

    /*!
     * @brief Sensor
     * @param frameId
     * @param seq
     * @param timestamp
     */
    Sensor(const std::string &frameId, int seq, long timestamp)
        : frameId(frameId)
        , seq(seq)
        , timestamp(timestamp)
    {}

    /*!
     * @brief Sensor
     * @param frameId
     * @param seq
     */
    Sensor(const std::string &frameId, int seq)
        : frameId(frameId)
        , seq(seq)
    {
        std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
        timestamp = ms.count();
    }

    /*!
     * @brief Sensor
     * @param that
     */
    Sensor(const Sensor &that)
        : frameId(that.frameId)
        , seq(that.seq)
        , timestamp(that.timestamp)
    {}

    /*!
     * @brief clone
     * @param that
     */
    void clone(const Sensor &that)
    {
        frameId = that.frameId;
        seq = that.seq;
        timestamp = that.timestamp;
    }

    /*!
     * @brief frameId
     */
    std::string frameId;

    /*!
     * @brief seq
     */
    int seq;

    /*!
     * @brief timestamp
     */
    long timestamp;
};
} // namespace kpsr
#endif
