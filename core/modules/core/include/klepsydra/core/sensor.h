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

#ifndef SENSOR_H
#define SENSOR_H

#include <string>
#include <chrono>

namespace kpsr
{
/*!
 * @brief The Sensor struct
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
    Sensor() {
        std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());
        timestamp = ms.count();
    }

    /*!
     * @brief Sensor
     * @param frameId
     * @param seq
     * @param timestamp
     */
    Sensor(std::string frameId, int seq, long timestamp)
        : frameId(frameId)
        , seq(seq)
        , timestamp(timestamp)
    {}

    /*!
     * @brief Sensor
     * @param frameId
     * @param seq
     */
    Sensor(std::string frameId, int seq)
        : frameId(frameId)
        , seq(seq)
    {
        std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());
        timestamp = ms.count();
    }

    /*!
     * @brief Sensor
     * @param that
     */
    Sensor(const Sensor & that)
        : frameId(that.frameId)
        , seq(that.seq)
        , timestamp(that.timestamp)
    {}

    /*!
     * @brief clone
     * @param that
     */
    void clone(const Sensor & that) {
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
}
#endif
