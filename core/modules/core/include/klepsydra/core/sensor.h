/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*                            All Rights Reserved.
*
*  This file is subject to the terms and conditions defined in
*  file 'LICENSE.md', which is part of this source code package.
*
*  NOTICE:  All information contained herein is, and remains the property of Klepsydra
*  Technologies GmbH and its suppliers, if any. The intellectual and technical concepts
*  contained herein are proprietary to Klepsydra Technologies GmbH and its suppliers and
*  may be covered by Swiss and Foreign Patents, patents in process, and are protected by
*  trade secret or copyright law. Dissemination of this information or reproduction of
*  this material is strictly forbidden unless prior written permission is obtained from
*  Klepsydra Technologies GmbH.
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
    Sensor(const std::string & frameId, int seq, long timestamp)
        : frameId(frameId)
        , seq(seq)
        , timestamp(timestamp)
    {}

    /*!
     * @brief Sensor
     * @param frameId
     * @param seq
     */
    Sensor(const std::string & frameId, int seq)
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
