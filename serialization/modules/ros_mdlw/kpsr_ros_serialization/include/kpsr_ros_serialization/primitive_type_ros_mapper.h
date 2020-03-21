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

#ifndef PRIMITIVE_TYPE_ROS_MAPPER_H
#define PRIMITIVE_TYPE_ROS_MAPPER_H

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"

#include <string>

#include <klepsydra/serialization/mapper.h>

namespace kpsr
{
template<>
/**
 * @brief The Mapper<bool, std_msgs::Bool> class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-rosstg-serialization
 *
 */
class Mapper<bool, std_msgs::Bool>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const std_msgs::Bool& message, bool& event) {
        event  = message.data;
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const bool& event, std_msgs::Bool& message) {
        message.data = event;
    }
};

template<>
/**
 * @brief The Mapper<int, std_msgs::Int32> class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-rosstg-serialization
 *
 */
class Mapper<int, std_msgs::Int32>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const std_msgs::Int32& message, int& event) {
        event  = message.data;
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const int& event, std_msgs::Int32& message) {
        message.data = event;
    }
};

template<>
/**
 * @brief The Mapper<long, std_msgs::Int64> class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-rosstg-serialization
 *
 */
class Mapper<long, std_msgs::Int64>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const std_msgs::Int64& message, long& event) {
        event  = message.data;
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const long& event, std_msgs::Int64& message) {
        message.data = event;
    }
};

template<>
/**
 * @brief The Mapper<float, std_msgs::Float32> class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-rosstg-serialization
 *
 */
class Mapper<float, std_msgs::Float32>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const std_msgs::Float32& message, float& event) {
        event  = message.data;
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const float& event, std_msgs::Float32& message) {
        message.data = event;
    }
};

template<>
/**
 * @brief The Mapper<double, std_msgs::Float64> class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-rosstg-serialization
 *
 */
class Mapper<double, std_msgs::Float64>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const std_msgs::Float64& message, double& event) {
        event  = message.data;
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const double& event, std_msgs::Float64& message) {
        message.data = event;
    }
};

template<>
/**
 * @brief The Mapper<std::string, std_msgs::String> class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-rosstg-serialization
 *
 */
class Mapper<std::string, std_msgs::String>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const std_msgs::String& message, std::string& event) {
        event  = message.data;
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const std::string& event, std_msgs::String& message) {
        message.data = event;
    }
};

}

#endif // PRIMITIVE_TYPE_ROS_MAPPER_H
