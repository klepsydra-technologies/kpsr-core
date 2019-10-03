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
