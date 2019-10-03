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

#ifndef ENUM_ROS_MAPPER_H
#define ENUM_ROS_MAPPER_H

#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include <klepsydra/serialization/mapper.h>

namespace kpsr
{
template<class E>
/**
 * @brief The Mapper<E, std_msgs::Int32> class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-rosstg-serialization
 *
 */
class Mapper<E, std_msgs::Int32>
{
public:
    /**
     * @brief fromMiddleware
     * @param environment
     * @param message
     * @param event
     */
    virtual void fromMiddleware(const std_msgs::Int32& message, E& event) {
        event  = (E) message.data;
    }

    /**
     * @brief toMiddleware
     * @param environment
     * @param event
     * @param message
     */
    void toMiddleware(const E& event, std_msgs::Int32& message) {
        message.data = event;
    }
};
}
#endif//ENUM_ROS_MAPPER_H
