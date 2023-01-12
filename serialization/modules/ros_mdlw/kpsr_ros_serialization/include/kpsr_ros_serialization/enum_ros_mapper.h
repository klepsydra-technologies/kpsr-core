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

#ifndef ENUM_ROS_MAPPER_H
#define ENUM_ROS_MAPPER_H

#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include <klepsydra/serialization/mapper.h>

namespace kpsr {
template<class E>
/**
 * @brief The Mapper<E, std_msgs::Int32> class
 *
 * @copyright 2023 Klepsydra Technologies AG
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
    virtual void fromMiddleware(const std_msgs::Int32 &message, E &event)
    {
        event = (E) message.data;
    }

    /**
     * @brief toMiddleware
     * @param environment
     * @param event
     * @param message
     */
    void toMiddleware(const E &event, std_msgs::Int32 &message) { message.data = event; }
};
} // namespace kpsr
#endif //ENUM_ROS_MAPPER_H
