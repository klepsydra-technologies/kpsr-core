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

#ifndef MAPPER_H
#define MAPPER_H

namespace kpsr {
template<class KpsrClass, class MddlwClass>
/*!
 * @brief The Mapper class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-serialization
 *
 * @details Main interface to transform from and to a middleware into the Klepsydra realm. Implementations of this class are only needed for non-memory middlewares like ROS.
*/
class Mapper
{
public:
    /*!
     * @brief fromMiddleware
     * This method converts a middleware message (e.g., ROS message <a href="http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html">geometry_msgs::PoseStamped</a>)
     * into a Klepsydra realm <a href="https://en.wikipedia.org/wiki/Plain_Old_C%2B%2B_Object">POCO</a> (e.g., kpsr::geometry::PoseEventData)
     * @param message
     * @param event
     */
    virtual void fromMiddleware(const MddlwClass &message, KpsrClass &event) = 0;
    /*!
     * @brief toMiddleware
     * This method converts a Klepsydra realm <a href="https://en.wikipedia.org/wiki/Plain_Old_C%2B%2B_Object">POCO</a> (e.g., kpsr::geometry::PoseEventData)
     * into a middleware message (e.g., ROS message <a href="http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html">geometry_msgs::PoseStamped</a>)
     * @param event
     * @param message
     */
    virtual void toMiddleware(const KpsrClass &event, MddlwClass &message) = 0;
};
} // namespace kpsr
#endif
