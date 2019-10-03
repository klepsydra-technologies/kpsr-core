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

#ifndef MAPPER_H
#define MAPPER_H

namespace kpsr
{
template <class KpsrClass, class MddlwClass>
/*!
 * @brief The Mapper class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-serialization
 *
 * @details Main interface to transform from and to a middleware into the Klepsydra realm. Implementations of this class are only needed for non-memory middlewares like ROS and DDS.
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
    virtual void fromMiddleware(const MddlwClass& message, KpsrClass& event) = 0;
    /*!
     * @brief toMiddleware
     * This method converts a Klepsydra realm <a href="https://en.wikipedia.org/wiki/Plain_Old_C%2B%2B_Object">POCO</a> (e.g., kpsr::geometry::PoseEventData)
     * into a middleware message (e.g., ROS message <a href="http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html">geometry_msgs::PoseStamped</a>)
     * @param event
     * @param message
     */
    virtual void toMiddleware(const KpsrClass& event, MddlwClass& message) = 0;
};
}
#endif
